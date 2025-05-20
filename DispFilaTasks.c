#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "pio_matrix.pio.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27
#define LED_BLUE 12
#define LED_GREEN  11
#define tam_quad 10

// Struct para armazenar os dados do joystick
typedef struct {
    uint16_t x_pos;
    uint16_t y_pos;
} joystick_data_t;

// Criação da fila para o joystick
QueueHandle_t xQueueJoystickData;
// Variáveis de estado crítico que controlam o fluxo do programa
bool estadoCriticoVolumeAgua = false, estadoCriticoChuva = false;

//------------- Inicio das estruturas e funções usadas pra matriz de leds -------------//
typedef struct {
  double r;
  double g;
  double b;
} Pixel;

// Padrão a ser exibido na matriz de leds
Pixel desenho[25] = { 
    {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0},
    {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0},
    {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0},
    {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0},
    {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}, {0.01, 0.0, 0.0}
};

//rotina para definição da intensidade de cores do led
uint32_t matrix_rgb(double b, double r, double g) {
  unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

//rotina para acionar a matrix de leds - ws2812b
void desenho_pio(PIO pio, uint sm) {
    uint32_t valor_led;

    for (int16_t i = 0; i < 25; i++) {
        valor_led = matrix_rgb(
        desenho[24-i].b,  // azul
        desenho[24-i].r,  // vermelho
        desenho[24-i].g   // verde
        );

        // Acende os leds somente se o estado crítico de volume de água ou chuva estiver ativo
        // Caso contrário, apaga os leds
        pio_sm_put_blocking(pio, sm, estadoCriticoChuva||estadoCriticoVolumeAgua?valor_led:0);
    }
}
//------------- Fim das estruturas e funções usadas pra matriz de leds -------------//

// Tarefa de controle da matriz de LEDs
void vLedMatrixTask() {
    //configurações da PIO
    PIO pio = pio0;
    set_sys_clock_khz(128000, false);
    uint offset = pio_add_program(pio, &pio_matrix_program);
    uint sm = pio_claim_unused_sm(pio, true);
    pio_matrix_program_init(pio, sm, offset, 7);

    while(1) {
        desenho_pio(pio, sm); // Chama a função para desenhar na matriz
        vTaskDelay(1); // Yield para outras tarefas
    }
}

void vJoystickTask(void *params) {
    // Inicializa o ADC
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_init();

    // Estrutura que guarda as leituras do joystick
    joystick_data_t joydata;

    while (true) {
        adc_select_input(0); // GPIO 26 = ADC0
        joydata.y_pos = adc_read();

        adc_select_input(1); // GPIO 27 = ADC1
        joydata.x_pos = adc_read();

        xQueueSend(xQueueJoystickData, &joydata, 0); // Envia o valor do joystick para a fila
        vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura
    }
}

void vDisplayTask(void *params) {
    joystick_data_t joydata; // Guarda os valores lidos do joystick e recuperados da fila
    bool cor = true; // Usado para controle do display
    char str[40]; // Buffer para guardar as strings que serão exibidas no display

    // Inicializa o display OLED
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Esse while controla o display e atualiza as flags de controle que são "ouvidas" no resto do código
    while (true) {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE) {
            // O joystick tem uma faixa de 0 a 4095, então convertemos para porcentagem
            float x = joydata.x_pos*100/4095.0;
            float y = joydata.y_pos*100/4095.0;

            ssd1306_fill(&ssd, !cor); // Limpa a tela
            sprintf(str, "Agua: %.0f", x);
            ssd1306_draw_string(&ssd, str, 0, 0); // Escreve o conteúdo do eixo x na tela
            sprintf(str, "Chuvas: %.0f", y);
            ssd1306_draw_string(&ssd, str, 0, 12); // Escreve o conteúdo do eixo y na tela

            // Verifica se os valores dos sensores estão acima do limite e modifica as flags de controle
            if(x>70) estadoCriticoVolumeAgua = true; else estadoCriticoVolumeAgua = false; 
            if(y>80) estadoCriticoChuva = true; else estadoCriticoChuva = false;

            // Caso as flags indiquem perigo, mostra alerta no display
            if(estadoCriticoVolumeAgua || estadoCriticoChuva) ssd1306_draw_string(&ssd, "ALERTA!", 40, 40);

            // Envia os dados para serem exibidos no display
            ssd1306_send_data(&ssd);
        }
    }
}

void vLedTask(void *params) {
    gpio_init(13); // Inicializa o LED
    gpio_set_dir(13, GPIO_OUT); // Configura como saída

    joystick_data_t joydata;
    while (true) {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE) {
            // Desliga ou liga o LED vermelho com base nos estados críticos
            if (estadoCriticoVolumeAgua || estadoCriticoChuva) gpio_put(13, 1); else gpio_put(13, 0);
        }
        vTaskDelay(1); // Atualiza a cada 50ms
    }
}

void vBuzzerTask(void *params) {
    // Variáveis de controle do buzzer
    static int freq = 0;
    bool subindo = true;

    // Configura o pino para a função PWM
    gpio_set_function(21, GPIO_FUNC_PWM);

    // Obtém o slice de PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(21);

    // Configura o PWM com a frequência desejada
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f); // Ajusta o divisor de clock conforme necessário
    // pwm_init(slice_num, &config, true);

    // Define o nível inicial do PWM (0 desliga o buzzer)
    pwm_set_gpio_level(21, 0);

    while(true) {
        // Entramos nesse if caso o volume de água ou a chuva estejam acima do limite
        if(estadoCriticoVolumeAgua || estadoCriticoChuva) {
            // Atualiza direção da variação de frequência
            if (subindo) {
                freq += 50;
                if (freq >= 2000) subindo = false;
            } else {
                freq -= 50;
                if (freq <= 0) subindo = true;
            }

            gpio_set_function(21, GPIO_FUNC_PWM);
            pwm_set_enabled(pwm_gpio_to_slice_num(21), true);
            uint32_t clock = clock_get_hz(clk_sys);
            uint32_t divider16 = (clock << 4) / (2000+freq);
            uint32_t wrap = (divider16 + (1 << 4) - 1) >> 4;

            pwm_set_wrap(slice_num, wrap);
            pwm_set_chan_level(slice_num, pwm_gpio_to_channel(21), wrap / 8); // 12,5% duty cycle
            pwm_set_enabled(slice_num, true);
            vTaskDelay(estadoCriticoChuva?pdMS_TO_TICKS(35):pdMS_TO_TICKS(10)); // Aguarda 50ms ou 100ms dependendo da flag ativa
        } else {
            // Se não houver estado crítico, desliga o buzzer
            pwm_set_gpio_level(21, 0);
            pwm_set_enabled(pwm_gpio_to_slice_num(21), false);
            gpio_set_function(21, GPIO_FUNC_SIO);
            gpio_set_dir(21, GPIO_OUT);
            gpio_put(21, 0);
        }
    }
}

// Modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events) {
    reset_usb_boot(0, 0);
}

int main() {
    // Ativa BOOTSEL via botão
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();

    // Cria a fila para compartilhamento de valor do joystick
    xQueueJoystickData = xQueueCreate(5, sizeof(joystick_data_t));

    // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 512, NULL, 1, NULL);
    xTaskCreate(vLedTask, "LED red Task", 256, NULL, 1, NULL);
    xTaskCreate(vLedMatrixTask, "Matriz de leds Task", 256, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 1, NULL);

    // Inicia o agendador
    vTaskStartScheduler();
    panic_unsupported();
}
