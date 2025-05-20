<h1>
  <p align="center" width="100%">
    <img width="30%" src="https://softex.br/wp-content/uploads/2024/09/EmbarcaTech_logo_Azul-1030x428.png">
  </p>
</h1>

# ✨Tecnologias
Esse projeto foi desenvolvido com as seguintes tecnologias.
- Placa Raspberry Pi Pico W
- Raspberry Pi Pico SDK
- C/C++

# 💻Projeto
Projeto Desenvolvido durante a residência em microcontrolados e sistemas embarcados para estudantes de nível superior ofertado pela CEPEDI e SOFTEX, polo Juazeiro-BA, na Universidade Federal do Vale do São Francisco (UNIVASF), que tem como objetivo simular uma estação de monitoramento de enchentes, utilizando a placa BitDogLab com Raspberry PI-Pico, e fortalecer o aprendizado sobre sensores e filas na plataforma supracitada.

# 🚀Como rodar
### **Softwares Necessários**
1. **VS Code** com a extensão **Raspberry Pi Pico** instalada.
2. **CMake** e **Ninja** configurados.
3. **SDK do Raspberry Pi Pico** corretamente configurado.

### **Clonando o Repositório**
Para começar, clone o repositório no seu computador:
```bash
git clone https://github.com/DevMaic/Monitoramento_de_enchentes-Raspberry-PI
cd Monitoramento_de_enchentes-Raspberry-PI
```
---


### **Execução na Placa BitDogLab**
#### **1. Upload de Arquivo `DispFilasTasks.uf2`**
1. Importe o projeto utilizando a extensão do VSCode, e o compile.
2. Abra a pasta `build` que será gerada na compilação.
3. Aperte o botão **BOOTSEL** no microcontrolador Raspberry Pi Pico W.
4. Ao mesmo tempo, aperte o botão de **Reset**..
5. Mova o arquivo `DispFilaTasks.uf2` para a placa de desenvolvimento.
#### **2. Acompanhar Execução do Programa**
1. Utilizando o joystick é possível varias os valores de nível de agua e quantidade de chuva.
2. No eixo x é possível simular o nível da agua, e no eixo y a quantidade de chuva.
3. Caso o nível da agua ultrapasse 70% ou de chuva 80%, alarmes sonoros e visuais serão exibidos.
   
