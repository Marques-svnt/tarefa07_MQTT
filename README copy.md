# Sistema de Monitoramento de Sinais Vitais com Acesso Web e FreeRTOS

Este projeto implementa um **sistema embarcado inteligente** utilizando o microcontrolador **RP2040** da placa BitDogLab, o sistema operacional **FreeRTOS** e conectividade Wi-Fi para **monitoramento de sinais vitais** e exibição remota via navegador.

## Periféricos Utilizados

* **Sensor MAX30102** – frequência cardíaca (**não integrado ainda**)
* **Sensor DS18B20** – temperatura corporal (**não integrado ainda**)
* **LED** de status (GPIO)
* **Botão físico** (GPIO com interrupção e debounce)
* **Conexão Wi-Fi** (módulo CYW43 da BitDogLab)

## Funcionalidades

* Estrutura multitarefa com **FreeRTOS**.
* Criação de **servidor web embutido**, acessível por IP local.
* Interface via navegador com leitura simulada dos sinais.
* **Botão físico** com debounce para interações futuras.
* Arquitetura pronta para integração com sensores reais.

## Modos de Operação

1. **Modo de Monitoramento**
   
   * Exibe dados (simulados) de **frequência cardíaca** e **temperatura**.
   * Acessível via navegador em qualquer dispositivo da mesma rede.
   * LED pisca como indicação de funcionamento do sistema.

## Botões

* **Botão A**: reservado para simulação dos sensores (com debounce de 300ms).

## Hardware Utilizado

* **RP2040** (BitDogLab)
* Sensor **MAX30102** (*a integrar*)
* Sensor **DS18B20** (*a integrar*)
* LED de status
* Botão físico

## Software e Ferramentas

* **FreeRTOS** para multitarefa.
* **SDK da Raspberry Pi Pico**.
* **LwIP** para conexão de rede.
* **CMake** e **Ninja** para build.

## Como Executar o Projeto

1. **Clone** o repositório:

   ```bash
   git clone <seu_repositorio_url>
   cd <nome_do_repositorio>
   ```

2. **Importe** no VS Code (extensão Raspberry Pi Pico).

3. **Ajuste** o caminho do FreeRTOS em `CMakeLists.txt`:

   ```cmake
   set(FREERTOS_PATH "caminho/para/freertos/source")
   ```

4. **Compile**:

   ```bash
   mkdir build && cd build
   cmake -G Ninja ..
   ninja
   ```

5. **Grave** na placa e **execute**.

6. **Acesse o IP** exibido no terminal pelo navegador para visualizar os dados.

## Vídeo de Demonstração

* [\[Link para o vídeo no YouTube\]](https://www.youtube.com/watch?v=YVdxUgJbh8k)

## Licença

Este projeto é **código aberto** e pode ser usado e modificado livremente para fins educacionais e não comerciais.