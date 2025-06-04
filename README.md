# Monitor Remoto de Sinais de Alerta SIRS com MQTT na BitDogLab (FreeRTOS)

Este projeto implementa um sistema de monitoramento remoto para simular a avaliação de Sinais de Resposta Inflamatória Sistêmica (SIRS) utilizando o microcontrolador **RP2040** (na placa BitDogLab), o sistema operacional de tempo real **FreeRTOS**, e o protocolo **MQTT** para comunicação. O sistema permite a alteração de estados simulados via botão físico e remotamente, com visualização de dados e alertas em um painel de controle.

## Periféricos Utilizados (BitDogLab)

* **Botão de Usuário (`BUTTON_A_PIN`)** – Simula a progressão entre diferentes estados de alerta SIRS.
* **LEDs de Alerta (PWM nos pinos `LED_RED_PIN` e `LED_GREEN_PIN`)** – Sinalização visual do estado atual da simulação SIRS.
* **LED Onboard (Pico W)** – Indica o status da conexão MQTT e pode ser controlado remotamente.
* **Módulo Wi-Fi (CYW43)** – Para conexão à rede e comunicação MQTT.

## Funcionalidades

* Estrutura **multitarefa com FreeRTOS**, utilizando tarefas dedicadas para:
    * Gerenciamento da lógica do botão e publicação dos dados simulados.
    * Controlo dos LEDs de alerta com base no estado da simulação.
    * Manutenção do loop principal do cliente MQTT e polling da interface de rede.
* **Simulação de Sinais Vitais:** Temperatura e Batimentos Por Minuto (BPM) são simulados e seus valores alterados com base no estado atual.
* **Comunicação MQTT:**
    * Publicação dos dados simulados (temperatura, BPM, estado da simulação) para um broker MQTT.
    * Subscrição a tópicos para controlo remoto do LED onboard e do estado da simulação SIRS.
* **Controlo de Estado Duplo:** O estado da simulação SIRS pode ser alterado tanto por um botão físico na placa quanto por comandos MQTT enviados remotamente.
* **Feedback Visual:**
    * LEDs de alerta indicam o nível de criticidade da simulação.
    * LED onboard indica o status da conexão MQTT.
* **Proteção de Acesso Concorrente:** Utilização de mutex (`shared_data_mutex`) para proteger o acesso às variáveis globais de simulação.
* **Last Will and Testament (LWT):** Configurado para indicar o status "offline" do dispositivo caso a conexão seja perdida abruptamente.

## Estados da Simulação SIRS (Indicados pelos LEDs de Alerta)

1.  **Normal**
    * LEDs: **Verde**
    * Dados: Temperatura e BPM normais.
2.  **BPM Alto OU Temperatura Alta**
    * LEDs: **Laranja/Amarelo** (combinação de vermelho e verde)
    * Dados: Um dos parâmetros (BPM ou Temperatura) está elevado.
3.  **Crítico (Ambos Altos)**
    * LEDs: **Vermelho**
    * Dados: Ambos os parâmetros (BPM e Temperatura) estão elevados.

## Destaque Técnico

* Sistema construído sobre **FreeRTOS**, com uma arquitetura modular de tarefas.
* Implementação de um cliente **MQTT** para comunicação IoT, utilizando a pilha LwIP.
* Uso de primitivas de sincronização do FreeRTOS:
    * `xSemaphoreCreateMutex()` para garantir acesso seguro e exclusivo às variáveis de simulação.
* Lógica de debounce para o botão físico.
* Geração de Client ID MQTT único baseado no ID da placa para permitir múltiplas instâncias (opcional, via `MQTT_UNIQUE_TOPIC`).

## Componentes do Sistema IoT

1.  **Placa BitDogLab (Cliente MQTT 01):**
    * Executa o firmware com FreeRTOS.
    * Simula os dados SIRS e interage com os periféricos.
    * Publica dados e subscreve a comandos via MQTT.
2.  **Broker MQTT (Celular Android com Mosquitto e Termux):**
    * Atua como servidor intermediário para as mensagens MQTT.
    * Requer autenticação (nome de utilizador e senha).
3.  **Painel de Controlo (PC com IoT MQTT Panel - Cliente MQTT 02):**
    * Conecta-se ao broker para visualizar os dados publicados pela BitDogLab.
    * Permite enviar comandos para a BitDogLab (ex: alterar estado SIRS, controlar LED onboard).

## Software e Ferramentas

* **FreeRTOS Kernel**
* **Pico SDK** para acesso ao hardware do RP2040
* **LwIP MQTT Client**
* Compilador C (GCC para ARM)
* **CMake** para o sistema de build
* **Mosquitto MQTT Broker** (executando no Android via Termux)
* **Termux** (emulador de terminal para Android)
* **IoT MQTT Panel** (ou outro cliente MQTT para PC/Mobile)
* **VS Code** com a extensão Raspberry Pi Pico (recomendado)

## Como Configurar e Executar o Projeto

1.  **Clone o Repositório:**
    ```bash
    git clone <url_do_seu_repositorio>
    cd <nome_do_seu_repositorio>
    ```

2.  **Configure o Ambiente de Desenvolvimento do Pico:**
    * Certifique-se de que o Pico SDK está instalado e as variáveis de ambiente (`PICO_SDK_PATH`) estão configuradas.
    * Verifique as configurações do FreeRTOS no `CMakeLists.txt` se necessário.

3.  **Atualize as Credenciais no Código C (`seu_arquivo.c`):**
    * Defina `WIFI_SSID` e `WIFI_PASSWORD` com os dados da sua rede Wi-Fi.
    * Defina `MQTT_SERVER` com o endereço IP do seu celular (onde o Mosquitto está a ser executado).
    * Defina `MQTT_USERNAME` e `MQTT_PASSWORD` com as credenciais configuradas no seu broker Mosquitto.

4.  **Compile o Projeto:**
    ```bash
    mkdir build
    cd build
    cmake ..
    make 
    ```

5.  **Carregue o Firmware na BitDogLab:**
    * Conecte sua placa RP2040 ao computador enquanto segura o botão BOOTSEL.
    * Arraste o arquivo `.uf2` gerado (localizado em `build/nome_do_seu_projeto.uf2`) para o dispositivo de armazenamento massivo que aparece (RPI-RP2).

6.  **Configure e Execute o Broker Mosquitto no Android (via Termux):**
    * Instale o Mosquitto no Termux: `pkg install mosquitto`
    * Crie um arquivo de senhas (ex: `pwfile.txt`): `mosquitto_passwd -c pwfile.txt SEU_USERNAME_MQTT` (ser-lhe-á pedida a senha).
    * Crie um arquivo de configuração `mosquitto.conf` (ex: em `/data/data/com.termux/files/usr/etc/mosquitto/mosquitto.conf`) com o seguinte conteúdo (ajuste o caminho do `password_file`):
        ```conf
        listener 1883 0.0.0.0
        allow_anonymous false
        password_file /caminho/completo/para/seu/pwfile.txt 
        ```
    * Inicie o Mosquitto: `mosquitto -c /caminho/para/seu/mosquitto.conf -v`
    * Use `termux-wake-lock` para manter o Termux ativo.

7.  **Configure o IoT MQTT Panel (ou outro cliente MQTT):**
    * Conecte-se ao endereço IP do seu celular (Broker) na porta 1883, usando o mesmo `MQTT_USERNAME` e `MQTT_PASSWORD`.
    * **Subscreva aos tópicos (para visualizar):**
        * `/sirs/temperatura`
        * `/sirs/bpm`
        * `/sirs/estado`
        * `/status`
        * (Se `MQTT_UNIQUE_TOPIC` for `0`, os tópicos serão `/sirs/temperatura`, etc.)
    * **Publique em tópicos (para controlar):**
        * `/led` (Payload: "ON" ou "OFF")
        * `/sirs/set_estado_remoto` (Payload: "0", "1", "2" ou "3")

8.  **Teste:**
    * Pressione o botão na BitDogLab para ciclar os estados da simulação SIRS.
    * Envie comandos do IoT MQTT Panel para alterar o estado SIRS remotamente e controlar o LED onboard.
    * Observe os dados publicados no IoT MQTT Panel e o comportamento dos LEDs de alerta na BitDogLab.

## Tópicos MQTT Principais

* **Publicados pela Placa:**
    * `/sirs/temperatura`: Valor da temperatura simulada.
    * `/sirs/bpm`: Valor do BPM simulado.
    * `/sirs/estado`: Descrição do estado atual da simulação (ex: "Normal (Botao)").
    * `/status`: "online" ou "offline" (LWT).
    * `/uptime`: Resposta a um comando de ping.
* **Subscritos pela Placa:**
    * `/led`: Controla o LED onboard (Payload: "ON"/"OFF").
    * `/sirs/set_estado_remoto`: Define o estado da simulação remotamente (Payload: "0"-"3").
    * `/print`: Mensagem a ser impressa no console serial do Pico.
    * `/ping`: Dispara uma resposta de uptime.
    * `/exit`: Solicita a desconexão do cliente MQTT.

## Vídeo de Demonstração

🎥 [Link para o vídeo de demonstração do projeto](https://www.youtube.com/watch?v=IHA5TyrxhRM)

## Licença

Este projeto pode ser utilizado livremente para fins educacionais e não comerciais.
