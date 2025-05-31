#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"  // Para controle de LED com PWM
#include "hardware/gpio.h" // Para GPIOs genéricos

#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/netif.h"

// Includes do FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" // Para semáforos

// Credenciais WIFI - Tome cuidado se publicar no github!
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

// Definição dos pinos
#define ONBOARD_LED_PIN CYW43_WL_GPIO_LED_PIN // LED onboard da Pico W
#define LED_GREEN_PIN 11                      // GPIO11 - LED verde (usado para laranja com PWM)
#define LED_RED_PIN 13                        // GPIO13 - LED vermelho (usado para laranja e vermelho com PWM)
#define BUTTON_A_PIN 5                        // GPIO5 - Botão A para simulação
#define ADC_HEART_PIN 26

// Constantes para simulação e alertas
#define TEMPERATURA_NORMAL 25.0f
#define TEMPERATURA_ALTA 38.5f
#define TEMPERATURA_SIRS 38.0f

#define BPM_NORMAL 70.0f
#define BPM_ALTO 120.0f
#define BPM_SIRS 90.0f

#define PWM_WRAP_VALUE 255 // Para controle PWM de 8 bits

// Variáveis globais para dados dos sensores (simulados) e estado
volatile float temp_sim = TEMPERATURA_NORMAL;
volatile float bpm_sim = BPM_NORMAL;
volatile int estado_temp = 0; // 0: normal, 1: alta

// Handle para o Mutex
SemaphoreHandle_t sensor_data_mutex;

// Handles das Tasks
TaskHandle_t heart_sensor_task_handle;
TaskHandle_t button_task_handle;
TaskHandle_t led_alert_task_handle;
TaskHandle_t network_poll_task_handle;
TaskHandle_t tcp_server_task_handle; // Servidor TCP for executado como uma tarefa

// --- Protótipos das Funções ---
void init_gpios_pwm(void);

// Callbacks LwIP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
void launch_web_server(void);

// Tasks do FreeRTOS
void heart_sensor_task(void *pvParameters);
void button_task(void *pvParameters);
void led_alert_task(void *pvParameters);
void network_poll_task(void *pvParameters);

// --- Inicializações ---
void init_gpios_pwm(void)
{
    // Botão A
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN); // Usar pull-up interno, botão conecta ao GND

    // LEDs para PWM
    uint led_pins[] = {LED_RED_PIN, LED_GREEN_PIN};
    for (int i = 0; i < sizeof(led_pins) / sizeof(led_pins[0]); ++i)
    {
        uint gpio = led_pins[i];
        gpio_set_function(gpio, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(gpio);

        pwm_config config = pwm_get_default_config();
        pwm_config_set_wrap(&config, PWM_WRAP_VALUE); // Define o ciclo máximo do PWM
        pwm_init(slice_num, &config, true);           // Inicia o PWM, true para habilitar
        pwm_set_gpio_level(gpio, 0);                  // Começa com LED desligado
    }
    // ADC for heart sensor
    adc_init();
    adc_gpio_init(ADC_HEART_PIN);
}

// --- Tasks do FreeRTOS ---

// Tarefa para simular leitura do sensor cardíaco 
void heart_sensor_task(void *pvParameters)
{
    (void)pvParameters;
    while (1)
    {
        uint16_t raw = adc_read();
        float voltage = raw * (3.3f / (1 << 12));
        // Map voltage [0.5V..2.5V] to BPM [50..150]
        float bpm = ((voltage - 0.5f) * (100.0f / 2.0f)) + 50.0f;
        if (bpm < 0)
            bpm = 0;
        if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE)
        {
            bpm_sim = bpm;
            xSemaphoreGive(sensor_data_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Tarefa para simular a temperatura pelo botão
void button_task(void *pvParameters)
{
    (void)pvParameters;
    bool last = true;
    TickType_t last_time = 0;
    while (1)
    {
        bool curr = gpio_get(BUTTON_A_PIN);
        if (!curr && last)
        {
            if (xTaskGetTickCount() - last_time > pdMS_TO_TICKS(200))
            {
                last_time = xTaskGetTickCount();
                if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE)
                {
                    estado_temp = (estado_temp + 1) % 2;
                    switch (estado_temp)
                    {
                    case 0:
                        temp_sim = TEMPERATURA_NORMAL;
                        break;
                    case 1:
                        temp_sim = TEMPERATURA_ALTA;
                        break;
                    }
                    xSemaphoreGive(sensor_data_mutex);
                }
            }
        }
        last = curr;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
// Tarefa para controlar LEDs com base nos alertas
void led_alert_task(void *pvParameters)
{
    (void)pvParameters;
    float temp, hr;
    bool temp_alerta, bpm_alerta;

    while (true)
    {
        if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE)
        {
            temp = temp_sim;
            hr = bpm_sim;
            xSemaphoreGive(sensor_data_mutex);
        }

        temp_alerta = (temp > TEMPERATURA_SIRS);
        bpm_alerta = (hr > BPM_SIRS);

        if (temp_alerta && bpm_alerta)
        {                                                    // Ambos altos = Vermelho
            pwm_set_gpio_level(LED_RED_PIN, PWM_WRAP_VALUE); // Vermelho máximo
            pwm_set_gpio_level(LED_GREEN_PIN, 0);            // Verde desligado
            cyw43_arch_gpio_put(ONBOARD_LED_PIN, 1);         // LED onboard piscando ou aceso para alerta
        }
        else if (temp_alerta || bpm_alerta)
        {                                                          // Um dos dois alto = Laranja
            pwm_set_gpio_level(LED_RED_PIN, PWM_WRAP_VALUE);       // Vermelho máximo
            pwm_set_gpio_level(LED_GREEN_PIN, PWM_WRAP_VALUE / 2); // Verde médio (para fazer laranja)
            cyw43_arch_gpio_put(ONBOARD_LED_PIN, 1);
        }
        else
        {                                            // Normal
            pwm_set_gpio_level(LED_RED_PIN, 0);      // Vermelho desligado
            pwm_set_gpio_level(LED_GREEN_PIN, 0);    // Verde desligado (ou pode ser um verde fixo se quiser)
            cyw43_arch_gpio_put(ONBOARD_LED_PIN, 0); // LED onboard apagado ou em estado normal
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Atualiza o estado dos LEDs
    }
}

// Tarefa para pollar a stack Wi-Fi
void network_poll_task(void *pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(10)); // Poll da rede com frequência
    }
}

// --- LwIP Callbacks e Configuração do Servidor ---
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    if (err != ERR_OK || newpcb == NULL)
    {
        printf("Falha ao aceitar conexão TCP: %d\n", err);
        return ERR_VAL;
    }
    printf("Nova conexão TCP aceita.\n");
    tcp_recv(newpcb, tcp_server_recv); // Define o callback para dados recebidos
    return ERR_OK;
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p)
    {
        tcp_close(tpcb);
        return ERR_OK;
    }
    if (p->len >= 3 && strncmp((char *)p->payload, "GET", 3) == 0)
    {
        char html[768];  // Aumentado para comportar o CSS
        float t, h;
        int est;
        xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(100));
        t = temp_sim;
        h = bpm_sim;
        est = estado_temp;
        xSemaphoreGive(sensor_data_mutex);

        const char *status = "Normal";
        if (t > TEMPERATURA_SIRS && h > BPM_SIRS)
            status = "Critico";
        else if (t > TEMPERATURA_SIRS)
            status = "Temp Alta";
        else if (h > BPM_SIRS)
            status = "HR Alta";

        int len = snprintf(html, sizeof(html),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Connection: close\r\n\r\n"
            "<html><head><title>Status</title>"
            "<style>"
            "body { background-color: #121212; color: #FFFFFF; font-family: Arial, sans-serif; text-align: center; padding: 40px; }"
            "h2 { margin: 15px 0; }"
            "</style>"
            "</head><body>"
            "<h2>Temp: %.1f &#8451;</h2>"
            "<h2>HR: %.0f bpm</h2>"
            "<h2>Status: %s</h2>"
            "<script>"
            "setInterval(function() { location.href='/'; }, 5000);"
            "</script>"
            "</body></html>",
            t, h, status);

        tcp_write(tpcb, html, len, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
    }
    pbuf_free(p);
    return ERR_OK;
}


void launch_web_server(void)
{
    struct tcp_pcb *server_pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!server_pcb)
    {
        printf("Falha ao criar PCB do servidor TCP\n");
        return;
    }

    err_t err = tcp_bind(server_pcb, IP_ADDR_ANY, 80);
    if (err != ERR_OK)
    {
        printf("Falha ao associar servidor TCP à porta 80: %d\n", err);
        tcp_close(server_pcb); // Libera o PCB se o bind falhar
        return;
    }

    server_pcb = tcp_listen_with_backlog(server_pcb, 1); // backlog de 1 é suficiente para este exemplo
    if (!server_pcb)
    {
        printf("Falha ao colocar servidor TCP em modo de escuta\n");
        // O PCB original pode ter sido liberado por tcp_listen em caso de erro.
        // Se tcp_listen falhar e retornar NULL, o pcb antigo já foi desalocado.
    }
    else
    {
        tcp_accept(server_pcb, tcp_server_accept);
        printf("Servidor TCP ouvindo na porta 80\n");
    }
}

// --- Função Principal ---
int main()
{
    stdio_init_all();

    // Inicializa GPIOs e PWM
    init_gpios_pwm();

    // Inicializa a arquitetura cyw43 (Wi-Fi)
    if (cyw43_arch_init())
    {
        printf("Falha ao inicializar Wi-Fi\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    cyw43_arch_gpio_put(ONBOARD_LED_PIN, 0); // LED onboard desligado inicialmente

    // Conecta ao Wi-Fi
    printf("Conectando ao Wi-Fi: %s...\n", WIFI_SSID);
    int retries = 0;
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 120000))
    {
        printf("Falha ao conectar ao Wi-Fi. Tentativa %d\n", ++retries);
        if (retries > 5)
        {
            printf("Muitas falhas ao conectar. Desistindo.\n");
            cyw43_arch_deinit();
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Espera antes de tentar novamente
    }
    printf("Conectado ao Wi-Fi!\n");
    printf("IP do dispositivo: %s\n", ipaddr_ntoa(netif_ip_addr4(netif_default)));
    cyw43_arch_gpio_put(ONBOARD_LED_PIN, 1); // Acende LED onboard para indicar conexão Wi-Fi

    // Cria o Mutex para os dados dos sensores
    sensor_data_mutex = xSemaphoreCreateMutex();
    if (sensor_data_mutex == NULL)
    {
        printf("Falha ao criar mutex!\n");
        return -1;
    }

    // Lança o servidor web (configura LwIP, não bloqueia)
    launch_web_server();

    // Cria as tasks do FreeRTOS
    xTaskCreate(heart_sensor_task, "HeartSensorTask", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, &heart_sensor_task_handle);

    xTaskCreate(button_task, "ButtonTask", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, &button_task_handle);

    xTaskCreate(led_alert_task, "LedAlertTask", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, &led_alert_task_handle);

    xTaskCreate(network_poll_task, "NetworkPollTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &network_poll_task_handle); // Prioridade mais alta para rede

    printf("Iniciando scheduler do FreeRTOS...\n");
    vTaskStartScheduler();

    panic_unsupported();

    return 0;
}