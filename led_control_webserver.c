#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Includes do Pico
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

// Includes do LwIP e MQTT
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h" // Para MQTT_OUTPUT_RINGBUF_SIZE
#include "lwip/dns.h"
#include "lwip/altcp_tls.h" // Se for usar TLS

// Includes do FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// --- Configurações de Rede e MQTT
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define MQTT_SERVER ""

#ifndef MQTT_PORT      // Guard against redefinition if LwIP headers define it
#define MQTT_PORT 1883 // Porta padrão MQTT, mudar para 8883 se usar TLS
#endif

// Opcional: credenciais MQTT se o broker exigir
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""

#define MQTT_DEVICE_NAME "PicoWSIRSMonitor" // Nome base do dispositivo
#define MQTT_UNIQUE_TOPIC 0                 // 1 para adicionar client_id aos tópicos, 0 caso contrário

// QoS e Retain
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0
#define MQTT_SUBSCRIBE_QOS 1

#define MQTT_TOPIC_SIRS_SET_ESTADO_REMOTO "/sirs/set_estado_remoto"

// Last Will and Testament
#define MQTT_WILL_TOPIC "/status" // Tópico LWT será MQTT_DEVICE_NAME<ID>/status
#define MQTT_WILL_MSG_ONLINE "online"
#define MQTT_WILL_MSG_OFFLINE "offline"
#define MQTT_WILL_QOS 1
#define MQTT_WILL_RETAIN 1 // Reter a mensagem de LWT

#define MQTT_KEEP_ALIVE_S 60
#define MQTT_TOPIC_LEN 128 // Aumentado para acomodar client_id + tópico base

// --- Definições de Pinos
// LED Onboard da Pico W (controlado pelo cyw43)
#define ONBOARD_LED_PIN CYW43_WL_GPIO_LED_PIN

// LEDs para Alerta SIRS (PWM)
#define LED_RED_PIN 13   // GPIO13 - LED vermelho
#define LED_GREEN_PIN 11 // GPIO11 - LED verde

// Botão para simulação de estados SIRS - DO SEU CÓDIGO ORIGINAL
#define BUTTON_A_PIN 5 // GPIO5 - Botão A

// --- Constantes para Simulação SIRS ---
#define TEMPERATURA_NORMAL 25.0f
#define TEMPERATURA_ALTA 38.5f
#define TEMPERATURA_SIRS 38.0f // Limite para alerta de temperatura

#define BPM_NORMAL 70.0f
#define BPM_ALTO 120.0f
#define BPM_SIRS 90.0f // Limite para alerta de BPM

#define PWM_WRAP_VALUE 255 // Para controle PWM de 8 bits

// --- Variáveis Globais ---
// Dados simulados SIRS
volatile float temp_sim = TEMPERATURA_NORMAL;
volatile float bpm_sim = BPM_NORMAL;
volatile int estado_sirs = 0; // 0: Normal, 1: HR Alto, 2: Temp Alta, 3: Ambos Altos

// Mutex para proteger dados compartilhados
SemaphoreHandle_t shared_data_mutex;

// Estrutura de dados do cliente MQTT
typedef struct MQTT_CLIENT_DATA_T_
{
    mqtt_client_t *mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char client_id[64];                        // Buffer para client_id
    char current_topic_buffer[MQTT_TOPIC_LEN]; // Buffer para o tópico da mensagem atual
    ip_addr_t mqtt_server_address;
    bool connect_done;          // Flag para indicar que a conexão inicial (ou tentativa) foi feita
    bool mqtt_connected;        // Flag para indicar se está ativamente conectado
    int subscribe_count;        // Contador de assinaturas ativas
    bool stop_client_requested; // Para solicitar parada do cliente
} MQTT_CLIENT_DATA_T;

static MQTT_CLIENT_DATA_T mqtt_state; // Instância global dos dados do cliente MQTT

// Funções de Inicialização
void init_peripherals(void);

// Funções MQTT (baseadas no código do professor)
static void mqtt_start_client(MQTT_CLIENT_DATA_T *state);
static void mqtt_dns_found_cb(const char *hostname, const ip_addr_t *ipaddr, void *arg);
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
static void mqtt_pub_request_cb(void *arg, err_t err);
static void mqtt_sub_unsub_request_cb(void *arg, err_t err);
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
static void mqtt_subscribe_to_topics(MQTT_CLIENT_DATA_T *state);
static void mqtt_publish_data(MQTT_CLIENT_DATA_T *state, const char *sub_topic, const char *payload);
static const char *mqtt_get_full_topic(MQTT_CLIENT_DATA_T *state, const char *sub_topic, char *buffer, size_t buffer_len);
void set_sirs_state_and_publish(int novo_estado, const char *origem_comando);

// Tarefas FreeRTOS
void sirs_button_task(void *pvParameters);
void led_alert_task(void *pvParameters);      // Mantém controle local dos LEDs de alerta SIRS
void mqtt_main_loop_task(void *pvParameters); // Tarefa para o loop principal MQTT e cyw43_poll

// --- Implementação das Funções de Inicialização ---
void init_peripherals(void)
{
    // Botão A
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);

    // LEDs para PWM (Alerta SIRS)
    uint led_pins[] = {LED_RED_PIN, LED_GREEN_PIN};
    for (int i = 0; i < sizeof(led_pins) / sizeof(led_pins[0]); ++i)
    {
        uint gpio = led_pins[i];
        gpio_set_function(gpio, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(gpio);

        pwm_config config = pwm_get_default_config();
        pwm_config_set_wrap(&config, PWM_WRAP_VALUE);
        pwm_init(slice_num, &config, true);
        pwm_set_gpio_level(gpio, 0);
    }
}

// --- Função para definir o estado SIRS e atualizar as variáveis simuladas ---
void set_sirs_state_and_publish(int novo_estado, const char *origem_comando)
{
    char temp_payload[10];
    char bpm_payload[10];
    char estado_payload_str[30]; // Buffer para string do estado com origem

    if (xSemaphoreTake(shared_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        estado_sirs = novo_estado % 4; // Garante que o estado esteja entre 0 e 3

        switch (estado_sirs)
        {
        case 0:
            temp_sim = TEMPERATURA_NORMAL;
            bpm_sim = BPM_NORMAL;
            snprintf(estado_payload_str, sizeof(estado_payload_str), "Normal (%s)", origem_comando);
            break;
        case 1:
            temp_sim = TEMPERATURA_NORMAL;
            bpm_sim = BPM_ALTO;
            snprintf(estado_payload_str, sizeof(estado_payload_str), "BPM Alto (%s)", origem_comando);
            break;
        case 2:
            temp_sim = TEMPERATURA_ALTA;
            bpm_sim = BPM_NORMAL;
            snprintf(estado_payload_str, sizeof(estado_payload_str), "Temp Alta (%s)", origem_comando);
            break;
        case 3:
            temp_sim = TEMPERATURA_ALTA;
            bpm_sim = BPM_ALTO;
            snprintf(estado_payload_str, sizeof(estado_payload_str), "Critico (%s)", origem_comando);
            break;
        default:             // Caso algo dê errado com novo_estado
            estado_sirs = 0; // Volta para o estado normal
            temp_sim = TEMPERATURA_NORMAL;
            bpm_sim = BPM_NORMAL;
            snprintf(estado_payload_str, sizeof(estado_payload_str), "Normal (Default %s)", origem_comando);
            break;
        }
        printf("Comando SIRS (%s): %s (Temp: %.1f, BPM: %.0f)\n", origem_comando, estado_payload_str, temp_sim, bpm_sim);

        // Prepara payloads para MQTT
        snprintf(temp_payload, sizeof(temp_payload), "%.1f", temp_sim);
        snprintf(bpm_payload, sizeof(bpm_payload), "%.0f", bpm_sim);

        xSemaphoreGive(shared_data_mutex);

        // Publica os novos dados via MQTT para confirmar a mudança
        if (mqtt_state.mqtt_connected)
        {
            mqtt_publish_data(&mqtt_state, "/sirs/temperatura", temp_payload);
            mqtt_publish_data(&mqtt_state, "/sirs/bpm", bpm_payload);
            mqtt_publish_data(&mqtt_state, "/sirs/estado", estado_payload_str);
        }
    }
    else
    {
        printf("Falha ao obter mutex para set_sirs_state_and_publish\n");
    }
}

// --- Implementação das Tarefas FreeRTOS ---

void sirs_button_task(void *pvParameters)
{
    (void)pvParameters;
    bool last_button_state = true;
    TickType_t last_press_time = 0;
    // As variáveis temp_payload, bpm_payload, estado_payload não são mais necessárias aqui

    while (true)
    {
        bool current_button_state = gpio_get(BUTTON_A_PIN);
        if (current_button_state == false && last_button_state == true)
        {
            if (xTaskGetTickCount() - last_press_time > pdMS_TO_TICKS(250))
            {
                last_press_time = xTaskGetTickCount();

                int proximo_estado;
                // Pega o estado atual antes de modificá-lo para calcular o próximo estado
                if (xSemaphoreTake(shared_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    proximo_estado = (estado_sirs + 1) % 4;
                    xSemaphoreGive(shared_data_mutex);
                    // Chama a função unificada para mudar o estado e publicar
                    set_sirs_state_and_publish(proximo_estado, "Botao");
                }
                else
                {
                    printf("Falha ao obter mutex em sirs_button_task para ler estado\n");
                }
            }
        }
        last_button_state = current_button_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void led_alert_task(void *pvParameters)
{
    (void)pvParameters;
    float current_temp, current_hr;
    bool temp_alerta, bpm_alerta;

    while (true)
    {
        if (xSemaphoreTake(shared_data_mutex, portMAX_DELAY) == pdTRUE)
        {
            current_temp = temp_sim;
            current_hr = bpm_sim;
            xSemaphoreGive(shared_data_mutex);
        }

        temp_alerta = (current_temp >= TEMPERATURA_SIRS);
        bpm_alerta = (current_hr >= BPM_SIRS);

        if (temp_alerta && bpm_alerta)
        {
            pwm_set_gpio_level(LED_RED_PIN, PWM_WRAP_VALUE);
            pwm_set_gpio_level(LED_GREEN_PIN, 0);
        }
        else if (temp_alerta || bpm_alerta)
        {
            pwm_set_gpio_level(LED_RED_PIN, PWM_WRAP_VALUE);
            pwm_set_gpio_level(LED_GREEN_PIN, PWM_WRAP_VALUE / 3);
        }
        else
        {
            pwm_set_gpio_level(LED_RED_PIN, 0);
            pwm_set_gpio_level(LED_GREEN_PIN, PWM_WRAP_VALUE);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void mqtt_main_loop_task(void *pvParameters)
{
    (void)pvParameters;
    // printf("MQTT Main Loop Task iniciada.\n");
    while (true)
    {
        cyw43_arch_poll();
        if (!mqtt_state.mqtt_connected && mqtt_state.connect_done && !mqtt_state.stop_client_requested)
        {
            // printf("MQTT desconectado. Tentando reconectar...\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// --- Implementação das Funções MQTT

static const char *mqtt_get_full_topic(MQTT_CLIENT_DATA_T *state, const char *sub_topic, char *buffer, size_t buffer_len)
{
#if MQTT_UNIQUE_TOPIC
    if (sub_topic[0] == '/')
    {
        snprintf(buffer, buffer_len, "%s%s", state->client_id, sub_topic);
    }
    else
    {
        snprintf(buffer, buffer_len, "%s/%s", state->client_id, sub_topic);
    }
#else
    strncpy(buffer, sub_topic, buffer_len);
#endif
    buffer[buffer_len - 1] = '\0';
    return buffer;
}

static void mqtt_publish_data(MQTT_CLIENT_DATA_T *state, const char *sub_topic, const char *payload)
{
    if (!state->mqtt_connected || !state->mqtt_client_inst)
    {
        return;
    }
    char full_topic_buf[MQTT_TOPIC_LEN];
    mqtt_get_full_topic(state, sub_topic, full_topic_buf, sizeof(full_topic_buf));

    cyw43_arch_lwip_begin();
    err_t err = mqtt_publish(state->mqtt_client_inst, full_topic_buf, payload, strlen(payload),
                             MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, mqtt_pub_request_cb, state);
    cyw43_arch_lwip_end();

    if (err != ERR_OK)
    {
        printf("Falha ao publicar MQTT para %s: %d\n", full_topic_buf, err);
    }
}

static void mqtt_pub_request_cb(void *arg, err_t err)
{
    if (err != ERR_OK)
    {
        printf("MQTT pub request cb: error %d\n", err);
    }
}

static void mqtt_sub_unsub_request_cb(void *arg, err_t err)
{
    MQTT_CLIENT_DATA_T *state_ptr = (MQTT_CLIENT_DATA_T *)arg; // Renamed to avoid conflict with global
    if (err != ERR_OK)
    {
        printf("MQTT sub/unsub request cb: error %d\n", err);
    }
    if (state_ptr->stop_client_requested && state_ptr->subscribe_count == 0)
    {
        printf("Todas as subscrições canceladas e parada solicitada. Desconectando MQTT.\n");
        mqtt_disconnect(state_ptr->mqtt_client_inst);
        state_ptr->mqtt_connected = false;
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    MQTT_CLIENT_DATA_T *state_ptr = (MQTT_CLIENT_DATA_T *)arg; // Renamed
    // printf("MQTT Início da Publicação: Topico='%s', Comprimento Total=%u\n", topic, (unsigned int)tot_len);
    if (topic != NULL)
    { // Store the topic for mqtt_incoming_data_cb
        strncpy(state_ptr->current_topic_buffer, topic, sizeof(state_ptr->current_topic_buffer));
        state_ptr->current_topic_buffer[sizeof(state_ptr->current_topic_buffer) - 1] = '\0';
    }
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    MQTT_CLIENT_DATA_T *state_ptr = (MQTT_CLIENT_DATA_T *)arg;
    char payload_str[len + 1];
    memcpy(payload_str, data, len);
    payload_str[len] = '\0';

    char *current_topic_full = state_ptr->current_topic_buffer;
    char expected_topic_full[MQTT_TOPIC_LEN];

    printf("MQTT Dados Recebidos: Topico='%s', Payload='%s'\n", current_topic_full, payload_str);

    if (strcmp(current_topic_full, mqtt_get_full_topic(state_ptr, "/led", expected_topic_full, sizeof(expected_topic_full))) == 0)
    {
        if (strcasecmp(payload_str, "ON") == 0 || strcmp(payload_str, "1") == 0)
        {
            cyw43_arch_gpio_put(ONBOARD_LED_PIN, 1);
            printf("MQTT: LED Onboard -> ON\n");
        }
        else if (strcasecmp(payload_str, "OFF") == 0 || strcmp(payload_str, "0") == 0)
        {
            cyw43_arch_gpio_put(ONBOARD_LED_PIN, 0);
            printf("MQTT: LED Onboard -> OFF\n");
        }
    }
    // NOVA LÓGICA PARA TRATAR O COMANDO REMOTO DE ESTADO SIRS
    else if (strcmp(current_topic_full, mqtt_get_full_topic(state_ptr, MQTT_TOPIC_SIRS_SET_ESTADO_REMOTO, expected_topic_full, sizeof(expected_topic_full))) == 0)
    {
        int novo_estado_remoto = atoi(payload_str); // Converte o payload (ex: "0", "1") para inteiro
        if (novo_estado_remoto >= 0 && novo_estado_remoto <= 3)
        {
            set_sirs_state_and_publish(novo_estado_remoto, "Remoto");
        }
        else
        {
            printf("Comando de estado SIRS remoto inválido: %s\n", payload_str);
        }
    }
    // FIM DA NOVA LÓGICA
    else if (strcmp(current_topic_full, mqtt_get_full_topic(state_ptr, "/print", expected_topic_full, sizeof(expected_topic_full))) == 0)
    {
        printf("MQTT Print: %s\n", payload_str);
    }
    else if (strcmp(current_topic_full, mqtt_get_full_topic(state_ptr, "/ping", expected_topic_full, sizeof(expected_topic_full))) == 0)
    {
        char uptime_buf[12];
        snprintf(uptime_buf, sizeof(uptime_buf), "%lu", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish_data(state_ptr, "/uptime", uptime_buf);
        printf("MQTT: Ping recebido, respondendo com uptime: %s\n", uptime_buf);
    }
    else if (strcmp(current_topic_full, mqtt_get_full_topic(state_ptr, "/exit", expected_topic_full, sizeof(expected_topic_full))) == 0)
    {
        printf("MQTT: Comando /exit recebido. Solicitando parada do cliente.\n");
        state_ptr->stop_client_requested = true;
    }
}

static void mqtt_subscribe_to_topics(MQTT_CLIENT_DATA_T *state_param)
{ // Renomeei state para state_param para clareza
    char topic_buf[MQTT_TOPIC_LEN];
    err_t err;

    const char *topics_to_subscribe[] = {
        "/led",
        "/print",
        "/ping",
        "/exit",
        MQTT_TOPIC_SIRS_SET_ESTADO_REMOTO // <--- ADICIONE ESTA LINHA
    };
    int num_topics = sizeof(topics_to_subscribe) / sizeof(topics_to_subscribe[0]);

    cyw43_arch_lwip_begin();
    for (int i = 0; i < num_topics; ++i)
    {
        mqtt_get_full_topic(state_param, topics_to_subscribe[i], topic_buf, sizeof(topic_buf));
        state_param->subscribe_count++;
        err = mqtt_subscribe(state_param->mqtt_client_inst, topic_buf, MQTT_SUBSCRIBE_QOS, mqtt_sub_unsub_request_cb, state_param);
        if (err != ERR_OK)
        {
            printf("Falha ao inscrever-se em %s, erro %d\n", topic_buf, err);
            state_param->subscribe_count--;
        }
    }
    cyw43_arch_lwip_end();
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    MQTT_CLIENT_DATA_T *state_ptr = (MQTT_CLIENT_DATA_T *)arg; // Renamed
    state_ptr->connect_done = true;

    if (status == MQTT_CONNECT_ACCEPTED)
    {
        printf("MQTT Conectado com sucesso!\n");
        state_ptr->mqtt_connected = true;
        cyw43_arch_gpio_put(ONBOARD_LED_PIN, 1);

        mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state_ptr);
        mqtt_subscribe_to_topics(state_ptr);

        char lwt_topic_full[MQTT_TOPIC_LEN];
        mqtt_get_full_topic(state_ptr, MQTT_WILL_TOPIC, lwt_topic_full, sizeof(lwt_topic_full));

        cyw43_arch_lwip_begin();
        mqtt_publish(client, lwt_topic_full, MQTT_WILL_MSG_ONLINE, strlen(MQTT_WILL_MSG_ONLINE),
                     MQTT_WILL_QOS, MQTT_WILL_RETAIN, mqtt_pub_request_cb, state_ptr);
        cyw43_arch_lwip_end();
    }
    else
    {
        printf("MQTT Falha na Conexao: %d\n", status);
        state_ptr->mqtt_connected = false;
        cyw43_arch_gpio_put(ONBOARD_LED_PIN, 0);
    }
}

static void mqtt_start_client(MQTT_CLIENT_DATA_T *state)
{
    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst)
    {
        printf("Falha ao criar instancia do cliente MQTT.\n");
        return;
    }

    state->mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;
    // client_id is already set in state->mqtt_client_info via main()

#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state->mqtt_client_info.client_user = MQTT_USERNAME;
    state->mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state->mqtt_client_info.client_user = NULL;
    state->mqtt_client_info.client_pass = NULL;
#endif

    static char will_topic_full_static[MQTT_TOPIC_LEN];
    mqtt_get_full_topic(state, MQTT_WILL_TOPIC, will_topic_full_static, sizeof(will_topic_full_static));
    state->mqtt_client_info.will_topic = will_topic_full_static;
    state->mqtt_client_info.will_msg = MQTT_WILL_MSG_OFFLINE;
    state->mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state->mqtt_client_info.will_retain = MQTT_WILL_RETAIN;

#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // If using TLS, tls_config needs to be properly initialized here.
    // For non-TLS, it should be NULL or the field might not exist if macros aren't defined.
    // state->mqtt_client_info.tls_config = altcp_tls_create_config_client(NULL, 0); // Example for basic TLS
    state->mqtt_client_info.tls_config = NULL; // Explicitly NULL for non-TLS if field exists
#else
    // If LWIP_ALTCP or LWIP_ALTCP_TLS is not defined, the tls_config member might not exist.
    // No action needed here for tls_config if it's not part of the struct.
#endif

    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(state->mqtt_client_inst,
                                    &state->mqtt_server_address,
                                    MQTT_PORT,
                                    mqtt_connection_cb,
                                    state,
                                    &state->mqtt_client_info);
    cyw43_arch_lwip_end();

    if (err != ERR_OK)
    {
        printf("Erro ao conectar ao broker MQTT: %d\n", err);
    }
}

static void mqtt_dns_found_cb(const char *hostname, const ip_addr_t *ipaddr, void *arg)
{
    MQTT_CLIENT_DATA_T *state_ptr = (MQTT_CLIENT_DATA_T *)arg; // Renamed
    if (ipaddr)
    {
        state_ptr->mqtt_server_address = *ipaddr;
        mqtt_start_client(state_ptr);
    }
    else
    {
        printf("Falha ao resolver DNS para %s\n", hostname);
    }
}

// --- Função Principal ---
int main()
{
    stdio_init_all();
    printf("Iniciando Monitor SIRS com MQTT e FreeRTOS (Simplificado)...\n");

    init_peripherals();

    if (cyw43_arch_init())
    {
        printf("Falha ao inicializar Wi-Fi (cyw43_arch_init)\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();

    char unique_id_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(unique_id_str, sizeof(unique_id_str));
    for (int i = 0; unique_id_str[i]; i++)
    {
        unique_id_str[i] = tolower(unique_id_str[i]);
    }
    // Initialize mqtt_state members before use
    memset(&mqtt_state, 0, sizeof(mqtt_state)); // Zero out the structure

    snprintf(mqtt_state.client_id, sizeof(mqtt_state.client_id), "%s_%s", MQTT_DEVICE_NAME, unique_id_str);
    mqtt_state.mqtt_client_info.client_id = mqtt_state.client_id;
    printf("MQTT Client ID: %s\n", mqtt_state.client_id);

    printf("Conectando ao Wi-Fi: %s...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Falha ao conectar ao Wi-Fi.\n");
        cyw43_arch_deinit();
        return -1;
    }
    printf("Conectado ao Wi-Fi com sucesso!\n");
    printf("IP do dispositivo: %s\n", ipaddr_ntoa(netif_ip4_addr(netif_default)));

    ip_addr_t server_ip;
    if (ipaddr_aton(MQTT_SERVER, &server_ip))
    {
        mqtt_state.mqtt_server_address = server_ip;
        mqtt_start_client(&mqtt_state);
    }
    else
    {
        cyw43_arch_lwip_begin();
        err_t err = dns_gethostbyname(MQTT_SERVER, &mqtt_state.mqtt_server_address, mqtt_dns_found_cb, &mqtt_state);
        cyw43_arch_lwip_end();
        if (err == ERR_OK)
        {
            mqtt_dns_found_cb(MQTT_SERVER, &mqtt_state.mqtt_server_address, &mqtt_state);
        }
        else if (err != ERR_INPROGRESS)
        {
            printf("Erro imediato ao iniciar resolucao DNS: %d\n", err);
        }
    }

    shared_data_mutex = xSemaphoreCreateMutex();
    if (shared_data_mutex == NULL)
    {
        printf("Falha ao criar mutex!\n");
        return -1;
    }

    BaseType_t sirs_task_status = xTaskCreate(sirs_button_task, "SIRSSimTask", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
    BaseType_t led_task_status = xTaskCreate(led_alert_task, "LEDAlertTask", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    BaseType_t mqtt_loop_task_status = xTaskCreate(mqtt_main_loop_task, "MQTTLoopTask", configMINIMAL_STACK_SIZE * 3, (void *)&mqtt_state, tskIDLE_PRIORITY + 3, NULL);

    if (sirs_task_status != pdPASS || led_task_status != pdPASS || mqtt_loop_task_status != pdPASS)
    {
        printf("Falha ao criar uma ou mais tasks!\n");
        return -1;
    }

    printf("Iniciando scheduler do FreeRTOS...\n");
    vTaskStartScheduler();

    while (true)
    {
    };
    return 0;
}
