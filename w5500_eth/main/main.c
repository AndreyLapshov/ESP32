#define SYSLOG_HOSTNAME   "esp32-w5500"
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "ethernet_init.h"
#include "lwip/sockets.h"
#include "lwip/apps/sntp.h"

#define SYSLOG_SERVER_IP    "192.168.1.100"
#define SYSLOG_SERVER_PORT  514
#define SYSLOG_FACILITY     1
#define SYSLOG_SEVERITY     6

static const char *TAG = "syslog_eth";

// Обработчик события получения IP по Ethernet
static void ip_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    ip_event_got_ip_t* evt = (ip_event_got_ip_t*)data;
    ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&evt->ip_info.ip));
}

// Синхронизация времени по SNTP
static void obtain_time(void)
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
    time_t now = 0;
    struct tm ti = { 0 };
    int retry = 0;
    while (ti.tm_year < (2020 - 1900) && retry++ < 15) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now); localtime_r(&now, &ti);
    }
    setenv("TZ", "UTC0", 1); tzset();
    ESP_LOGI(TAG, "Current time: %s", asctime(&ti));
}

// Задача отправки syslog по UDP
static void syslog_task(void *pvParameters)
{
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(SYSLOG_SERVER_PORT),
    };
    inet_pton(AF_INET, SYSLOG_SERVER_IP, &dest.sin_addr.s_addr);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { ESP_LOGE(TAG, "socket failed: errno %d", errno); vTaskDelete(NULL); }

    // Убираем gethostname(), просто используем статичное имя:
    const char *hostname = SYSLOG_HOSTNAME;

    int counter = 0;
    while (1) {
        counter++;
        time_t now = time(NULL);
        struct tm ti;
        localtime_r(&now, &ti);
        char timestamp[32];
        strftime(timestamp, sizeof(timestamp), "%b %d %H:%M:%S", &ti);

        int pri = SYSLOG_FACILITY*8 + SYSLOG_SEVERITY;
        char msg[256];
        snprintf(msg, sizeof(msg),
                 "<%d>%s %s syslog_eth[%d]: Hello via W5500 #%d",
                 pri, timestamp, hostname, getpid(), counter);

        int err = sendto(sock, msg, strlen(msg), 0,
                         (struct sockaddr*)&dest, sizeof(dest));
        if (err < 0) {
            ESP_LOGE(TAG, "sendto failed: errno %d", errno);
        } else {
            ESP_LOGI(TAG, "Sent: %s", msg);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    // …
}

void app_main(void)
{
    // Инициализация NVS и стека
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 1) Ethernet-инициализация
    #define MAX_ETH_PORTS  (CONFIG_ETHERNET_INTERNAL_SUPPORT + CONFIG_ETHERNET_SPI_NUMBER)
    esp_eth_handle_t *eth_handles[MAX_ETH_PORTS];
    uint8_t eth_port_cnt = 0;
    ESP_ERROR_CHECK( ethernet_init_all(eth_handles, &eth_port_cnt) );  // :contentReference[oaicite:1]{index=1}
    ESP_LOGI(TAG, "Ethernet initialized, %d port(s)", eth_port_cnt);

    // 2) Подписка на получение IP (DHCP)
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               ip_event_handler, NULL) );

    // 3) SNTP и запуск syslog-задачи
    obtain_time();
    xTaskCreate(syslog_task, "syslog_task", 4096, NULL, 5, NULL);
}
