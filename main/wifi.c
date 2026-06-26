#include "wifi.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "config.h"

static const char *TAG = "WIFI";
static int sock = -1;
static struct sockaddr_in dest;

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
        ESP_LOGI(TAG, "WiFi connected, ready to send on port %d", TELEMETRY_UDP_PORT);
}

void wifi_init(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);

    wifi_config_t wcfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    esp_wifi_start();

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    dest.sin_family = AF_INET;
    dest.sin_port = htons(TELEMETRY_UDP_PORT);
    dest.sin_addr.s_addr = inet_addr(WIFI_UDP_TARGET_IP);
    ESP_LOGI(TAG, "WiFi STA init, UDP target %s:%d", WIFI_UDP_TARGET_IP, TELEMETRY_UDP_PORT);
}

void wifi_send_telemetry(float line_pos, float left_speed, float right_speed,
                         float target_speed, float gyro_z, float left_thr, float right_thr) {
    if (sock < 0) return;
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "LP:%.2f LS:%.2f RS:%.2f TS:%.2f GZ:%.2f LT:%.3f RT:%.3f\n",
        line_pos, left_speed, right_speed, target_speed, gyro_z, left_thr, right_thr);
    sendto(sock, buf, len, 0, (struct sockaddr*)&dest, sizeof(dest));
}
