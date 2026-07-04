#include "wifi.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "config.h"

static const char *TAG = "WIFI";
static int sock = -1;
static struct sockaddr_in dest;
bool wifi_ready = false;

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_ready = false;
        esp_wifi_connect();
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi connected!");
        wifi_ready = true; // 只置标志位，不在回调里建 Socket
    }
}

void wifi_init(void) {
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

    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_max_tx_power(52);
}

void wifi_send_telemetry(float line_pos, float left_speed, float right_speed,
                         float target_speed, float gyro_z,
                         float ff_omega, float lt, float rt, bool running) {
    if (!wifi_ready) return;

    // 懒加载 Socket — 不在回调里建
    if (sock < 0) {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) return;
        int broadcast = 1;
        setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
        dest.sin_family = AF_INET;
        dest.sin_port = htons(TELEMETRY_UDP_PORT);
        dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        ESP_LOGI(TAG, "Telemetry UDP ready");
    }

    char buf[160];
    int len = snprintf(buf, sizeof(buf),
        "LP:%.2f LS:%.2f RS:%.2f TS:%.2f GZ:%.2f FF:%.2f LT:%.3f RT:%.3f RUN:%d\n",
        line_pos, left_speed, right_speed, target_speed, gyro_z,
        ff_omega, lt, rt, running ? 1 : 0);
    sendto(sock, buf, len, 0, (struct sockaddr*)&dest, sizeof(dest));
}
