#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/pulse_cnt.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "lwip/sockets.h"
#include "oled.h"

static const char *TAG = "MAIN";

/* BARE_OLED_TEST: 只跑 OLED + UART，验证 DIO flash + 无 PSRAM 稳定性 */
#define BARE_OLED_TEST 1

// ================== 配置定义 ==================
#define CONTROL_LOOP_HZ       1000
#define CONTROL_DT            0.001f
#define MAX_SPEED_MPS         3.0f
#define WHEEL_DIAMETER_MM     22.0f
#define ENCODER_GEAR_RATIO    10.0f
#define PULSES_PER_REV        40
#define TRACK_WIDTH_MM        160.0f
#define MAP_SAMPLE_RATE_HZ    100
#define RINGBUF_SIZE          512
#define TELEMETRY_UDP_PORT    3333
#define COMMAND_UDP_PORT      3334
#define WIFI_SSID             "mcy4"
#define WIFI_PASS             "mcy12306"
#define SPEED_PID_KP          1.5f
#define SPEED_PID_KI          0.1f
#define SPEED_PID_KD          0.05f
#define LINE_PID_KP           2.0f
#define LINE_PID_KI           0.2f
#define LINE_PID_KD           0.1f
#define I2C_MASTER_SDA_IO     16
#define I2C_MASTER_SCL_IO     15
#define I2C_MASTER_NUM        I2C_NUM_0
#define IMU_ADDR              0x68
#define OLED_ADDR             0x3C
#define LEDC_CH0_GPIO         33
#define LEDC_CH1_GPIO         34
#define LEDC_FREQ_HZ          20000
#define LEDC_RESOLUTION       LEDC_TIMER_10_BIT
#define MOTOR_L_IN1           35
#define MOTOR_L_IN2           36
#define MOTOR_R_IN1           37
#define MOTOR_R_IN2           38
#define MOTOR_STBY            39
#define START_BUTTON_GPIO     9
#define UART_TX_GPIO          43
#define UART_RX_GPIO          44
#define I2C_MASTER_FREQ_HZ    400000
#define IMU_ADDR              0x68
#define OLED_ADDR             0x3C
#define LEDC_CH0_GPIO         33
#define LEDC_CH1_GPIO         34
#define LEDC_FREQ_HZ          20000
#define LEDC_RESOLUTION       LEDC_TIMER_10_BIT
#define MOTOR_L_IN1           35
#define MOTOR_L_IN2           36
#define MOTOR_R_IN1           37
#define MOTOR_R_IN2           38
#define MOTOR_STBY            39
#define START_BUTTON_GPIO     9
#define UART_TX_GPIO          43
#define UART_RX_GPIO          44
#define PROFILE_STEP_M        0.01f
#define UART_RXBUF_SIZE       20480
#define WATCHDOG_TIMEOUT_MS   500
#define BUTTON_DEBOUNCE_MS    50

// ================== 状态机 ==================
typedef enum {
    STATE_IDLE = 0,
    STATE_ARMED,
    STATE_RUNNING
} system_state_t;

static volatile system_state_t sys_state = STATE_IDLE;
static volatile uint32_t    g_watchdog_last_ms = 0;
static volatile bool        g_button_pressed = false;

// ================== 数据结构 ==================
typedef struct {
    float kp, ki, kd;
    float integral, prev_error;
    float output_limit;
} PidHandle;

typedef struct {
    bool     active;
    float    feedforward_omega;   // deg/s, scaled for target speed
    float    target_speed;        // m/s
    float    curvature;           // legacy
    uint32_t timestamp_ms;        // for freshness check
} control_cmd_t;

typedef struct {
    float distance_m;
    float line_pos;
    float curvature;
    float speed_mps;
} map_sample_t;

// ================== 全局变量 ==================
static PidHandle           speed_pid, line_pid;
static int16_t             enc_last[2] = {0, 0};
static pcnt_unit_handle_t  pcnt_unit[2] = {NULL, NULL};
static adc_oneshot_unit_handle_t adc_handle = NULL;
static float  gyro_z_off    = 0.0f;
static float  last_line_pos = 0.0f;
EXT_RAM_BSS_ATTR static map_sample_t  map_data[2048];
static int           map_count     = 0;
static float         total_distance = 0.0f;
static int           lap_count     = 0;
EXT_RAM_BSS_ATTR static float         profile[2048];
static int           profile_len   = 0;
EXT_RAM_BSS_ATTR static control_cmd_t ringbuf[RINGBUF_SIZE];
static volatile int  ring_hd = 0, ring_tl = 0;
static volatile float cur_lp = 0, cur_ls = 0, cur_rs = 0, cur_ts = 0, cur_gz = 0, cur_lt = 0, cur_rt = 0;
static char          uart_rxbuf[UART_RXBUF_SIZE];
static bool    motor_ramp_active = false;
static int32_t motor_ramp_step   = 0;
static int32_t motor_ramp_thr[2] = {0, 0};
static float   g_vbat_comp       = 1.0f;

// ================== 环形缓冲区 ==================
void ringbuf_init(void) { ring_hd = ring_tl = 0; }

void ringbuf_push(control_cmd_t cmd) {
    int n = (ring_hd + 1) % RINGBUF_SIZE;
    if (n != ring_tl) { ringbuf[ring_hd] = cmd; ring_hd = n; }
}

control_cmd_t ringbuf_pop(void) {
    control_cmd_t e = {false, 0, MAX_SPEED_MPS, 0, 0};
    if (ring_hd == ring_tl) return e;
    control_cmd_t c = ringbuf[ring_tl];
    ring_tl = (ring_tl + 1) % RINGBUF_SIZE;
    return c;
}

bool ringbuf_is_fresh(control_cmd_t *cmd) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (now - cmd->timestamp_ms) < 15;
}

// ================== PID 控制 ==================
void pid_init(PidHandle *pid, float kp, float ki, float kd, float limit) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->integral = 0; pid->prev_error = 0; pid->output_limit = limit;
}

float pid_update_hold(PidHandle *pid, float setpoint, float measurement, bool hold) {
    float error = setpoint - measurement;
    if (!hold) {
        pid->integral += error;
        float int_limit = (pid->ki > 0) ? (pid->output_limit / pid->ki) : 0;
        if (pid->integral >  int_limit) pid->integral =  int_limit;
        if (pid->integral < -int_limit) pid->integral = -int_limit;
    }
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if (output >  pid->output_limit) return  pid->output_limit;
    if (output < -pid->output_limit) return -pid->output_limit;
    return output;
}

// ================== 硬件初始化 ==================
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num  = I2C_MASTER_SDA_IO,
        .scl_io_num  = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void ledc_init(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num  = LEDC_TIMER_0,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz    = LEDC_FREQ_HZ,
        .clk_cfg    = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);
    ledc_channel_config_t ch_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty  = 0,
        .hpoint = 0,
    };
    ch_conf.channel = LEDC_CHANNEL_0; ch_conf.gpio_num = LEDC_CH0_GPIO;
    ledc_channel_config(&ch_conf);
    ch_conf.channel = LEDC_CHANNEL_1; ch_conf.gpio_num = LEDC_CH1_GPIO;
    ledc_channel_config(&ch_conf);
}

void pcnt_init_unit(int ch, int pinA, int pinB) {
    pcnt_unit_config_t unit_cfg = { .low_limit = -32767, .high_limit = 32767 };
    pcnt_new_unit(&unit_cfg, &pcnt_unit[ch]);
    pcnt_glitch_filter_config_t filt_cfg = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(pcnt_unit[ch], &filt_cfg);
    pcnt_chan_config_t chan_cfg_a = { .edge_gpio_num = pinA, .level_gpio_num = pinB };
    pcnt_chan_config_t chan_cfg_b = { .edge_gpio_num = pinB, .level_gpio_num = pinA };
    pcnt_channel_handle_t ch_a = NULL, ch_b = NULL;
    pcnt_new_channel(pcnt_unit[ch], &chan_cfg_a, &ch_a);
    pcnt_new_channel(pcnt_unit[ch], &chan_cfg_b, &ch_b);
    pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_enable(pcnt_unit[ch]);
    pcnt_unit_clear_count(pcnt_unit[ch]);
    pcnt_unit_start(pcnt_unit[ch]);
}

// ================== 电机驱动 ==================
void motor_init(void) {
    gpio_set_direction(MOTOR_L_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_L_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_R_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_R_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_STBY, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_STBY, 1);
}

void motor_set_speed(int ch, int32_t throttle) {
    if (throttle > 100)  throttle = 100;
    if (throttle < -100) throttle = -100;
    int in1 = (ch == 0) ? MOTOR_L_IN1 : MOTOR_R_IN1;
    int in2 = (ch == 0) ? MOTOR_L_IN2 : MOTOR_R_IN2;
    if (throttle > 0) {
        gpio_set_level(in1, 1); gpio_set_level(in2, 0);
    } else if (throttle < 0) {
        gpio_set_level(in1, 0); gpio_set_level(in2, 1);
        throttle = -throttle;
    } else {
        gpio_set_level(in1, 0); gpio_set_level(in2, 0);
    }
    uint32_t duty = (throttle * 1023) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch == 0 ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch == 0 ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1);
}

void motor_coast(void) {
    gpio_set_level(MOTOR_L_IN1, 0); gpio_set_level(MOTOR_L_IN2, 0);
    gpio_set_level(MOTOR_R_IN1, 0); gpio_set_level(MOTOR_R_IN2, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    motor_ramp_active = false;
}

// ================== 电池电压补偿标定 ==================
#define VBAT_NOMINAL_MPS 2.0f
#define VBAT_COMP_MIN    0.7f
#define VBAT_COMP_MAX    1.3f

float encoder_get_speed(int ch);  // forward declaration

static void vbat_calibrate(void) {
    ESP_LOGI(TAG, "VBAT cal: start (lift wheels off ground)");
    // reset encoder counters before measurement
    for (int ch = 0; ch < 2; ch++) {
        pcnt_unit_clear_count(pcnt_unit[ch]);
        enc_last[ch] = 0;
    }
    motor_set_speed(0, 100);
    motor_set_speed(1, 100);
    vTaskDelay(pdMS_TO_TICKS(200));
    float l = encoder_get_speed(0);
    float r = encoder_get_speed(1);
    motor_coast();
    vTaskDelay(pdMS_TO_TICKS(100));
    float avg = (l + r) * 0.5f;
    if (avg > 0.3f) {
        g_vbat_comp = VBAT_NOMINAL_MPS / avg;
        if (g_vbat_comp > VBAT_COMP_MAX) g_vbat_comp = VBAT_COMP_MAX;
        if (g_vbat_comp < VBAT_COMP_MIN) g_vbat_comp = VBAT_COMP_MIN;
    } else {
        ESP_LOGW(TAG, "VBAT cal: wheels not spinning? keeping comp=1.0");
        g_vbat_comp = 1.0f;
    }
    ESP_LOGI(TAG, "VBAT cal: L=%.2f R=%.2f m/s  comp=%.3f", l, r, (double)g_vbat_comp);
}

// ================== GPIO 9 启动按钮 ISR ==================
static void IRAM_ATTR start_button_isr(void *arg) {
    g_button_pressed = true;
}

void start_button_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << START_BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(START_BUTTON_GPIO, start_button_isr, NULL);
    ESP_LOGI(TAG, "Start button on GPIO %d (neg edge, pull-up)", START_BUTTON_GPIO);
}

static void handle_button_press(void) {
    static uint32_t last_press_tick = 0;
    uint32_t now = xTaskGetTickCount();
    if (now - last_press_tick < pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) return;
    last_press_tick = now;

    if (sys_state == STATE_ARMED) {
        sys_state = STATE_RUNNING;
        pid_init(&speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 100.0f);
        pid_init(&line_pid,  LINE_PID_KP,  LINE_PID_KI,  LINE_PID_KD,  50.0f);
        ringbuf_init();
        motor_ramp_active = false;
        g_watchdog_last_ms = xTaskGetTickCount();
        ESP_LOGI(TAG, ">>> RUNNING (button)");
    }
}

// ================== 编码器 ==================
void encoder_init(void) {
    pcnt_init_unit(0, 17, 18);
    // encoder ch1 (gpio 21/22) may fail on some boards — skip gracefully
    pcnt_unit_config_t unit_cfg = { .low_limit = -32767, .high_limit = 32767 };
    esp_err_t ret = pcnt_new_unit(&unit_cfg, &pcnt_unit[1]);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "encoder ch1: pcnt_new_unit failed (%s), skipping", esp_err_to_name(ret));
        return;
    }
    pcnt_glitch_filter_config_t filt_cfg = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(pcnt_unit[1], &filt_cfg);
    ret = pcnt_unit_enable(pcnt_unit[1]);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "encoder ch1: enable failed, skipping");
        return;
    }
    pcnt_unit_clear_count(pcnt_unit[1]);
    pcnt_unit_start(pcnt_unit[1]);
    ESP_LOGI(TAG, "encoder ch1 running on fallback (no quadrature)");
}

float encoder_get_speed(int ch) {
    if (!pcnt_unit[ch]) return 0.0f;
    int count = 0;
    pcnt_unit_get_count(pcnt_unit[ch], &count);
    int d = count - enc_last[ch];
    enc_last[ch] = count;
    return d * 0.172787f;
}

// ================== 灰度传感器 ==================
void grayscale_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&init_cfg, &adc_handle);
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    for (int i = 0; i < 8; i++) {
        adc_oneshot_config_channel(adc_handle, i, &chan_cfg);
    }
}

float grayscale_get_line_position(void) {
    int raw[8];
    float ws = 0, t = 0;
    for (int i = 0; i < 8; i++) {
        adc_oneshot_read(adc_handle, i, &raw[i]);
        ws += (float)i * raw[i];
        t  += raw[i];
    }
    if (t < 10.0f) return last_line_pos;
    last_line_pos = (ws / t) - 3.5f;
    return last_line_pos;
}

// ================== IMU ==================
static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, reg, true);
    i2c_master_write_byte(c, val, true);
    i2c_master_stop(c);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    return ret;
}

static int16_t i2c_read_int16(uint8_t addr, uint8_t reg) {
    uint8_t b[2];
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, reg, true);
    i2c_master_stop(c);
    i2c_master_cmd_begin(I2C_MASTER_NUM, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(c, &b[0], I2C_MASTER_ACK);
    i2c_master_read_byte(c, &b[1], I2C_MASTER_NACK);
    i2c_master_stop(c);
    i2c_master_cmd_begin(I2C_MASTER_NUM, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    return (int16_t)((b[0] << 8) | b[1]);
}

void imu_init(void) {
    i2c_write_reg(IMU_ADDR, 0x6B, 0x00);
}

void imu_calibrate(void) {
    gyro_z_off = 0;
    float sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += i2c_read_int16(IMU_ADDR, 0x47) / 65.5f;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    gyro_z_off = sum / 100.0f;
}

float imu_get_gyro_z(void) {
    return (i2c_read_int16(IMU_ADDR, 0x47) / 65.5f - gyro_z_off) * M_PI / 180.0f;
}

// ================== Mapper ==================
void mapper_sample(float avg_spd, float line_pos, float gyro_z) {
    if (map_count < 2048 && sys_state == STATE_RUNNING) {
        float dist_inc = avg_spd * 0.01f;
        total_distance += dist_inc;
        float curv = 0.0f;
        if (avg_spd > 0.2f) curv = gyro_z / avg_spd;
        map_data[map_count].distance_m = total_distance;
        map_data[map_count].line_pos   = line_pos;
        map_data[map_count].curvature  = curv;
        map_data[map_count].speed_mps  = avg_spd;
        map_count++;
    }
}

control_cmd_t mapper_predict(float current_dist, float avg_spd) {
    control_cmd_t cmd = {false, 0, MAX_SPEED_MPS, 0, 0};
    if (profile_len > 0 && sys_state == STATE_RUNNING) {
        float look_ahead = current_dist + 0.15f;
        int p_idx = (int)(look_ahead / PROFILE_STEP_M);
        if (p_idx >= 0 && p_idx < profile_len) {
            cmd.active        = true;
            cmd.target_speed  = profile[p_idx];
            int low = 0, high = map_count - 1, idx = -1;
            while (low <= high) {
                int mid = low + (high - low) / 2;
                if (map_data[mid].distance_m < look_ahead) { idx = mid; low = mid + 1; }
                else { high = mid - 1; }
            }
            float k = (idx != -1) ? map_data[idx].curvature : 0.0f;
            cmd.curvature         = k;
            cmd.feedforward_omega = k * cmd.target_speed;
            cmd.timestamp_ms      = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
    }
    return cmd;
}

void mapper_reset(void) {
    map_count = 0; total_distance = 0.0f; lap_count = 0;
}

// ================== WiFi & UDP ==================
static volatile bool wifi_ready = false;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
        ESP_LOGI(TAG, "WiFi connected, got IP");
}

esp_err_t wifi_init(void) {
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    wifi_ready = true;
    ESP_LOGI(TAG, "WiFi init done");
    return ESP_OK;
}

void wifi_send_telemetry(float lp, float ls, float rs, float ts, float gz, float lt, float rt) {
    static int sock = -1;
    if (sock < 0) {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        int broadcast = 1;
        setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    }
    char buf[128];
    int len = sprintf(buf, "LP:%.2f LS:%.2f RS:%.2f TS:%.2f GZ:%.1f LT:%.2f RT:%.2f RUN:%d\n",
                      lp, ls, rs, ts, gz, lt, rt, (sys_state == STATE_RUNNING) ? 1 : 0);
    struct sockaddr_in dest;
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(TELEMETRY_UDP_PORT);
    dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    sendto(sock, buf, len, 0, (struct sockaddr *)&dest, sizeof(dest));
}

// ================== UART 命令处理 ==================
static uint8_t hex_to_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

// 通用命令处理器，结果写入 out，返回长度
static int process_command(char *cmd, char *out, int out_max) {
    // ── 状态查询 / 数据导出 ──
    if (strcmp(cmd, "map") == 0) {
        // map 数据量大, 通过串口逐条写出后仍返回总结 JSON
        char buf[256]; int len;
        len = sprintf(buf, "{\"lap\":%d,\"total_dist_m\":%.2f,\"n\":%d,\"samples\":[",
                      lap_count, total_distance, map_count);
        uart_write_bytes(UART_NUM_0, buf, len);
        for (int i = 0; i < map_count; i++) {
            len = sprintf(buf, "{\"d\":%.2f,\"g\":%.4f,\"p\":%.2f,\"v\":%.2f}%s",
                          map_data[i].distance_m, map_data[i].curvature,
                          map_data[i].line_pos, map_data[i].speed_mps,
                          (i == map_count - 1) ? "" : ",");
            uart_write_bytes(UART_NUM_0, buf, len);
        }
        uart_write_bytes(UART_NUM_0, "]}\n", 3);
        return snprintf(out, out_max, "{\"ok\":\"map\",\"n\":%d}\n", map_count);
    }
    if (strcmp(cmd, "lap") == 0) {
        return snprintf(out, out_max, "{\"lap\":%d,\"dist\":%.2f,\"samples\":%d,\"speed\":%.2f,\"state\":%d}\n",
                        lap_count, total_distance, map_count, cur_ts, (int)sys_state);
    }
    if (strcmp(cmd, "reset") == 0) {
        mapper_reset();
        return snprintf(out, out_max, "{\"ok\":\"mapper reset\"}\n");
    }
    if (strcmp(cmd, "ping") == 0) {
        g_watchdog_last_ms = xTaskGetTickCount();
        return snprintf(out, out_max, "{\"ok\":\"pong\"}\n");
    }

    // ── 状态机命令 ──
    if (strcmp(cmd, "arm") == 0) {
        if (sys_state == STATE_RUNNING)
            return snprintf(out, out_max, "{\"err\":\"already running\"}\n");
        sys_state = STATE_ARMED;
        ESP_LOGI(TAG, "ARMED");
        return snprintf(out, out_max, "{\"ok\":\"armed\"}\n");
    }
    if (strcmp(cmd, "go") == 0) {
        if (sys_state == STATE_IDLE) sys_state = STATE_ARMED;
        sys_state = STATE_RUNNING;
        pid_init(&speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 100.0f);
        pid_init(&line_pid,  LINE_PID_KP,  LINE_PID_KI,  LINE_PID_KD,  50.0f);
        ringbuf_init();
        motor_ramp_active = false;
        g_watchdog_last_ms = xTaskGetTickCount();
        ESP_LOGI(TAG, ">>> RUNNING (go)");
        return snprintf(out, out_max, "{\"ok\":\"running\"}\n");
    }
    if (strcmp(cmd, "stop") == 0) {
        sys_state = STATE_IDLE;
        motor_ramp_active = true;
        motor_ramp_step   = 0;
        motor_ramp_thr[0] = (int32_t)(cur_lt * 100);
        motor_ramp_thr[1] = (int32_t)(cur_rt * 100);
        ESP_LOGI(TAG, "STOP (ramp)");
        return snprintf(out, out_max, "{\"ok\":\"stopped\"}\n");
    }
    if (strcmp(cmd, "estop") == 0) {
        sys_state = STATE_IDLE;
        motor_coast();
        ESP_LOGW(TAG, "ESTOP");
        return snprintf(out, out_max, "{\"ok\":\"estop\"}\n");
    }

    // ── Profile 上传 ──
    if (strncmp(cmd, "pload:", 6) == 0) {
        if (sys_state == STATE_RUNNING)
            return snprintf(out, out_max, "{\"err\":\"stop first\"}\n");
        int n = 0;
        char *p = cmd + 6;
        while (*p != ':' && *p != '\0') { n = n * 10 + (*p - '0'); p++; }
        if (*p == ':') p++;
        if (n > 2048) n = 2048;
        for (int i = 0; i < n; i++) {
            uint8_t bytes[4];
            for (int j = 0; j < 4; j++) {
                char c1 = p[i * 8 + j * 2];
                char c2 = p[i * 8 + j * 2 + 1];
                bytes[j] = (hex_to_val(c1) << 4) | hex_to_val(c2);
            }
            memcpy(&profile[i], bytes, 4);
        }
        profile_len = n;
        ESP_LOGI(TAG, "Profile loaded: %d floats", n);
        return snprintf(out, out_max, "{\"ok\":\"loaded\",\"n\":%d}\n", n);
    }

    return snprintf(out, out_max, "{\"err\":\"unknown: %s\"}\n", cmd);
}

void uart_task(void *arg) {
    int idx = 0;
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)(uart_rxbuf + idx), 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            if (uart_rxbuf[idx] == '\n' || uart_rxbuf[idx] == '\r') {
                uart_rxbuf[idx] = '\0';
                if (idx > 0) {
                    char out[512];
                    int rlen = process_command(uart_rxbuf, out, sizeof(out));
                    if (rlen > 0) uart_write_bytes(UART_NUM_0, out, rlen);
                }
                idx = 0;
            } else {
                idx++;
                if (idx >= UART_RXBUF_SIZE) idx = 0;
            }
        } else if (len < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ================== WiFi 命令接收任务 (Core 0) ==================
static void __attribute__((unused)) wifi_cmd_task(void *arg) {
    // Wait for WiFi/lwIP init to complete (done in app_main)
    while (!wifi_ready) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "wifi_cmd: socket failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(COMMAND_UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "wifi_cmd: bind :%d failed", COMMAND_UDP_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "wifi_cmd: listening on UDP :%d", COMMAND_UDP_PORT);

    while (1) {
        char buf[256];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&from, &from_len);
        if (n <= 0) continue;
        buf[n] = '\0';

        // strip trailing \r\n
        while (n > 0 && (buf[n-1] == '\r' || buf[n-1] == '\n')) buf[--n] = '\0';

        if (n == 0) continue;
        ESP_LOGI(TAG, "wifi_cmd: '%s'", buf);

        char out[512];
        int rlen = process_command(buf, out, sizeof(out));
        if (rlen > 0) {
            sendto(sock, out, rlen, 0, (struct sockaddr *)&from, from_len);
        }
    }
}

// ================== 1kHz 控制任务 (Core 1) ==================
static void __attribute__((unused)) control_task(void *arg) {
    // Delay to let Core 0 finish OLED + WiFi init before we touch flash/hw
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "Control on Core 1 @ %d Hz", CONTROL_LOOP_HZ);
    motor_init();
    encoder_init();
    grayscale_init();
    imu_init();
    // imu_calibrate 已移到 planner_task，避免在 1kHz 控制循环中阻塞
    pid_init(&speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 100.0f);
    pid_init(&line_pid,  LINE_PID_KP,  LINE_PID_KI,  LINE_PID_KD,  50.0f);
    vbat_calibrate();
    TickType_t lw = xTaskGetTickCount();

    while (1) {
        // 临时空循环，验证 control_task 本身是否会导致 WDT
        vTaskDelayUntil(&lw, pdMS_TO_TICKS(1));
    }
}

// ================== Core 0 规划 & 遥测 ==================
static void planner_task(void *arg) {
    ESP_LOGI(TAG, "Planner on Core 0 @ %d Hz", MAP_SAMPLE_RATE_HZ);

    // Yield for tcpip task to create its mbox
    vTaskDelay(pdMS_TO_TICKS(50));
#ifndef BARE_OLED_TEST
    ringbuf_init();
#endif
    oled_init();
    oled_start_carousel_task();
#ifndef BARE_OLED_TEST
    imu_calibrate();  // 在 Core 0 规划任务中校准，不阻塞 1kHz 控制循环
#endif

#ifndef BARE_OLED_TEST
    int tick = 0;
#endif
    while (1) {
#ifndef BARE_OLED_TEST
        float avg_spd = (cur_ls + cur_rs) * 0.5f;
        mapper_sample(avg_spd, cur_lp, cur_gz);

        if (sys_state == STATE_RUNNING) {
            control_cmd_t cmd = mapper_predict(total_distance, avg_spd);
            ringbuf_push(cmd);
        }

        tick++;
        if (tick % 5 == 0) {   // 20Hz telemetry (100Hz loop / 5)
            wifi_send_telemetry(cur_lp, cur_ls, cur_rs, cur_ts, cur_gz, cur_lt, cur_rt);
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================== Main ==================
void app_main(void) {
    // 1. 彻底注释网络协议栈 — 隔离 lwIP WDT 根因
    ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "[Qizhi-Cup] ESP32-S3 v0.12 — UDP cmd + WiFi telemetry");

    // 2. 外设初始化
    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    i2c_master_init();
#ifndef BARE_OLED_TEST
    ledc_init();
    // 临时注释掉启动按钮，排查 GPIO 9 中断风暴
    // start_button_init();

    // 3. WiFi — 临时关闭，先验证 DIO flash 基础稳定性
    // vTaskDelay(pdMS_TO_TICKS(500));
    // ESP_ERROR_CHECK(wifi_init());
#endif

    xTaskCreatePinnedToCore(planner_task,  "plan",    8192, NULL, tskIDLE_PRIORITY + 3, NULL, 0);
    // 临时关闭 WiFi/Control 任务，先验证基础稳定性
    // xTaskCreatePinnedToCore(wifi_cmd_task, "wificmd", 4096, NULL, tskIDLE_PRIORITY + 2, NULL, 0);
    xTaskCreatePinnedToCore(control_task,  "ctrl",    8192, NULL, tskIDLE_PRIORITY + 4, NULL, 1);
    xTaskCreatePinnedToCore(uart_task,     "uart",    4096, NULL, tskIDLE_PRIORITY + 2, NULL, 0);
}
