#ifndef CONFIG_H
#define CONFIG_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define CONTROL_LOOP_HZ   1000
#define PID_OUTPUT_LIMIT   1.0f

#define SPEED_PID_KP  1.5f
#define SPEED_PID_KI  0.1f
#define SPEED_PID_KD  0.05f

#define LINE_PID_KP   2.0f
#define LINE_PID_KI   0.2f
#define LINE_PID_KD   0.1f
#define LINE_PID_FF_RATIO  0.3f

#define GYRO_TO_DIFF   0.000443f
#define FF_OMEGA_GAIN  0.0005f

#define WHEEL_DIAMETER_MM   22.0f
#define PULSES_PER_REV      40
#define ENCODER_GEAR_RATIO  10.0f
#define TRACK_WIDTH_MM      160.0f

#define MAX_SPEED_MPS       3.5f
#define MAP_SAMPLE_RATE_HZ  100
#define RINGBUF_SIZE        512

// State machine & safety
#define START_BUTTON_GPIO    9
#define BUTTON_DEBOUNCE_MS  50
#define WATCHDOG_TIMEOUT_MS 500
#define BATTERY_NOMINAL_V    11.1f

// Communication
#define TELEMETRY_UDP_PORT    3333
#define COMMAND_UDP_PORT      3334
#define WIFI_UDP_TARGET_IP    "192.168.4.2"
#define WIFI_SSID             "mcy4"
#define WIFI_PASS             "mcy12306"

// Global I2C mutex for cross-core OLED/IMU access
extern SemaphoreHandle_t i2c_mutex;

#include <stdbool.h>
bool is_system_running(void);

#endif
