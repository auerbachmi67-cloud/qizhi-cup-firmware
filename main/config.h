#ifndef CONFIG_H
#define CONFIG_H

#define CONTROL_LOOP_HZ   1000
#define PID_OUTPUT_LIMIT   100.0f

#define SPEED_PID_KP  1.5f
#define SPEED_PID_KI  0.1f
#define SPEED_PID_KD  0.05f

#define LINE_PID_KP   2.0f
#define LINE_PID_KI   0.2f
#define LINE_PID_KD   0.1f
#define LINE_PID_FF_RATIO  0.3f   // reduce line PID to 30% when FF active

// Physical conversion: ω (°/s) → differential throttle
// Δv = ω_rad × track_width = ω_dps × π/180 × 0.160
// diff = Δv / (2 × V_ss_max) = ω_dps × 0.00279 / 6.3
#define GYRO_TO_DIFF   0.000443f   // (°/s) → throttle diff, theory
#define FF_OMEGA_GAIN  0.0005f     // tune experimentally

#define WHEEL_DIAMETER_MM   22.0f
#define PULSES_PER_REV      40
#define ENCODER_GEAR_RATIO  10.0f
#define TRACK_WIDTH_MM      160.0f

#define MAX_SPEED_MPS       3.5f
#define MAP_SAMPLE_RATE_HZ  100
#define RINGBUF_SIZE        512

// ── State machine & safety ───────────────────────────────────────────────
#define START_BUTTON_GPIO    9    // GPIO 9, internal pull-up, button to GND
#define BUTTON_DEBOUNCE_MS  50
#define WATCHDOG_TIMEOUT_MS 500  // PC heartbeat → auto-stop
#define BATTERY_NOMINAL_V    11.1f

// ── Communication ─────────────────────────────────────────────────────────
#define TELEMETRY_UDP_PORT    3333
#define WIFI_UDP_TARGET_IP    "192.168.4.2"
#define WIFI_SSID             "mcy4"
#define WIFI_PASS             "mcy12306"

#endif
