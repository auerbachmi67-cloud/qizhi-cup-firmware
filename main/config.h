#ifndef CONFIG_H
#define CONFIG_H
#define CONTROL_LOOP_HZ  1000
#define PID_OUTPUT_LIMIT 100.0f
#define SPEED_PID_KP 1.5f
#define SPEED_PID_KI 0.1f
#define SPEED_PID_KD 0.05f
#define LINE_PID_KP  2.0f
#define LINE_PID_KI  0.2f
#define LINE_PID_KD  0.1f  // curvature → steering feedforward (m²)
#define WHEEL_DIAMETER_MM 22.0f
#define PULSES_PER_REV 40
#define ENCODER_GEAR_RATIO 10.0f
#define TRACK_WIDTH_MM 160.0f
#define MAX_SPEED_MPS 3.5f
#define MAP_SAMPLE_RATE_HZ 100
#define RINGBUF_SIZE 512
#define TELEMETRY_UDP_PORT 3333
#define WIFI_UDP_TARGET_IP "192.168.4.2"
#define WIFI_SSID "qizhi-cup"
#define WIFI_PASS "12345678"
#endif
