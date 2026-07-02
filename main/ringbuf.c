#include "ringbuf.h"
#include "config.h"
#include "esp_timer.h"

static control_cmd_t buf[RINGBUF_SIZE];
static volatile int hd = 0, tl = 0;

void ringbuf_init(void) { hd = tl = 0; }

void ringbuf_push(control_cmd_t cmd) {
    int n = (hd + 1) % RINGBUF_SIZE;
    if (n != tl) {
        buf[hd] = cmd;
        __sync_synchronize();
        hd = n;
    }
}

control_cmd_t ringbuf_pop(void) {
    control_cmd_t e = {false, 0, 0, 0};
    if (hd == tl) return e;
    control_cmd_t c = buf[tl];
    __sync_synchronize();
    tl = (tl + 1) % RINGBUF_SIZE;
    return c;
}

bool ringbuf_empty(void) { return hd == tl; }

bool ringbuf_is_fresh(control_cmd_t cmd) {
    if (!cmd.active) return false;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    int32_t age = (int32_t)(now - cmd.timestamp_ms);
    return (age >= 0 && age < 15);
}
