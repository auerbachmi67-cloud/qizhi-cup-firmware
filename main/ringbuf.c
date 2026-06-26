#include "ringbuf.h"
#include "config.h"
static control_cmd_t buf[RINGBUF_SIZE];
static volatile int hd = 0, tl = 0;
void ringbuf_init(void) { hd = tl = 0; }
void ringbuf_push(control_cmd_t cmd) {
    int n = (hd + 1) % RINGBUF_SIZE;
    if (n != tl) {
        buf[hd] = cmd;
        __sync_synchronize(); // store buf[hd] before hd (cross-core visibility)
        hd = n;
    }
}
control_cmd_t ringbuf_pop(void) {
    control_cmd_t e = {false, 0, 0, 0};
    if (hd == tl) return e;
    control_cmd_t c = buf[tl];
    __sync_synchronize(); // ensure buf[] read happens after tl read
    tl = (tl + 1) % RINGBUF_SIZE;
    return c;
}
bool ringbuf_empty(void) { return hd == tl; }
