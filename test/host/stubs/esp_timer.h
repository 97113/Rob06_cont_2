// Host stub: a virtual microsecond clock.
// Time advances only inside twai_receive(), which models "time passes while
// you poll the bus" and keeps every test deterministic.
#pragma once
#include <stdint.h>
extern int64_t g_fake_now_us;
static inline int64_t esp_timer_get_time() { return g_fake_now_us; }
