#ifndef AUTO_START_H
#define AUTO_START_H
#include <stdint.h>

#define AUTO_CENTER_TICKS 100U /* 100 x 10 ms = 1 second */
typedef enum {
    AUTO_WAIT = 0,
    AUTO_CENTER = 1,
    AUTO_STARTED = 2,
    AUTO_LOCKED = 3
} auto_phase_t;
typedef struct {
    auto_phase_t phase;
    uint32_t center_since;
} auto_start_t;

auto_phase_t auto_start_step(auto_start_t *state, uint32_t tick,
                           uint8_t track_ready, uint8_t inhibited,
                           uint8_t locked);
#endif
