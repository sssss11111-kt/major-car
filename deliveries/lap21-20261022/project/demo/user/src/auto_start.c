#include "auto_start.h"
auto_phase_t auto_start_step(auto_start_t *state, uint32_t tick,
                           uint8_t track_ready, uint8_t inhibited,
                           uint8_t locked)
{
    if (locked || state->phase == AUTO_LOCKED) {
        state->phase = AUTO_LOCKED;
        return state->phase;
    }
    if (state->phase == AUTO_STARTED) return state->phase;
    if (!track_ready || inhibited) {
        state->phase = AUTO_WAIT;
        return state->phase;
    }
    if (state->phase == AUTO_WAIT) {
        state->center_since = tick;
        state->phase = AUTO_CENTER;
    } else if ((uint32_t)(tick - state->center_since) >= AUTO_CENTER_TICKS) {
        state->phase = AUTO_STARTED;
    }
    return state->phase;
}
