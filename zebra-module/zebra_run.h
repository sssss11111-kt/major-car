#ifndef ZEBRA_RUN_H
#define ZEBRA_RUN_H

#include <stdint.h>


#define ZEBRA_FORWARD_METERS 0.25f
#define ZEBRA_FORWARD_TICKS 30U /* 10 ms control period */
#define ZEBRA_REARM_METERS 1.0f
#define ZEBRA_REVERSE_LIMIT_METERS 0.65f
#define ZEBRA_CLEAR_FRAMES 5U
#define ZEBRA_CONFIRM_FRAMES 3U
#define ZEBRA_SETTLED_TICKS 10U

enum {
    ZEBRA_WAIT, ZEBRA_LEAVE, ZEBRA_LAP, ZEBRA_FORWARD,
    ZEBRA_SETTLE, ZEBRA_REVERSE, ZEBRA_DONE, ZEBRA_ABORT
};

typedef struct {
    uint32_t last_frame, phase_tick;
    int32_t count_sum;
    uint8_t phase, clear_frames, confirm_frames, still_ticks;
} zebra_run_t;

void zebra_run_step(volatile zebra_run_t *run, uint32_t tick,
                                  uint32_t frame, uint16_t bottom, int stop_row,
                                  int left_counts, int right_counts, float meters_per_count);

/* Call once when the first current stripe and valid track permit launch. */
void zebra_run_start(volatile zebra_run_t *run, uint32_t frame);

/* Gray8 image, one threshold per image row. ROI endpoints are inclusive.
 * Returns the lowest confirmed stripe row, or zero for no stripe. */
uint16_t zebra_detect_bottom(const uint8_t *gray, int width, int height,
                             int stride, const uint8_t *row_threshold,
                             int top, int bottom);
#endif
