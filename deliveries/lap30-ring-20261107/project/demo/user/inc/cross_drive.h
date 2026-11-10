#ifndef CROSS_DRIVE_H
#define CROSS_DRIVE_H

#include "special_track.h"

typedef enum { CROSS_DRIVE_IDLE, CROSS_DRIVE_FOLLOW, CROSS_DRIVE_EXIT, CROSS_DRIVE_LOST } cross_drive_phase_t;
typedef struct {
    cross_drive_phase_t phase;
    int8_t side;
    uint8_t clear_frames, saw_split;
    int16_t last_near_error;
    uint32_t started_tick;
} cross_drive_t;
typedef struct {
    uint8_t active, valid;
    int16_t near_error, preview_error;
} cross_drive_output_t;

/* Called in the foreground. A lost route stays invalid until reset. */
cross_drive_output_t cross_drive_step(cross_drive_t *state, const special_track_output_t *scan,
                                     const uint8_t *raw, const uint8_t *thresholds, uint32_t tick,
                                     int running, int roundabout_busy, int road_recovered);

#endif
