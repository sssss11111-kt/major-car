#ifndef OBSTACLE_DETECT_H
#define OBSTACLE_DETECT_H

#include <stdint.h>

#ifndef OBSTACLE_SHADOW_ENABLED
#define OBSTACLE_SHADOW_ENABLED 0
#endif

enum { OBS_WIDTH = 188, OBS_HEIGHT = 120, OBS_PIXELS = OBS_WIDTH * OBS_HEIGHT };

typedef struct {
    uint8_t left, right, measured;
} obstacle_lane_row_t;

typedef struct {
    uint8_t x_min, y_min, x_max, y_max;
    uint8_t mean_gray, contrast, edge_columns;
    uint16_t area, fill_permille, binary_dark_permille;
} obstacle_candidate_t;

typedef struct {
    uint32_t last_frame;
    obstacle_candidate_t last;
    uint8_t hits;
} obstacle_track_t;

typedef struct {
    obstacle_candidate_t best;
    uint16_t candidate_count;
    uint8_t found, confirmed, hits, overflow, interpolated_rows;
} obstacle_result_t;

/* Main-loop only; shared fixed workspace. Clear track on camera restart.
 * Input pixels and lane rows are read-only. Supply either a 0/nonzero binary
 * image or the vision owner's per-row thresholds (binary takes precedence).
 * The output box describes the visible dark face in image coordinates. */
obstacle_result_t obstacle_detect_frame(const uint8_t gray[OBS_PIXELS],
    const uint8_t *binary, const uint8_t *thresholds,
    const obstacle_lane_row_t lane[OBS_HEIGHT], uint32_t frame, obstacle_track_t *track);

#endif
