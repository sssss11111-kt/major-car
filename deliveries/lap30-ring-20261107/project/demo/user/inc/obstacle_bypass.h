#ifndef OBSTACLE_BYPASS_H
#define OBSTACLE_BYPASS_H

#include "obstacle_detect.h"

typedef struct {
    uint8_t row;
    uint16_t distance_mm, mm_per_px_q8;
} obstacle_cal_point_t;

typedef struct {
    const obstacle_cal_point_t *points; /* Sorted by increasing row, decreasing distance. */
    uint8_t point_count;
    uint32_t counts_per_m, max_counts_step;
    uint16_t vehicle_clearance_mm; /* Half width, turn sweep and boundary/latency margin. */
    uint16_t obstacle_uncertainty_mm, front_buffer_mm, brick_depth_mm, rear_buffer_mm;
    uint16_t return_mm, cooldown_mm, min_turn_radius_mm, max_slope_permille, range_tolerance_mm;
} obstacle_bypass_config_t;

typedef enum {
    OBS_BYPASS_IDLE, OBS_BYPASS_APPROACH, OBS_BYPASS_PASS,
    OBS_BYPASS_RETURN, OBS_BYPASS_COOLDOWN, OBS_BYPASS_STOP
} obstacle_bypass_phase_t;

typedef struct {
    obstacle_result_t target;
    uint8_t lane_left_x, lane_center_x, lane_right_x; /* At target y_max, measured. */
    uint16_t corridor_left_mm, corridor_right_mm;   /* Minimum swept-path clearances from center. */
    uint8_t route_permitted, odometry_valid, emergency_stop;
    uint32_t forward_counts; /* Cumulative signed-forward measurement, stored modulo 2^32. */
} obstacle_bypass_input_t;

typedef struct {
    obstacle_bypass_phase_t phase;
    int16_t target_offset_mm;
    uint32_t previous_counts, obstacle_mm, approach_mm, rear_clear_mm;
    uint64_t traveled_counts;
} obstacle_bypass_t;

typedef struct {
    obstacle_bypass_phase_t phase;
    int16_t offset_mm; /* Positive is vehicle right. */
    uint8_t stop_requested;
} obstacle_bypass_result_t;

/* Offline planner. Zero-initialize state. Keep calibration fixed through
 * each plan. A stop is latched until an explicit supervisor reset. Supply
 * route_permitted only on calibrated clear corridors, never at special
 * elements such as roundabouts and intersections. */
obstacle_bypass_result_t obstacle_bypass_update(obstacle_bypass_t *state,
    const obstacle_bypass_config_t *config, const obstacle_bypass_input_t *input);

uint16_t obstacle_bypass_scale_at_row(const obstacle_bypass_config_t *config, uint8_t row);

#endif
