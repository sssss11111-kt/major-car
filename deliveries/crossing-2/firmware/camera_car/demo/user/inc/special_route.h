/* Special-track diagnostic FSM and image-space path contracts. GPL-3.0-or-later. */
#ifndef SPECIAL_ROUTE_H
#define SPECIAL_ROUTE_H

#include "car_core.h"

#define SPECIAL_PATH_POINTS 12

typedef enum { PORT_UNKNOWN, PORT_AUTO, PORT_A, PORT_B, PORT_C, PORT_D } special_port_t;
typedef enum { ROUTE_CCW = -1, ROUTE_CW = 1 } special_direction_t;

typedef struct {
    uint8_t enabled, route_valid;
    special_port_t entry_port, exit_port;
    special_direction_t direction;
} special_route_config_t;

typedef struct {
    uint8_t stable_frames;
    uint16_t loss_timeout_ms, maneuver_timeout_ms;
} special_fsm_config_t;

/* Only current-frame observations. Ports and progress require calibrated evidence. */
typedef struct {
    special_port_t entry_port, exit_port;
    special_direction_t direction;
    uint8_t path_current, entry_aligned, entry_progress_ready;
    uint8_t entry_clearance_valid; /* Current entry segment passed special_ring_entry_target(). */
    uint8_t follow_established, exit_search_ready, exit_crossed, road_recovered;
} special_route_observation_t;

typedef struct {
    uint32_t started_ms, last_path_ms;
    uint8_t stable;
} special_fsm_timer_t;

typedef struct { int16_t x, y; } special_path_point_t;
typedef struct {
    /* Camera coordinates: x right, y down, points ordered near to far. */
    uint8_t count;
    special_path_point_t points[SPECIAL_PATH_POINTS];
} special_path_t;

typedef struct {
    special_path_t entry, follow, exit;
    /* Current inner boundary indexed by image row, -1 for missing. */
    int16_t entry_inner_x[CAR_H];
    int16_t entry_reference_x; /* Measured ordinary center at the first path point's y. */
    /* Side is -1 left, +1 right. The evidence must cover the entire entry segment. */
    int8_t entry_inner_side;
    uint8_t entry_inner_top, entry_inner_bottom;
    uint8_t current, calibrated, lookahead_y;
    float entry_min_clearance_px, entry_max_curvature_px_inv, entry_max_join_offset_px;
} special_plan_evidence_t;

typedef struct {
    uint8_t target_valid;
    float target_error, curvature_px_inv, max_curvature_px_inv;
} special_plan_output_t;

int special_route_valid(const special_route_config_t *route);
int special_fsm_expired(special_fsm_timer_t *timer, const special_fsm_config_t *cfg,
                        int path_current, uint32_t now);
int special_fsm_stable(special_fsm_timer_t *timer, unsigned required, int condition);

/* A missing boundary is -1. A single boundary requires a calibrated width. */
int special_path_center(int left, int right, unsigned width, int16_t *center);
int special_path_target(const special_path_t *path, unsigned lookahead_y,
                        special_plan_output_t *out);
int special_ring_entry_target(const special_plan_evidence_t *evidence,
                              special_plan_output_t *out);

#endif
