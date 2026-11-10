/* Fixed-capacity special-track detection. GPL-3.0-or-later. */
#ifndef SPECIAL_TRACK_H
#define SPECIAL_TRACK_H

#include "cross_island.h"
#include "roundabout.h"

#define SPECIAL_SCAN_ROWS 12
#define SPECIAL_MONITOR_ROWS 6
#define SPECIAL_MAX_RUNS_PER_ROW 4
#define SPECIAL_VOTE_WINDOW 5

typedef enum { SPECIAL_NONE, SPECIAL_CROSS_ISLAND, SPECIAL_ROUNDABOUT, SPECIAL_UNKNOWN }
    special_track_type_t;
typedef enum {
    SPECIAL_IDLE, SPECIAL_CANDIDATE, SPECIAL_ACTIVE_CROSS, SPECIAL_ACTIVE_ROUNDABOUT,
    SPECIAL_UNKNOWN_STATE, SPECIAL_COOLDOWN, SPECIAL_FAULT
} special_track_state_id_t;

enum {
    SPECIAL_DIAG_LOW_CONTRAST = 1u << 0,
    SPECIAL_DIAG_OVERFLOW     = 1u << 1,
    SPECIAL_DIAG_WEAK         = 1u << 2,
    SPECIAL_DIAG_CONFLICT     = 1u << 3,
    SPECIAL_DIAG_NO_APPROACH  = 1u << 4,
    SPECIAL_DIAG_TIMEOUT      = 1u << 5,
    SPECIAL_DIAG_CONFIG       = 1u << 6,
    SPECIAL_DIAG_ROUTE_DISABLED = 1u << 7
};

typedef struct { int16_t left, right, center; uint16_t width; } special_run_t;
typedef struct {
    uint8_t y, run_count, overflow;
    special_run_t runs[SPECIAL_MAX_RUNS_PER_ROW];
} special_scan_row_t;
typedef struct {
    uint8_t row_count, overflow, usable_rows, multi_run_rows, central_dark_rows;
    uint8_t curved_boundary_rows, normal_edge_loss_rows, edge_breaks, clipped_bank_rows, ambiguous;
    uint8_t linked_gap_rows, cross_rows, ring_rows, ring_conflict;
    int8_t island_side;
    special_scan_row_t rows[SPECIAL_SCAN_ROWS];
} special_scan_result_t;

typedef struct {
    uint8_t scan_top, scan_bottom, monitor_rows, min_run_width, min_channel_width;
    uint8_t min_gap_width, center_tolerance, max_row_shift, min_curve_step, min_contrast;
    uint8_t min_cross_rows, min_ring_rows, min_arc_rows;
    uint8_t candidate_on, candidate_off, min_confidence, vote_window, vote_required;
    uint8_t recover_frames;
    uint16_t cooldown_ms, candidate_timeout_ms, approach_window_ms, loss_timeout_ms;
    uint16_t active_timeout_ms;
} special_track_config_t;

typedef struct { special_track_type_t type; int8_t island_side; } special_vote_t;

typedef struct {
    special_track_state_id_t state;
    uint8_t candidate, vote_pos, vote_count, vote_window, recovery, approach_seen, frame_seen;
    int8_t confirmed_side;
    uint32_t state_since_ms, last_support_ms, approach_ms, frame_ms;
    special_vote_t votes[SPECIAL_VOTE_WINDOW];
    special_scan_result_t scan;
    cross_island_t cross;
    roundabout_t round;
} special_track_state_t;

typedef struct {
    special_track_type_t type, frame_type;
    special_track_state_id_t state;
    uint8_t candidate, candidate_score, confidence, full_scan_attempted, full_scan_valid;
    uint8_t route_ready, target_valid, diagnostic_flags;
    int8_t island_side;
    cross_island_state_id_t cross_state;
    roundabout_state_id_t round_state;
    float target_error;
} special_track_output_t;

void special_track_config_default(special_track_config_t *cfg);
void special_track_init(special_track_state_t *state);
void special_track_update(special_track_state_t *state, const special_track_config_t *cfg,
                          const uint8_t *binary, const vision_result_t *normal,
                          uint32_t now, special_track_output_t *out);

#endif
