#include "obstacle_bypass.h"
#include <assert.h>
#include <stdio.h>

/* Synthetic geometry. No dimensions in this file are vehicle calibration. */
static const obstacle_cal_point_t points[] = {{40, 800, 512}, {70, 500, 512}, {100, 200, 512}};
static const obstacle_bypass_config_t config = {
    .points = points, .point_count = 3, .counts_per_m = 1000, .max_counts_step = 100,
    .vehicle_clearance_mm = 60, .obstacle_uncertainty_mm = 20, .front_buffer_mm = 100,
    .brick_depth_mm = 100, .rear_buffer_mm = 120, .return_mm = 500, .cooldown_mm = 100,
    .min_turn_radius_mm = 200, .max_slope_permille = 700, .range_tolerance_mm = 180
};

static obstacle_bypass_input_t scene(void)
{
    return (obstacle_bypass_input_t){
        .target = {.found = 1, .confirmed = 1, .candidate_count = 1,
                   .best = {.x_min = 85, .x_max = 105, .y_min = 60, .y_max = 70}},
        .lane_left_x = 10, .lane_center_x = 95, .lane_right_x = 180,
        .corridor_left_mm = 170, .corridor_right_mm = 170, .route_permitted = 1,
        .odometry_valid = 1
    };
}

int main(void)
{
    obstacle_bypass_t state = {0};
    obstacle_bypass_input_t in = scene();
    assert(obstacle_bypass_update(&state, 0, &in).stop_requested);
    state = (obstacle_bypass_t){0};
    in.target.confirmed = 0;
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_IDLE);
    in = scene();
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_APPROACH);
    assert(state.target_offset_mm == -101); /* Equal cost, including outward rounding. */
    for (int i = 1; i <= 4; ++i) {
        in.forward_counts = (uint32_t)(i * 100);
        in.target.best.y_max = (uint8_t)(70 + i * 7);
        obstacle_bypass_result_t r = obstacle_bypass_update(&state, &config, &in);
        assert(r.phase == (i == 4 ? OBS_BYPASS_PASS : OBS_BYPASS_APPROACH));
        assert(r.offset_mm < 0 && r.offset_mm >= -101);
    }
    in.target = (obstacle_result_t){0};
    for (uint32_t distance = 500; distance <= 700; distance += 100) {
        in.forward_counts = distance;
        assert(obstacle_bypass_update(&state, &config, &in).offset_mm == -101);
    }
    in.forward_counts = 800;
    obstacle_bypass_result_t r = obstacle_bypass_update(&state, &config, &in);
    assert(r.phase == OBS_BYPASS_RETURN && r.offset_mm < 0 && r.offset_mm > -101);
    for (uint32_t distance = 900; distance <= 1400; distance += 100) {
        in.forward_counts = distance;
        r = obstacle_bypass_update(&state, &config, &in);
    }
    assert(r.phase == OBS_BYPASS_IDLE && r.offset_mm == 0);
    in.target = scene().target; /* A second brick after returning and cooldown. */
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_APPROACH);

    state = (obstacle_bypass_t){0};
    in = scene();
    in.corridor_left_mm = 130;
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_APPROACH);
    assert(state.target_offset_mm == 101);
    in.emergency_stop = 1;
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);
    in.emergency_stop = 0;
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);

    state = (obstacle_bypass_t){0};
    in = scene();
    in.corridor_right_mm = 130;
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_APPROACH);
    assert(state.target_offset_mm == -101);
    in.route_permitted = 0;
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);
    state = (obstacle_bypass_t){0};
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_IDLE);

    state = (obstacle_bypass_t){0};
    in = scene();
    in.corridor_left_mm = in.corridor_right_mm = 130;
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);

    state = (obstacle_bypass_t){0};
    in = scene();
    in.target.best.x_min = 15;
    in.target.best.x_max = 33;
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_APPROACH);
    assert(state.target_offset_mm == 0); /* Center already clears a near-edge brick. */

    state = (obstacle_bypass_t){0};
    in = scene();
    in.target.best.y_max = 100; /* Too close to meet the curvature bound. */
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);

    state = (obstacle_bypass_t){0};
    in = scene();
    in.forward_counts = UINT32_MAX - 10;
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_APPROACH);
    in.forward_counts = 20; /* A valid wraparound. */
    assert(!obstacle_bypass_update(&state, &config, &in).stop_requested);
    in.forward_counts = 19; /* An invalid reverse sample. */
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);

    state = (obstacle_bypass_t){0};
    in = scene();
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_APPROACH);
    in.target.found = 0;
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);

    state = (obstacle_bypass_t){0};
    in = scene();
    in.target.candidate_count = 2;
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);
    state = (obstacle_bypass_t){0};
    in = scene();
    in.odometry_valid = 0;
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);
    state = (obstacle_bypass_t){0};
    in = scene();
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_APPROACH);
    in.corridor_left_mm = 100;
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);

    state = (obstacle_bypass_t){0};
    in = scene();
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_APPROACH);
    in.forward_counts = 101;
    assert(obstacle_bypass_update(&state, &config, &in).stop_requested);

    obstacle_bypass_config_t invalid = config;
    invalid.counts_per_m = 0;
    state = (obstacle_bypass_t){0};
    in = scene();
    assert(obstacle_bypass_update(&state, &invalid, &in).stop_requested);
    state = (obstacle_bypass_t){0};
    assert(obstacle_bypass_update(&state, &config, &in).phase == OBS_BYPASS_APPROACH);
    assert(obstacle_bypass_update(&state, &invalid, &in).stop_requested);
    invalid = config;
    static const obstacle_cal_point_t reversed[] = {{70, 500, 512}, {40, 800, 512}};
    invalid.points = reversed;
    invalid.point_count = 2;
    state = (obstacle_bypass_t){0};
    assert(obstacle_bypass_update(&state, &invalid, &in).stop_requested);
    puts("PASS: calibration gating, safe sides, central/edge targets, rear clear, return, cooldown, second brick, blocked paths, odometry, lost target and emergency");
    return 0;
}
