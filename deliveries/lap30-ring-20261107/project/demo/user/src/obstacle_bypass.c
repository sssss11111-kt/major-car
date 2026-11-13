/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "obstacle_bypass.h"

static int valid_config(const obstacle_bypass_config_t *c)
{
    if (!c || !c->points || c->point_count < 2 || c->point_count > 16 ||
        !c->counts_per_m || !c->max_counts_step || c->max_counts_step > INT32_MAX ||
        !c->vehicle_clearance_mm || !c->obstacle_uncertainty_mm || !c->front_buffer_mm ||
        !c->brick_depth_mm || !c->rear_buffer_mm || !c->return_mm || !c->cooldown_mm ||
        !c->min_turn_radius_mm || !c->max_slope_permille || !c->range_tolerance_mm)
        return 0;
    for (unsigned i = 0; i < c->point_count; ++i) {
        obstacle_cal_point_t p = c->points[i];
        if (p.row >= OBS_HEIGHT || !p.distance_mm || !p.mm_per_px_q8 ||
            (i && (p.row <= c->points[i - 1].row || p.distance_mm >= c->points[i - 1].distance_mm)))
            return 0;
    }
    return 1;
}

static int interpolate(int a, int b, int numerator, int denominator)
{
    return a + (int)((int32_t)(b - a) * numerator / denominator);
}

static obstacle_cal_point_t project(const obstacle_bypass_config_t *c, int row)
{
    if (row < c->points[0].row) return (obstacle_cal_point_t){0};
    for (unsigned i = 1; i < c->point_count; ++i) {
        obstacle_cal_point_t a = c->points[i - 1], b = c->points[i];
        if (row <= b.row)
            return (obstacle_cal_point_t){(uint8_t)row,
                (uint16_t)interpolate(a.distance_mm, b.distance_mm, row - a.row, b.row - a.row),
                (uint16_t)interpolate(a.mm_per_px_q8, b.mm_per_px_q8, row - a.row, b.row - a.row)};
    }
    return (obstacle_cal_point_t){0};
}

static int magnitude(int v) { return v < 0 ? -v : v; }

static int ramp_feasible(const obstacle_bypass_config_t *c, int offset, uint32_t length)
{
    uint32_t shift = (uint32_t)magnitude(offset);
    /* For cubic smoothstep the maximum slope is 1.5*shift/L and
     * maximum |second derivative| is 6*shift/L^2. */
    return length && (uint64_t)1500 * shift <= (uint64_t)c->max_slope_permille * length &&
           (uint64_t)6 * shift * c->min_turn_radius_mm <= (uint64_t)length * length;
}

static int offset_fits(const obstacle_bypass_config_t *c, int offset, int left, int right)
{
    return left >= c->vehicle_clearance_mm && right >= c->vehicle_clearance_mm &&
           offset >= c->vehicle_clearance_mm - left && offset <= right - c->vehicle_clearance_mm;
}

static int plan(obstacle_bypass_t *s, const obstacle_bypass_config_t *c,
                const obstacle_bypass_input_t *in)
{
    const obstacle_candidate_t *b = &in->target.best;
    obstacle_cal_point_t at = project(c, b->y_max);
    int center = in->lane_center_x;
    if (at.distance_mm <= c->front_buffer_mm || in->lane_left_x >= center ||
        center >= in->lane_right_x || in->lane_right_x >= OBS_WIDTH ||
        b->y_min > b->y_max || b->x_min > b->x_max ||
        b->x_min <= in->lane_left_x || b->x_max >= in->lane_right_x)
        return 0;
    int px_scale = at.mm_per_px_q8;
    int clearance = c->vehicle_clearance_mm + c->obstacle_uncertainty_mm;
    int left_mm = (center - in->lane_left_x) * px_scale / 256;
    int right_mm = (in->lane_right_x - center) * px_scale / 256;
    if (left_mm > in->corridor_left_mm) left_mm = in->corridor_left_mm;
    if (right_mm > in->corridor_right_mm) right_mm = in->corridor_right_mm;
    /* Round the obstacle outward before adding uncertainty. */
    int left = (b->x_min - center) * px_scale / 256 - clearance - 1;
    int right = (b->x_max - center) * px_scale / 256 + clearance + 1;
    if (left > 0) left = 0;
    if (right < 0) right = 0;
    uint32_t approach = (uint32_t)(at.distance_mm - c->front_buffer_mm);
    int can_left = left >= INT16_MIN && offset_fits(c, left, left_mm, right_mm) &&
                   ramp_feasible(c, left, approach) && ramp_feasible(c, left, c->return_mm);
    int can_right = right <= INT16_MAX && offset_fits(c, right, left_mm, right_mm) &&
                    ramp_feasible(c, right, approach) && ramp_feasible(c, right, c->return_mm);
    if (!can_left && !can_right) return 0;
    s->target_offset_mm = (int16_t)(can_left && (!can_right || -left <= right) ? left : right);
    s->obstacle_mm = at.distance_mm;
    /* Aim to finish the shift one eighth earlier when geometry permits. */
    uint32_t early_approach = approach - approach / 8;
    s->approach_mm = ramp_feasible(c, s->target_offset_mm, early_approach)
        ? early_approach : approach;
    s->rear_clear_mm = (uint32_t)at.distance_mm + c->brick_depth_mm + c->rear_buffer_mm;
    s->previous_counts = in->forward_counts;
    s->traveled_counts = 0;
    s->phase = OBS_BYPASS_APPROACH;
    return 1;
}

static int16_t ramp(int16_t offset, uint32_t progress, uint32_t length)
{
    /* Q10 smoothstep, zero slope at both ends. Caller clamps the progress. */
    uint32_t t = progress * 1024u / length;
    uint32_t blend = t * t * (3072u - 2u * t) / (1024u * 1024u);
    return (int16_t)(offset * (int32_t)blend / 1024);
}

static obstacle_bypass_result_t stop(obstacle_bypass_t *s)
{
    if (s) s->phase = OBS_BYPASS_STOP;
    return (obstacle_bypass_result_t){.phase = OBS_BYPASS_STOP, .stop_requested = 1};
}

obstacle_bypass_result_t obstacle_bypass_update(obstacle_bypass_t *s,
    const obstacle_bypass_config_t *c, const obstacle_bypass_input_t *in)
{
    if (!s || !in || in->emergency_stop || s->phase == OBS_BYPASS_STOP) return stop(s);
    int confirmed = in->target.found && in->target.confirmed;
    if (!in->route_permitted) {
        if (s->phase != OBS_BYPASS_IDLE) return stop(s);
        return (obstacle_bypass_result_t){0};
    }
    if (!valid_config(c)) {
        if (s->phase != OBS_BYPASS_IDLE || confirmed) return stop(s);
        return (obstacle_bypass_result_t){0};
    }
    if (s->phase == OBS_BYPASS_IDLE) {
        if (!confirmed) return (obstacle_bypass_result_t){0};
        if (!in->odometry_valid || in->target.candidate_count != 1 ||
            in->target.overflow || !plan(s, c, in)) return stop(s);
        return (obstacle_bypass_result_t){.phase = s->phase};
    }

    uint32_t delta = in->forward_counts - s->previous_counts;
    if (!in->odometry_valid || delta > c->max_counts_step || in->target.overflow ||
        !offset_fits(c, s->target_offset_mm, in->corridor_left_mm, in->corridor_right_mm) ||
        in->target.candidate_count > 1) return stop(s);
    s->previous_counts = in->forward_counts;
    s->traveled_counts += delta;
    uint64_t traveled = s->traveled_counts * 1000u / c->counts_per_m;
    if (s->phase == OBS_BYPASS_APPROACH && !in->target.found) return stop(s);
    if (in->target.found && (s->phase == OBS_BYPASS_APPROACH || confirmed)) {
        uint16_t distance = project(c, in->target.best.y_max).distance_mm;
        int expected = traveled < s->obstacle_mm ? (int)(s->obstacle_mm - traveled) : 0;
        if (!distance || magnitude((int)distance - expected) > c->range_tolerance_mm) return stop(s);
    }

    int16_t offset = 0;
    uint32_t return_end = s->rear_clear_mm + c->return_mm;
    if (traveled < s->obstacle_mm - c->front_buffer_mm) {
        s->phase = OBS_BYPASS_APPROACH;
        offset = traveled < s->approach_mm
            ? ramp(s->target_offset_mm, (uint32_t)traveled, s->approach_mm)
            : s->target_offset_mm;
    } else if (traveled < s->rear_clear_mm) {
        s->phase = OBS_BYPASS_PASS;
        offset = s->target_offset_mm;
    } else if (traveled < return_end) {
        s->phase = OBS_BYPASS_RETURN;
        offset = ramp(s->target_offset_mm, return_end - (uint32_t)traveled, c->return_mm);
    } else if (traveled < return_end + c->cooldown_mm || in->target.found) {
        s->phase = OBS_BYPASS_COOLDOWN;
    } else {
        *s = (obstacle_bypass_t){0};
    }
    return (obstacle_bypass_result_t){.phase = s->phase, .offset_mm = offset};
}

uint16_t obstacle_bypass_scale_at_row(const obstacle_bypass_config_t *c, uint8_t row)
{
    return valid_config(c) ? project(c, row).mm_per_px_q8 : 0;
}
