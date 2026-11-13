#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#define RING_ENTRY_IMPLEMENTATION
#include "img_process.h"

static uint8_t raw[188 * 120], thresholds[120];
static entry_path_t path;

static uint8_t island_left[120], island_right[120], island_mask[120], half_width[120];

static void check_current_branch(void) {
    entry_context_t context = {0};
    context.active = context.fresh_island = 1;
    context.island_left = island_left;
    context.island_right = island_right;
    context.island_mask = island_mask;
    context.half_width = half_width;
    memset(half_width, 40, sizeof half_width);
    for (int side = -1; side <= 1; side += 2) {
        memset(raw, 20, sizeof raw);
        memset(island_mask, 0, sizeof island_mask);
        for (int y = 30; y <= 105; ++y) {
            for (int x = 0; x < 188; ++x)
                if (y < 50 || y > 75) raw[y*188+x] = 220;
            if (y >= 50 && y <= 75) {
                island_mask[y] = 1;
                island_left[y] = side < 0 ? 119 : 0;
                island_right[y] = side < 0 ? 187 : 68;
            }
        }
        ep_reset();
        ep_set_context(&context);
        ep_column_opening_t openings[EP_MAX_OPENINGS];
        int count = ep_column_openings(raw, thresholds, side, 0, openings);
        assert(count == 1 && openings[0].inner == 49);
        assert(openings[0].center < 50);
        /* The white exit below this island is rejected even with cached width. */
        ep_previous_width = 50;
        ep_previous_side = side;
        count = ep_column_openings(raw, thresholds, side, 0, openings);
        assert(count <= 1);
        for (int i = 0; i < count; ++i) assert(openings[i].inner == 49);
        context.fresh_island = 0;
        ep_set_context(&context);
        assert(ep_column_openings(raw, thresholds, side, 0, openings) == 0);
        context.fresh_island = 1;
    }
    ep_previous_width = 30;
    ep_width_tick = 0xffffffefU;
    context.tick = ep_width_tick + EP_WIDTH_MAX_TICKS;
    ep_set_context(&context);
    assert(ep_previous_width == 30);
    ++context.tick;
    ep_set_context(&context);
    assert(!ep_previous_width);
    ep_previous_width = 30;
    ep_width_tick = context.tick;
    ep_width_odometry_valid = context.odometry_valid = 1;
    ep_width_distance = 1.0f;
    context.distance_m = 1.21f;
    ep_set_context(&context);
    assert(!ep_previous_width);
    /* Appending distant points cannot change fixed-distance control samples. */
    memset(&path, 0, sizeof path);
    for (int i = 0; i < 45; ++i) {
        path.points[i].x = (uint8_t)(94 + i);
        path.points[i].y = (uint8_t)(105 - 2*i);
    }
    path.count = 35;
    ep_control_points(&path);
    int error = path.error, preview = path.preview;
    path.count = 45;
    ep_control_points(&path);
    assert(error == path.error && preview == path.preview && preview > error);
    memset(&context, 0, sizeof context);
    ep_set_context(&context);
    ep_reset();
}

static void check_entry_island_clearance(void) {
    entry_context_t context = {0};
    context.active = context.fresh_island = 1;
    context.island_left = island_left;
    context.island_right = island_right;
    context.island_mask = island_mask;
    ep_set_context(&context);
    memset(island_mask, 0, sizeof island_mask);
    for (int y = 45; y <= 80; ++y) {
        island_mask[y] = 3;
        island_left[y] = 80;
        island_right[y] = 107;
    }
    path.count = 2;
    for (int side = -1; side <= 1; side += 2) {
        path.points[0].x = path.points[1].x = (uint8_t)(side < 0 ? 60 : 127);
        path.points[0].y = 95;
        path.points[1].y = 55;
        assert(ep_entry_clearance_ok(&path, side));
        /* The ends are above and below the bank. Check the interior of the
         * segment, where it would pass one pixel from the current island. */
        path.points[0].x = path.points[1].x = (uint8_t)(side < 0 ? 79 : 108);
        path.points[1].y = 30;
        assert(!ep_entry_clearance_ok(&path, side));
        /* The island corner is visible slightly below the end of the path. */
        path.points[0].y = 39;
        assert(!ep_entry_clearance_ok(&path, side));
    }
    context.fresh_island = 0;
    ep_set_context(&context);
    assert(!ep_entry_clearance_ok(&path, -1));
    memset(&context, 0, sizeof context);
    ep_set_context(&context);
}

static void check_exported_path(void) {
    assert(path.count >= 12);
    for (int i = 0; i < path.count; ++i) {
        const entry_point_t *point = &path.points[i];
        assert(ep_white(raw, thresholds, point->x, point->y));
        if (i) assert(ep_segment_white(raw, thresholds, path.points[i-1].x,
                                      path.points[i-1].y, point->x, point->y));
        if (i >= 2) {
            float ax = (float)path.points[i-1].x - path.points[i-2].x;
            float ay = (float)path.points[i-1].y - path.points[i-2].y;
            float bx = (float)point->x - path.points[i-1].x;
            float by = (float)point->y - path.points[i-1].y;
            /* Independent angle calculation on the exported coordinates. */
            double angle = atan2(fabs((double)ax * by - (double)ay * bx),
                                 (double)ax * bx + (double)ay * by);
            assert(angle <= 40.0 * 3.141592653589793 / 180.0);
        }
    }
}

static void check_smooth_forward_corridor(void) {
    uint8_t half_width[120];
    entry_context_t context = {0};
    context.active = 1;
    context.half_width = half_width;
    memset(half_width,55,sizeof half_width);
    ep_set_context(&context);
    for (int side = -1; side <= 1; side += 2) {
        memset(raw,20,sizeof raw);
        for (int y = 18; y <= 105; ++y) {
            int edge = 20 + (105-y)/2 + ((y/8)%2)*6;
            for (int x = edge; x < 188; ++x)
                raw[y*188 + (side < 0 ? x : 187-x)] = 220;
        }
        assert(ep_build_measured_corridor(raw,thresholds,side,94,&path));
        check_exported_path();
        assert(path.points[0].x == 94 && path.points[0].y == 105);
        assert(path.curve[2] == path.curve[0]);
        int changes = 0, previous_sign = 0;
        for (int i = 0; i <= 128; ++i) {
            float x,y,dx,dy,ddx,ddy;
            ep_curve_sample(path.curve,i/128.0f,&x,&y,&dx,&dy,&ddx,&ddy);
            float cross = dx*ddy-dy*ddx;
            int sign = cross > 0.01f ? 1 : (cross < -0.01f ? -1 : 0);
            if (sign && previous_sign && sign != previous_sign) ++changes;
            if (sign) previous_sign = sign;
        }
        assert(changes <= 1);
        assert(path.boundary_source == 5);
    }
    memset(raw,220,sizeof raw);
    assert(!ep_build_measured_corridor(raw,thresholds,-1,94,&path));
    assert(path.failure == 10 && !path.count);
    memset(&context,0,sizeof context);
    ep_set_context(&context);
}

int main(void) {
    entry_context_t context = {0};
    ep_set_context(&context);
    assert(ep_turn_allowed(1, 0, 1, 0));
    for (int sign = -1; sign <= 1; sign += 2) {
        float inside = 39.99f * 0.01745329252f;
        float outside = 40.01f * 0.01745329252f;
        assert(ep_turn_allowed(1, 0, cosf(inside), sign * sinf(inside)));
        assert(!ep_turn_allowed(1, 0, cosf(outside), sign * sinf(outside)));
    }
    assert(!ep_turn_allowed(0, 0, 1, 0));
    assert(!ep_turn_allowed(1, 0, -1, 0));
    assert(!ep_turn_allowed(1, 0, 1, 1));
    memset(raw, 220, sizeof raw);
    memset(thresholds, 100, sizeof thresholds);
    check_current_branch();
    check_entry_island_clearance();
    memset(raw,220,sizeof raw);
    const float curve[8] = {94,100,94,60,150,50,182,50};
    assert(ep_sample_curve(raw, thresholds, curve, &path));
    check_exported_path();
    for (int side = -1; side <= 1; side += 2) {
        memset(raw, 20, sizeof raw);
        for (int y = 18; y <= 105; ++y) {
            int center = 94 + side * ((105-y) / 3);
            for (int x = center - 40; x <= center + 40; ++x) raw[y*188+x] = 220;
        }
        assert(ep_build_measured_corridor(raw, thresholds, side, 94, &path));
        check_exported_path();
        assert((path.goal_x - 94) * side > 0);
        /* A full dark row between samples breaks the current white connection. */
        memset(raw + 104 * 188, 20, 188);
        assert(!ep_build_measured_corridor(raw, thresholds, side, 94, &path));
        assert(!path.count);
    }
    check_smooth_forward_corridor();
    ep_reset();
    assert(!ep_build(raw, thresholds, 0, 94, &path));
    puts("PASS: 40-degree boundary, mirrors, zero/reversed segments, rounded curve, smooth global corridor, current island, expiry and dark gap");
    return 0;
}
