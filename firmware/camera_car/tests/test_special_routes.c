#include "cross_island.h"
#include "roundabout.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const special_fsm_config_t timing = {2, 180, 3000};
static const special_route_config_t route = {1, 1, PORT_A, PORT_C, ROUTE_CCW};

static special_route_observation_t observation(void)
{
    return (special_route_observation_t){
        .entry_port = PORT_A, .exit_port = PORT_UNKNOWN, .direction = ROUTE_CCW,
        .path_current = 1
    };
}

static void cross_sequence(void)
{
    cross_island_t fsm;
    special_route_observation_t o = observation();
    cross_island_begin(&fsm, 0);
    o.entry_port = PORT_B;
    cross_island_update(&fsm, &route, &timing, &o, 20);
    o.entry_port = PORT_A;
    cross_island_update(&fsm, &route, &timing, &o, 40);
    assert(fsm.phase == CROSS_APPROACH);
    cross_island_update(&fsm, &route, &timing, &o, 60);
    assert(fsm.phase == CROSS_ENTRY_ALIGN);
    o.entry_aligned = 1;
    cross_island_update(&fsm, &route, &timing, &o, 80);
    assert(fsm.phase == CROSS_ENTRY_ALIGN);
    cross_island_update(&fsm, &route, &timing, &o, 100);
    assert(fsm.phase == CROSS_ISLAND_FOLLOW);
    o.exit_port = PORT_D;
    o.exit_crossed = 1;
    cross_island_update(&fsm, &route, &timing, &o, 120);
    cross_island_update(&fsm, &route, &timing, &o, 140);
    assert(fsm.phase == CROSS_ISLAND_FOLLOW);
    o.exit_port = PORT_C;
    cross_island_update(&fsm, &route, &timing, &o, 160);
    cross_island_update(&fsm, &route, &timing, &o, 180);
    assert(fsm.phase == CROSS_EXIT_CAPTURE);
    o.exit_port = PORT_D;
    cross_island_update(&fsm, &route, &timing, &o, 200);
    assert(fsm.phase == CROSS_EXIT_CAPTURE);
    o.exit_port = PORT_C;
    cross_island_update(&fsm, &route, &timing, &o, 220);
    cross_island_update(&fsm, &route, &timing, &o, 240);
    assert(fsm.phase == CROSS_RECOVER);
    o.road_recovered = 1;
    cross_island_update(&fsm, &route, &timing, &o, 260);
    cross_island_update(&fsm, &route, &timing, &o, 280);
    assert(fsm.phase == CROSS_IDLE);
}

static void round_sequence(void)
{
    roundabout_t fsm;
    special_route_observation_t o = observation();
    o.entry_aligned = 1;
    roundabout_begin(&fsm, 0);
    o.entry_clearance_valid = 1;
    roundabout_update(&fsm, &route, &timing, &o, 20);
    roundabout_update(&fsm, &route, &timing, &o, 40);
    assert(fsm.phase == ROUNDABOUT_APPROACH); /* clearance alone cannot turn early */
    o.entry_progress_ready = 1;
    o.entry_clearance_valid = 0;
    roundabout_update(&fsm, &route, &timing, &o, 60);
    roundabout_update(&fsm, &route, &timing, &o, 80);
    assert(fsm.phase == ROUNDABOUT_APPROACH); /* close island cannot force a tight turn */
    o.entry_clearance_valid = 1;
    roundabout_update(&fsm, &route, &timing, &o, 100);
    o.path_current = 0;
    roundabout_update(&fsm, &route, &timing, &o, 120);
    o.path_current = 1;
    roundabout_update(&fsm, &route, &timing, &o, 140);
    assert(fsm.phase == ROUNDABOUT_APPROACH);
    roundabout_update(&fsm, &route, &timing, &o, 160);
    assert(fsm.phase == ROUNDABOUT_ENTRY);
    o.follow_established = 1;
    roundabout_update(&fsm, &route, &timing, &o, 180);
    roundabout_update(&fsm, &route, &timing, &o, 200);
    assert(fsm.phase == ROUNDABOUT_FOLLOW);
    o.exit_port = PORT_C;
    o.exit_crossed = 1;
    roundabout_update(&fsm, &route, &timing, &o, 220);
    roundabout_update(&fsm, &route, &timing, &o, 240);
    assert(fsm.phase == ROUNDABOUT_FOLLOW); /* correct port still needs exit progress */
    o.exit_search_ready = 1;
    o.exit_port = PORT_D;
    roundabout_update(&fsm, &route, &timing, &o, 260);
    roundabout_update(&fsm, &route, &timing, &o, 280);
    assert(fsm.phase == ROUNDABOUT_EXIT_SEARCH);
    roundabout_update(&fsm, &route, &timing, &o, 300);
    roundabout_update(&fsm, &route, &timing, &o, 320);
    assert(fsm.phase == ROUNDABOUT_EXIT_SEARCH);
    o.exit_port = PORT_C;
    roundabout_update(&fsm, &route, &timing, &o, 340);
    roundabout_update(&fsm, &route, &timing, &o, 360);
    assert(fsm.phase == ROUNDABOUT_EXIT_CAPTURE);
    o.exit_port = PORT_B;
    roundabout_update(&fsm, &route, &timing, &o, 380);
    assert(fsm.phase == ROUNDABOUT_EXIT_CAPTURE);
    o.exit_port = PORT_C;
    roundabout_update(&fsm, &route, &timing, &o, 400);
    roundabout_update(&fsm, &route, &timing, &o, 420);
    assert(fsm.phase == ROUNDABOUT_RECOVER);
    o.path_current = 0;
    o.road_recovered = 1;
    roundabout_update(&fsm, &route, &timing, &o, 440);
    roundabout_update(&fsm, &route, &timing, &o, 460);
    assert(fsm.phase == ROUNDABOUT_IDLE);
}

static void loss_and_wrap(void)
{
    roundabout_t round;
    cross_island_t cross;
    special_route_observation_t o = observation();
    uint32_t start = UINT32_MAX - 35u;
    roundabout_begin(&round, start);
    cross_island_begin(&cross, start);
    o.entry_aligned = o.entry_progress_ready = o.entry_clearance_valid = 1;
    roundabout_update(&round, &route, &timing, &o, start + 20);
    roundabout_update(&round, &route, &timing, &o, start + 40);
    cross_island_update(&cross, &route, &timing, &o, start + 20);
    cross_island_update(&cross, &route, &timing, &o, start + 40);
    assert(round.phase == ROUNDABOUT_ENTRY && cross.phase == CROSS_ENTRY_ALIGN);
    o.entry_clearance_valid = 0;
    roundabout_update(&round, &route, &timing, &o, start + 60);
    assert(round.phase == ROUNDABOUT_ENTRY);
    o.entry_clearance_valid = 1;
    roundabout_update(&round, &route, &timing, &o, start + 80);
    assert(round.phase == ROUNDABOUT_ENTRY);
    o.entry_clearance_valid = 0;
    o.path_current = 0;
    cross_island_update(&cross, &route, &timing, &o, start + 40 + timing.loss_timeout_ms);
    assert(cross.phase == CROSS_FAILSAFE);
    o.path_current = 1; /* a center path does not replace the lost inner boundary */
    roundabout_update(&round, &route, &timing, &o, start + 80 + timing.loss_timeout_ms);
    assert(round.phase == ROUNDABOUT_FAILSAFE);
}

static void planner_geometry(void)
{
    roundabout_t round = {.phase = ROUNDABOUT_ENTRY};
    cross_island_t cross = {.phase = CROSS_EXIT_CAPTURE};
    special_route_observation_t o = observation();
    special_plan_evidence_t e = {
        .entry = {3, {{90,100}, {92,90}, {96,80}}},
        .current = 1, .calibrated = 1, .lookahead_y = 95,
        .entry_reference_x = 90, .entry_inner_side = -1,
        .entry_inner_top = 80, .entry_inner_bottom = 100,
        .entry_min_clearance_px = 14, .entry_max_curvature_px_inv = 0.03f,
        .entry_max_join_offset_px = 4
    };
    special_plan_output_t out;
    int16_t center;
    assert(special_path_center(50, 138, 0, &center) && center == 94);
    assert(!special_path_center(50, -1, 0, &center));
    assert(special_path_center(50, -1, 88, &center) && center == 94);
    assert(!special_path_center(180, -1, 88, &center));

    for (int y = 80; y <= 100; ++y) e.entry_inner_x[y] = 70;
    o.entry_progress_ready = o.entry_clearance_valid = 1;
    roundabout_plan(&round, &route, &o, &e, &out);
    assert(out.target_valid && fabsf(out.target_error + 2.5f) < 0.01f);
    assert(out.curvature_px_inv > 0 && out.max_curvature_px_inv < 0.03f);

    e.entry_inner_x[95] = 88; /* close inner edge between sparse path points */
    roundabout_plan(&round, &route, &o, &e, &out);
    assert(!out.target_valid);
    e.entry_inner_x[95] = -1; /* no substitute from the previous frame */
    roundabout_plan(&round, &route, &o, &e, &out);
    assert(!out.target_valid);
    e.entry_inner_x[95] = 70;
    e.entry_max_curvature_px_inv = 0.001f;
    roundabout_plan(&round, &route, &o, &e, &out);
    assert(!out.target_valid);
    e.entry_max_curvature_px_inv = 0.03f;
    e.entry_reference_x = 104;
    roundabout_plan(&round, &route, &o, &e, &out);
    assert(!out.target_valid);
    e.entry_reference_x = CAR_W - 1 - 90;
    e.entry_inner_side = 1;
    for (unsigned i = 0; i < e.entry.count; ++i)
        e.entry.points[i].x = (int16_t)(CAR_W - 1 - e.entry.points[i].x);
    for (int y = 80; y <= 100; ++y) e.entry_inner_x[y] = CAR_W - 1 - 70;
    roundabout_plan(&round, &route, &o, &e, &out);
    assert(out.target_valid && out.curvature_px_inv < 0 && fabsf(out.target_error - 2.5f) < 0.01f);

    o.path_current = 0;
    roundabout_plan(&round, &route, &o, &e, &out);
    assert(!out.target_valid && out.target_error == 0);
    o.path_current = 1;
    special_route_config_t unavailable = route;
    unavailable.exit_port = PORT_UNKNOWN;
    roundabout_plan(&round, &unavailable, &o, &e, &out);
    assert(!out.target_valid);
    unavailable = route;
    unavailable.route_valid = 0;
    roundabout_plan(&round, &unavailable, &o, &e, &out);
    assert(!out.target_valid);
    e.exit = e.entry;
    o.exit_port = PORT_D;
    cross_island_plan(&cross, &route, &o, &e, &out);
    assert(!out.target_valid);
    o.exit_port = PORT_C;
    cross_island_plan(&cross, &route, &o, &e, &out);
    assert(out.target_valid);
    e.exit.count = SPECIAL_PATH_POINTS + 1;
    cross_island_plan(&cross, &route, &o, &e, &out);
    assert(!out.target_valid);
}

int main(void)
{
    cross_sequence();
    round_sequence();
    loss_and_wrap();
    planner_geometry();
    puts("special routes: PASS (independent phases, exit matching, loss, entry clearance and curvature)");
    return 0;
}
