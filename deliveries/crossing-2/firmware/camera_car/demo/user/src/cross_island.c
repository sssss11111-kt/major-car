/* Cross-island state machine and planner interface. GPL-3.0-or-later. */
#include "cross_island.h"

void cross_island_begin(cross_island_t *fsm, uint32_t now)
{
    *fsm = (cross_island_t){CROSS_APPROACH, {now, now, 0}};
}

void cross_island_update(cross_island_t *fsm, const special_route_config_t *route,
                         const special_fsm_config_t *cfg,
                         const special_route_observation_t *o, uint32_t now)
{
    if (fsm->phase == CROSS_IDLE || fsm->phase == CROSS_FAILSAFE) return;
    int path = o->path_current && o->direction == route->direction;
    int support = path;
    if (fsm->phase == CROSS_EXIT_CAPTURE) support = path && o->exit_port == route->exit_port;
    if (fsm->phase == CROSS_RECOVER) support = o->road_recovered;
    if (special_fsm_expired(&fsm->timer, cfg, support, now)) {
        fsm->phase = CROSS_FAILSAFE;
        return;
    }
    if (!special_route_valid(route)) {
        if (fsm->phase != CROSS_APPROACH) fsm->phase = CROSS_FAILSAFE;
        return;
    }

    switch (fsm->phase) {
    case CROSS_APPROACH:
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames, path && o->entry_port == route->entry_port))
            fsm->phase = CROSS_ENTRY_ALIGN;
        break;
    case CROSS_ENTRY_ALIGN:
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames, path && o->entry_aligned))
            fsm->phase = CROSS_ISLAND_FOLLOW;
        break;
    case CROSS_ISLAND_FOLLOW:
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames, path && o->exit_port == route->exit_port))
            fsm->phase = CROSS_EXIT_CAPTURE;
        break;
    case CROSS_EXIT_CAPTURE:
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames,
                               path && o->exit_crossed && o->exit_port == route->exit_port))
            fsm->phase = CROSS_RECOVER;
        break;
    case CROSS_RECOVER:
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames, o->road_recovered))
            fsm->phase = CROSS_IDLE;
        break;
    default:
        break;
    }
}

void cross_island_plan(const cross_island_t *fsm, const special_route_config_t *route,
                       const special_route_observation_t *o,
                       const special_plan_evidence_t *e, special_plan_output_t *out)
{
    *out = (special_plan_output_t){0};
    if (!special_route_valid(route) || !o->path_current || o->direction != route->direction ||
        !e->current || !e->calibrated) return;

    if (fsm->phase == CROSS_ENTRY_ALIGN && o->entry_port == route->entry_port)
        special_path_target(&e->entry, e->lookahead_y, out);
    if (fsm->phase == CROSS_ISLAND_FOLLOW)
        special_path_target(&e->follow, e->lookahead_y, out);
    if (fsm->phase == CROSS_EXIT_CAPTURE && o->exit_port == route->exit_port)
        special_path_target(&e->exit, e->lookahead_y, out);
}
