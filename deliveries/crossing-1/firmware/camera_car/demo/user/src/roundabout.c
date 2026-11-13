/* Roundabout state machine and planner interface. GPL-3.0-or-later. */
#include "roundabout.h"

void roundabout_begin(roundabout_t *fsm, uint32_t now)
{
    *fsm = (roundabout_t){ROUNDABOUT_APPROACH, {now, now, 0}};
}

void roundabout_update(roundabout_t *fsm, const special_route_config_t *route,
                       const special_fsm_config_t *cfg,
                       const special_route_observation_t *o, uint32_t now)
{
    if (fsm->phase == ROUNDABOUT_IDLE || fsm->phase == ROUNDABOUT_FAILSAFE) return;
    int path = o->path_current && o->direction == route->direction;
    int support = path;
    if (fsm->phase == ROUNDABOUT_ENTRY) support = path && o->entry_clearance_valid;
    if (fsm->phase == ROUNDABOUT_EXIT_CAPTURE) support = path && o->exit_port == route->exit_port;
    if (fsm->phase == ROUNDABOUT_RECOVER) support = o->road_recovered;
    if (special_fsm_expired(&fsm->timer, cfg, support, now)) {
        fsm->phase = ROUNDABOUT_FAILSAFE;
        return;
    }
    if (!special_route_valid(route)) {
        if (fsm->phase != ROUNDABOUT_APPROACH) fsm->phase = ROUNDABOUT_FAILSAFE;
        return;
    }

    switch (fsm->phase) {
    case ROUNDABOUT_APPROACH:
        /* A visible island alone must not initiate the turn. */
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames,
                               path && o->entry_port == route->entry_port && o->entry_aligned &&
                               o->entry_progress_ready && o->entry_clearance_valid))
            fsm->phase = ROUNDABOUT_ENTRY;
        break;
    case ROUNDABOUT_ENTRY:
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames,
                               path && o->entry_clearance_valid && o->follow_established))
            fsm->phase = ROUNDABOUT_FOLLOW;
        break;
    case ROUNDABOUT_FOLLOW:
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames, path && o->exit_search_ready))
            fsm->phase = ROUNDABOUT_EXIT_SEARCH;
        break;
    case ROUNDABOUT_EXIT_SEARCH:
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames, path && o->exit_port == route->exit_port))
            fsm->phase = ROUNDABOUT_EXIT_CAPTURE;
        break;
    case ROUNDABOUT_EXIT_CAPTURE:
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames,
                               path && o->exit_crossed && o->exit_port == route->exit_port))
            fsm->phase = ROUNDABOUT_RECOVER;
        break;
    case ROUNDABOUT_RECOVER:
        if (special_fsm_stable(&fsm->timer, cfg->stable_frames, o->road_recovered))
            fsm->phase = ROUNDABOUT_IDLE;
        break;
    default:
        break;
    }
}

void roundabout_plan(const roundabout_t *fsm, const special_route_config_t *route,
                     const special_route_observation_t *o,
                     const special_plan_evidence_t *e, special_plan_output_t *out)
{
    *out = (special_plan_output_t){0};
    if (!special_route_valid(route) || !o->path_current || o->direction != route->direction ||
        !e->current || !e->calibrated) return;

    if (fsm->phase == ROUNDABOUT_ENTRY && o->entry_progress_ready && o->entry_clearance_valid)
        special_ring_entry_target(e, out);
    if (fsm->phase == ROUNDABOUT_FOLLOW || fsm->phase == ROUNDABOUT_EXIT_SEARCH)
        special_path_target(&e->follow, e->lookahead_y, out);
    if (fsm->phase == ROUNDABOUT_EXIT_CAPTURE && o->exit_port == route->exit_port)
        special_path_target(&e->exit, e->lookahead_y, out);
}
