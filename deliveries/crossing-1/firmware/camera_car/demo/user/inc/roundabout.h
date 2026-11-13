/* Roundabout route phases. GPL-3.0-or-later. */
#ifndef ROUNDABOUT_H
#define ROUNDABOUT_H

#include "special_route.h"

typedef enum {
    ROUNDABOUT_IDLE, ROUNDABOUT_APPROACH, ROUNDABOUT_ENTRY, ROUNDABOUT_FOLLOW,
    ROUNDABOUT_EXIT_SEARCH, ROUNDABOUT_EXIT_CAPTURE, ROUNDABOUT_RECOVER, ROUNDABOUT_FAILSAFE
} roundabout_state_id_t;

typedef struct {
    roundabout_state_id_t phase;
    special_fsm_timer_t timer;
} roundabout_t;

void roundabout_begin(roundabout_t *fsm, uint32_t now);
void roundabout_update(roundabout_t *fsm, const special_route_config_t *route,
                       const special_fsm_config_t *cfg,
                       const special_route_observation_t *observation, uint32_t now);
void roundabout_plan(const roundabout_t *fsm, const special_route_config_t *route,
                     const special_route_observation_t *observation,
                     const special_plan_evidence_t *evidence, special_plan_output_t *out);

#endif
