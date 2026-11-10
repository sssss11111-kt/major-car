/* Cross-island route phases. GPL-3.0-or-later. */
#ifndef CROSS_ISLAND_H
#define CROSS_ISLAND_H

#include "special_route.h"

typedef enum {
    CROSS_IDLE, CROSS_APPROACH, CROSS_ENTRY_ALIGN, CROSS_ISLAND_FOLLOW,
    CROSS_EXIT_CAPTURE, CROSS_RECOVER, CROSS_FAILSAFE
} cross_island_state_id_t;

typedef struct {
    cross_island_state_id_t phase;
    special_fsm_timer_t timer;
} cross_island_t;

void cross_island_begin(cross_island_t *fsm, uint32_t now);
void cross_island_update(cross_island_t *fsm, const special_route_config_t *route,
                         const special_fsm_config_t *cfg,
                         const special_route_observation_t *observation, uint32_t now);
void cross_island_plan(const cross_island_t *fsm, const special_route_config_t *route,
                       const special_route_observation_t *observation,
                       const special_plan_evidence_t *evidence, special_plan_output_t *out);

#endif
