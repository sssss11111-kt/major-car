#ifndef ROUNDABOUT_SCAN_H
#define ROUNDABOUT_SCAN_H

#include "special_track.h"
#include "roundabout_control.h"

/* Called once per new camera frame after the primary R17 image processing. */
void rb_scan_reset(void);
int rb_scan_pending_frames(void);
special_track_output_t rb_scan_frame(const uint8_t *raw, uint32_t tick, int force_full);
int rb_scan_allows_entry(const special_track_output_t *out, int expected_island_side);
void rb_scan_observe(volatile rb_state_t *state, int island, int entry_allowed,
                     int cross_owned, int running);

#endif
