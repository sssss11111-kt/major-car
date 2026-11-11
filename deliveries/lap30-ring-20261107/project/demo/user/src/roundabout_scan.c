#include "roundabout_scan.h"
#include "img_process.h"

#if MT9V03X_W != CAR_W || MT9V03X_H != CAR_H
#error Camera dimensions must match the special-track scanner.
#endif

static special_track_state_t scan_state;
static special_track_config_t scan_config;
static vision_result_t primary;

void rb_scan_reset(void)
{
    special_track_init(&scan_state);
    special_track_config_default(&scan_config);
    scan_config.scan_bottom = IP_BODY_MASK_START_ROW - 2;
}

special_track_output_t rb_scan_frame(const uint8_t *raw, uint32_t tick, int force_full)
{
    special_track_output_t out;
    primary.valid = img_get_valid();
    primary.contrast = img_get_scene_contrast();
    primary.scanned_rows = img_get_usable_rows();
    primary.paired_rows = img_get_both_rows();
    special_track_update_rows(&scan_state, &scan_config, raw, img_get_row_thresholds(),
                              &primary, tick * 10u, force_full, &out);
    return out;
}

int rb_scan_allows_entry(const special_track_output_t *out, int expected_island_side)
{
    if (!out->full_scan_valid ||
        (out->diagnostic_flags & (SPECIAL_DIAG_CONFLICT | SPECIAL_DIAG_OVERFLOW)) ||
        out->frame_type == SPECIAL_CROSS_ISLAND || out->type == SPECIAL_CROSS_ISLAND)
        return 0;
    if (out->frame_type == SPECIAL_ROUNDABOUT)
        return out->island_side == expected_island_side;
    /* A clipped island may have no complete multi-channel topology.
       The primary R17 current-frame geometry and clearance must still pass. */
    return 1;
}

void rb_scan_observe(volatile rb_state_t *state, int island, int entry_allowed,
                     int cross_owned, int running)
{
    if (cross_owned && !state->fault && (state->phase == RB_IDLE || state->phase == RB_ALIGN)) {
        /* Preserve the next exit index. Only an uncommitted alignment can yield. */
        *state = (rb_state_t){.route = state->route};
        return;
    }
    int acquiring = state->phase == RB_IDLE || state->phase == RB_ALIGN;
    rb_geometry(state, img_get_entry_ready() && (!acquiring || entry_allowed),
                img_get_roundabout_corner_row(), img_get_exit_ready(), img_get_exit_error());
    rb_observe(state, island && (state->phase != RB_IDLE || entry_allowed),
               img_get_near_valid(), img_get_stripe_active(), img_get_road_recovered(), running);
}
