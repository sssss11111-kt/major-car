#include "roundabout_scan.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    static uint8_t raw[CAR_H * CAR_W], thresholds[CAR_H];
    special_track_state_t state;
    special_track_config_t cfg;
    special_track_output_t out;
    vision_result_t normal = {.valid = 1, .contrast = 120, .scanned_rows = 12, .paired_rows = 12};
    special_track_init(&state);
    special_track_config_default(&cfg);
    for (int y = 0; y < CAR_H; ++y) {
        thresholds[y] = y < 80 ? 90 : 210;
        for (int x = 0; x < CAR_W; ++x) {
            int road = x >= 28 && x <= 159 && (y < 56 || y > 87 || x < 83 || x > 104);
            raw[y * CAR_W + x] = (uint8_t)(thresholds[y] + (road ? 30 : -30));
        }
    }
    for (uint32_t now = 0; now <= 40; now += 20)
        special_track_update_rows(&state, &cfg, raw, thresholds, &normal, now, 1, &out);
    assert(out.full_scan_valid && out.type == SPECIAL_CROSS_ISLAND);
    assert(!rb_scan_allows_entry(&out, 1));

    out = (special_track_output_t){.full_scan_valid = 1, .type = SPECIAL_UNKNOWN};
    assert(rb_scan_allows_entry(&out, 1));
    out.diagnostic_flags = SPECIAL_DIAG_CONFLICT;
    assert(!rb_scan_allows_entry(&out, 1));
    out.diagnostic_flags = SPECIAL_DIAG_OVERFLOW;
    assert(!rb_scan_allows_entry(&out, 1));
    out.diagnostic_flags = 0;
    out.frame_type = SPECIAL_ROUNDABOUT;
    out.island_side = -1;
    assert(!rb_scan_allows_entry(&out, 1));
    assert(rb_scan_allows_entry(&out, -1));
    out.full_scan_valid = 0;
    assert(!rb_scan_allows_entry(&out, -1));
    puts("PASS: row-adaptive scanner, cross-island and overflow rejection, mirrored ring gating");
    return 0;
}
