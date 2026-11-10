/* Read-only audit exports around the unchanged production algorithm. */
#include "ordered-replay.c"

__declspec(dllexport) void replay_audit_state(float *out) {
    out[0] = replay_steering.near_filter;
    out[1] = replay_steering.preview_filter;
    out[2] = replay_steering.last.error;
    out[3] = replay_steering.last.severity;
    out[4] = replay_steering.last.speed_scale;
    out[5] = replay_steering.last.limited;
    out[6] = replay_steering.last.slew_limited;
    out[7] = replay_steering.preview_count;
    out[8] = s_ring_bend.single_count;
    out[9] = s_ring_bend.paired;
    out[10] = s_ring_bend.preview_count;
    out[11] = s_ring_bend.near_weight;
    out[12] = s_ring_bend.near_weight ?
        (float)s_ring_bend.near_sum / s_ring_bend.near_weight - IP_W / 2 : 0;
    out[13] = s_ring_entrance_seen;
    out[14] = s_approach_active ||
        (s_ring_miny && !s_ring_hold_age && s_ring_maxy < IP_TRACK_BOTTOM);
    out[15] = s_preview_valid;
    out[16] = s_preview_error;
    out[17] = s_ring_bend.usable;
    out[18] = s_entry_status;
    out[19] = s_bridge_guard.active;
}
