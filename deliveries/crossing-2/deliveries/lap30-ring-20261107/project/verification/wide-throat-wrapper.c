/* Replay harness for acceptance route B, extending the packaged wrapper with
 * the two accessors added for the near-corridor validity rule.
 *
 * All decisions come from production code (img_process.c, steering_control.c,
 * roundabout_scan.c, cross_drive.c, special_track*.c). The frame number is used
 * only as an odometry/time input, never as a routing decision. */
#include "img_process.h"
#include "steering_control.h"
#include "main.h"
#include "roundabout_control.h"
#include "roundabout_scan.h"
#include "cross_drive.h"
#include <stdint.h>
#include <string.h>

static uint8_t left[MT9V03X_H], mid[MT9V03X_H], right[MT9V03X_H];
static rb_state_t ring;
static cross_drive_t cross;
static special_track_output_t scan;
static steering_state_t steer;

__declspec(dllexport) void replay_ring_reset(void)
{
    img_process_reset();
    rb_scan_reset();
    memset(&ring, 0, sizeof ring);
    memset(&cross, 0, sizeof cross);
    memset(&scan, 0, sizeof scan);
    steering_reset(&steer, SERVO_MOTOR_MIDDLE, 0);
}

__declspec(dllexport) void replay_ring_step(uint8_t *raw, uint32_t tick, float *out)
{
    int guide, valid, island, allowed, phase_before;
    cross_drive_output_t cross_path;
    steering_result_t turn;
    const entry_path_t *path;
    float e;

    guide = rb_guiding(&ring) ? -RB_TURN_SIGN : 0;
    img_set_roundabout_guidance(guide);
    img_set_roundabout_entry(rb_guiding(&ring) ? ring.phase : 0);
    img_set_exit_search((uint8_t)rb_exit_search(&ring));
    /* Odometry input: a stationary capture supplies no motion, which is the
     * conservative choice (yaw never advances without measured travel). */
    img_set_entry_odometry(ring.distance_m, ring.yaw_deg, tick, 0);

    auxiliary_process(raw, MT9V03X_H, MT9V03X_W, 127, left, mid, right);
    valid = img_get_valid();
    if (rb_busy(&ring) && !img_get_near_valid()) valid = 0;

    island = img_get_roundabout_candidate();
    scan = rb_scan_frame(raw, tick, island || rb_busy(&ring) || cross.phase != CROSS_DRIVE_IDLE);
    allowed = rb_scan_allows_entry(&scan, RB_TURN_SIGN);
    cross_path = cross_drive_step(&cross, &scan, raw, img_get_row_thresholds(), tick, 1,
                                  ring.fault || (rb_busy(&ring) && ring.phase != RB_ALIGN),
                                  img_get_road_recovered());

    phase_before = ring.phase;
    rb_scan_observe(&ring, island, allowed && cross.phase == CROSS_DRIVE_IDLE,
                    cross_path.valid && !img_get_stripe_active(), 1);

    if (phase_before != ring.phase) {
        /* Same-frame recompute, as main.c does after a phase change. */
        guide = rb_guiding(&ring) ? -RB_TURN_SIGN : 0;
        img_set_roundabout_guidance(guide);
        img_set_roundabout_entry(rb_guiding(&ring) ? ring.phase : 0);
        img_set_exit_search((uint8_t)rb_exit_search(&ring));
        auxiliary_process(raw, MT9V03X_H, MT9V03X_W, 127, left, mid, right);
        valid = img_get_valid();
        if (!img_get_near_valid()) valid = 0;
    }

    if (ring.just_entered && !img_get_entry_path_safe()) {
        rb_reject_entry(&ring);
        valid = 0;
    }
    if (cross_path.active) valid = cross_path.valid && !img_get_stripe_active();

    path = img_get_entry_path();
    e = rb_plan_entry_error(ring.phase, guide, (float)img_get_error(),
                            (float)img_get_roundabout_entry_error(),
                            (float)img_get_preview_error(),
                            img_get_roundabout_preview_valid(), valid,
                            img_get_stripe_active());
    if (path->count) e = (float)img_get_error();
    if (cross_path.active) e = (float)cross_path.near_error;

    turn = steering_update(&steer, (int)e,
                           cross_path.active ? cross_path.preview_error : img_get_preview_error(),
                           cross_path.active ? valid : img_get_preview_valid(),
                           valid, 1, tick,
                           SERVO_MOTOR_MIDDLE, SERVO_DIR,
                           SERVO_MOTOR_R_MAX, SERVO_MOTOR_L_MAX);

    out[0] = turn.command;
    out[1] = (float)ring.phase;
    out[2] = (float)cross.phase;
    out[3] = (float)valid;
    out[4] = (float)img_get_error();
    out[5] = (float)img_get_preview_error();
    out[6] = (float)img_get_preview_valid();
    out[7] = (float)img_get_entry_status();
    out[8] = (float)island;
    out[9] = (float)img_get_roundabout_evidence();
    out[10] = (float)img_get_roundabout_corner_row();
    out[11] = (float)img_get_road_recovered();
    out[12] = (float)scan.type;
    out[13] = (float)scan.frame_type;
    out[14] = (float)scan.diagnostic_flags;
    out[15] = (float)img_get_both_rows();
    out[16] = (float)img_get_near_run();
    out[17] = (float)img_get_entry_bridge_guard()->active;
    out[18] = (float)ring.fault;
    out[19] = (float)path->count;
    out[20] = (float)img_get_preview_error();
    out[21] = (float)img_get_usable_rows();
    out[22] = (float)img_get_single_rows();
    out[23] = (float)turn.speed_scale;
}
