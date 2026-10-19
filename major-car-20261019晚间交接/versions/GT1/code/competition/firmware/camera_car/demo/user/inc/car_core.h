#ifndef CAR_CORE_H
#define CAR_CORE_H
#include <stdint.h>
#define CAR_W 188
#define CAR_H 120
typedef struct {
    uint8_t otsu, fixed_threshold, white_road;
    uint8_t first_row, last_row, row_step;
    uint8_t edge_run, max_row_jump, max_width_change; /* STEP-03, pixel units. */
} vision_config_t;
typedef struct {
    int16_t previous_center;
    uint16_t width[CAR_H];
    uint8_t trusted; /* Update history only after a valid frame. */
} vision_state_t;
typedef struct {
    int16_t left[CAR_H], right[CAR_H], center[CAR_H];
    uint8_t edge_flags[CAR_H]; /* bit 0 left, bit 1 right */
    uint16_t valid_rows, paired_rows, scanned_rows;
    uint8_t threshold, contrast, valid;
    uint8_t quality, rejected_rows; /* STEP-03: geometry support, not accuracy probability. */
    float error; /* positive: track center to vehicle right */
} vision_result_t;
typedef enum { CAR_IDLE, CAR_RUNNING, CAR_TEST, CAR_FAULT, CAR_SPEED_TEST, CAR_STEER_TEST, CAR_DRIVE_TEST, CAR_CALIBRATION, CAR_SUSTAIN, CAR_LOW_SPEED, CAR_GROUND_TEST } car_mode_t;
typedef enum { FAULT_NONE, FAULT_CAMERA, FAULT_FRAME, FAULT_LINE,
               FAULT_FOREGROUND, FAULT_ENCODER, FAULT_LINK } car_fault_t;
typedef struct {
    float speed_target, speed_kp, speed_ki, pwm_max;
    /* T2 STEP-01: tracking-only PI bias; zero preserves the 1500 legacy seed. */
    float tracking_pwm_bias[2];
    float low_kp,low_ki,low_ff_offset[2],low_ff_slope[2];
    uint8_t tracking_low_speed,tracking_ground;
    float steer_kp, steer_kd, steer_sign;
    float servo_center_us, servo_min_us, servo_max_us;
    uint32_t frame_timeout_ms, heartbeat_timeout_ms;
    /* STEP-02: 0 disables the corresponding optional trial guard. */
    uint32_t tracking_settle_ms, tracking_duration_ms, tracking_lease_ms;
    uint8_t simulated;
} control_config_t;
typedef struct {
    control_config_t cfg;
    car_mode_t mode;
    car_fault_t fault;
    uint32_t frame_ms, heartbeat_ms, test_until, motion_ms;
    uint8_t good_frames, bad_frames, test_motor;
    uint16_t stalled_ticks[2];
    float error, derivative, previous_error;
    float integral[2], duty[2], servo_us;
    float bench_target;
    uint8_t drive_start_ticks[2];
    uint8_t drive_lease_active;
    uint32_t drive_command_ms;
    uint32_t trial_id, trial_started_ms, trial_elapsed_ms;
    uint8_t trial_done; /* Only normal tracking expiry sets this; explicit stop/fault clears it. */
    int8_t drive_turn; /* -1 left, 0 straight, +1 right */
    uint8_t bench_started; /* bit 0 left feedback detected, bit 1 right */
    uint16_t calibration_duty;
    uint16_t sustain_duty,sustain_n,sustain_zeros,sustain_stall_ticks;
    int16_t sustain_min,sustain_max;
    int32_t sustain_sum;
    uint32_t sustain_id,sustain_started,sustain_hold_started,sustain_total_ms,sustain_hold_ms;
    uint8_t sustain_phase,sustain_result,sustain_kick_ticks;
    uint32_t ground_id,ground_started,ground_elapsed,ground_hold_started;
    uint16_t ground_duty;
    uint8_t ground_phase,ground_done,ground_closed;
    uint32_t low_id,low_started,low_elapsed;
    uint8_t low_done,low_confirmed,low_target,low_regulating;
    uint16_t low_n[2];
    int16_t low_min[2],low_max[2];
    int32_t low_sum[2];
    uint32_t low_error2[2],low_abs_error[2];
} control_state_t;
void vision_config_default(vision_config_t *cfg);
void vision_reset(vision_state_t *state);
void vision_process(vision_state_t *state, const vision_config_t *cfg,
                    const uint8_t *gray, uint8_t *binary, vision_result_t *out);
void control_init(control_state_t *state, const control_config_t *cfg);
void control_frame(control_state_t *state, const vision_result_t *frame, uint32_t now);
/* STEP-04: capture and completion are distinct timestamps. */
void control_frame_at(control_state_t *state,const vision_result_t *frame,uint32_t captured,uint32_t now);
int control_drive_long_test(control_state_t *state,uint32_t now);
void control_drive_keepalive(control_state_t *state,uint32_t now);
int control_drive_test(control_state_t *state, int turn, uint32_t now);
int control_steer_test(control_state_t *state, uint32_t now);
int control_start(control_state_t *state, uint32_t now);
int control_tracking_ready(const control_state_t *state, uint32_t now);
void control_stop(control_state_t *state);
void control_fault(control_state_t *state, car_fault_t fault);
int control_test(control_state_t *state, uint8_t motor, uint32_t now);
#define CAR_CALIBRATION_MAX_DUTY 7000u /* Diagnostic envelope; not a rated hardware limit. */
#define CAR_CALIBRATION_ENCODER_LIMIT 1500 /* Counts/10ms; calibration only, not rated motor RPM. */
int control_calibration_test(control_state_t *state,uint8_t motor,uint16_t duty,uint32_t now);
int control_sustain_test(control_state_t *state,uint8_t motor,uint16_t duty,uint32_t now);
int control_low_test(control_state_t *state,uint8_t motor,uint8_t target,uint32_t now);
int control_ground_closed_test(control_state_t *state,uint32_t now);
int control_ground_test(control_state_t *state,uint16_t duty,uint32_t now);
#define CAR_GROUND_MAX_MS 2000u
#define CAR_GROUND_HOLD_MS 1500u
#define CAR_GROUND_ENCODER_LIMIT 120
#define CAR_LOW_DURATION_MS 6000u
#define CAR_LOW_STATS_MS 3000u
#define CAR_LOW_CAP 1500u
#define CAR_LOW_KICK 3000u
#define CAR_LOW_LEASE_MS 500u
#define CAR_SUSTAIN_KICK_DUTY 3000u
#define CAR_SUSTAIN_KICK_MIN_MS 200u
#define CAR_SUSTAIN_KICK_MAX_MS 300u
#define CAR_SUSTAIN_HOLD_MS 3000u
#define CAR_SUSTAIN_STATS_AFTER_MS 1000u
#define CAR_SUSTAIN_LEASE_MS 500u
int control_speed_test_right(control_state_t *state, uint32_t now);
int control_speed_test(control_state_t *state, uint8_t motor, uint32_t now);
void control_tick(control_state_t *state, uint32_t now, int16_t left, int16_t right);
#endif
