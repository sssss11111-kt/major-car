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
typedef enum { CAR_IDLE, CAR_RUNNING, CAR_TEST, CAR_FAULT, CAR_SPEED_TEST, CAR_STEER_TEST, CAR_DRIVE_TEST } car_mode_t;
typedef enum { FAULT_NONE, FAULT_CAMERA, FAULT_FRAME, FAULT_LINE,
               FAULT_FOREGROUND, FAULT_ENCODER, FAULT_LINK } car_fault_t;
typedef struct {
    float speed_target, speed_kp, speed_ki, pwm_max;
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
    int8_t drive_turn; /* -1 left, 0 straight, +1 right */
    uint8_t bench_started; /* bit 0 left feedback detected, bit 1 right */
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
void control_stop(control_state_t *state);
void control_fault(control_state_t *state, car_fault_t fault);
int control_test(control_state_t *state, uint8_t motor, uint32_t now);
int control_speed_test_right(control_state_t *state, uint32_t now);
int control_speed_test(control_state_t *state, uint8_t motor, uint32_t now);
void control_tick(control_state_t *state, uint32_t now, int16_t left, int16_t right);
#endif
