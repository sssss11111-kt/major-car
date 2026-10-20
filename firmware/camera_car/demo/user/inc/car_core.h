#ifndef CAR_CORE_H
#define CAR_CORE_H
#include <stdint.h>
#define CAR_W 188
#define CAR_H 120
typedef struct {
    uint8_t otsu, fixed_threshold, white_road;
    uint8_t first_row, last_row, row_step;
} vision_config_t;
typedef struct {
    int16_t previous_center;
    uint16_t width[CAR_H];
    uint8_t width_age[CAR_H]; /* 0 measured this frame; >8 unusable */
    uint8_t history_valid, polarity;
} vision_state_t;
typedef struct {
    int16_t left[CAR_H], right[CAR_H], center[CAR_H];
    uint8_t edge_flags[CAR_H]; /* bit 0 left, bit 1 right */
    uint16_t valid_rows, paired_rows, scanned_rows;
    uint8_t threshold, contrast, valid;
    uint8_t row_threshold[CAR_H]; /* automatic mode uses these, not threshold */
    uint16_t inferred_rows; /* one real edge, never counted as paired */
    float error; /* positive: track center to vehicle right */
} vision_result_t;
typedef enum { CAR_IDLE, CAR_RUNNING, CAR_TEST, CAR_FAULT } car_mode_t;
typedef enum { FAULT_NONE, FAULT_CAMERA, FAULT_FRAME, FAULT_LINE,
               FAULT_FOREGROUND, FAULT_ENCODER } car_fault_t;
typedef struct {
    float speed_target, speed_kp, speed_ki, pwm_max;
    float steer_kp, steer_kd, steer_sign;
    float servo_center_us, servo_min_us, servo_max_us;
    uint32_t frame_timeout_ms, heartbeat_timeout_ms;
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
} control_state_t;
void vision_config_default(vision_config_t *cfg);
void vision_reset(vision_state_t *state);
void vision_process(vision_state_t *state, const vision_config_t *cfg,
                    const uint8_t *gray, uint8_t *binary, vision_result_t *out);
void control_init(control_state_t *state, const control_config_t *cfg);
void control_frame(control_state_t *state, const vision_result_t *frame, uint32_t now);
int control_start(control_state_t *state, uint32_t now);
void control_stop(control_state_t *state);
void control_fault(control_state_t *state, car_fault_t fault);
int control_test(control_state_t *state, uint8_t motor, uint32_t now);
void control_tick(control_state_t *state, uint32_t now, int16_t left, int16_t right);
#endif
