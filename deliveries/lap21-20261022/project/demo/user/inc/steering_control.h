#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H
#include <stdint.h>
/* User measurements: geometric reference, NOT a calibrated servo/road model. */
#define STEERING_WHEELBASE_M 0.200f
#define STEERING_MAX_WHEEL_ANGLE_RAD 0.7853981634f
#define STEERING_MODEL_CALIBRATED 0
#define STEERING_TIGHT_CURVE_SPEED_SCALE (24.0f/45.0f) /* Keep tight turns at 24 counts/10ms. */
#define STEERING_STRAIGHT_GAIN 0.50f
#define STEERING_CURVE_GAIN    1.00f
#define STEERING_PREVIEW_BLEND 0.40f
#define STEERING_CURVE_DELTA   6
#define STEERING_CURVE_NEAR    6
#define STEERING_CURVE_PREVIEW 10
typedef struct {
    float command, error, gain;
    float curvature_reference, speed_scale;
    uint8_t curve, limited;
} steering_result_t;
steering_result_t steering_control(int near_error, int preview_error,
    uint8_t preview_valid, float middle, float direction, float low, float high);
#endif
