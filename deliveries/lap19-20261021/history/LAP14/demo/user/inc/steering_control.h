#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H
#include <stdint.h>
#define STEERING_STRAIGHT_GAIN 0.50f
#define STEERING_CURVE_GAIN    0.65f
#define STEERING_PREVIEW_BLEND 0.25f
#define STEERING_CURVE_DELTA   8
typedef struct {
    float command, error, gain;
    uint8_t curve, limited;
} steering_result_t;
steering_result_t steering_control(int near_error, int preview_error,
    uint8_t preview_valid, float middle, float direction, float low, float high);
#endif
