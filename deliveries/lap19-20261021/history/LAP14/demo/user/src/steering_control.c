#include "steering_control.h"
steering_result_t steering_control(int near_error, int preview_error,
    uint8_t preview_valid, float middle, float direction, float low, float high)
{
    steering_result_t out = {middle, (float)near_error, STEERING_STRAIGHT_GAIN, 0, 0};
    int delta = preview_error - near_error;
    if (delta < 0) delta = -delta;
    if (preview_valid && delta >= STEERING_CURVE_DELTA) {
        out.curve = 1;
        out.gain = STEERING_CURVE_GAIN;
        /* Do not reverse an established near target in an opposite bend. */
        if (near_error * preview_error >= 0 || (near_error >= -2 && near_error <= 2))
            out.error += STEERING_PREVIEW_BLEND * (float)(preview_error-near_error);
    }
    out.command = middle + out.error * out.gain * direction;
    if (out.command < low) { out.command = low; out.limited = 1; }
    if (out.command > high) { out.command = high; out.limited = 1; }
    return out;
}
