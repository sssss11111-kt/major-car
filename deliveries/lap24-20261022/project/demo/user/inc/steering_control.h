#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H
#include <stdint.h>
#define STEERING_WHEELBASE_M 0.200f
#define STEERING_MAX_WHEEL_ANGLE_RAD 0.7853981634f
#define STEERING_MODEL_CALIBRATED 0
#define STEERING_TIGHT_CURVE_SPEED_SCALE (54.0f/100.0f)
#define STEERING_STRAIGHT_GAIN 0.50f
#define STEERING_LINEAR_GAIN STEERING_STRAIGHT_GAIN
#define STEERING_PREVIEW_BLEND 0.40f
#define STEERING_PREVIEW_DELTA_MAX 8.0f
#define STEERING_PREVIEW_AUTHORITY_PX 8.0f
/* Provisional inner clearance, not a measured mechanical safe range. */
#define STEERING_SOFT_LIMIT_MARGIN 3.0f
/* Command points/s; not measured front-wheel angular velocity. */
#define STEERING_SLEW_OUTWARD 110.0f
#define STEERING_SLEW_RETURN 180.0f
#define STEERING_SPEED_RECOVERY_PER_S 0.8f
typedef struct {
    float command,error,gain,curvature_reference,speed_scale,severity;
    uint8_t curve,limited,bend_level,slew_limited;
} steering_result_t;
typedef struct {
    float near_filter,preview_filter,near_history[2],preview_history[2];
    uint32_t last_tick;
    uint8_t initialized,preview_count,bend_level;
    steering_result_t last;
} steering_state_t;
steering_result_t steering_control(int near,int preview,uint8_t pv,
    float middle,float direction,float low,float high);
void steering_reset(steering_state_t *s,float middle,uint32_t tick);
steering_result_t steering_update(steering_state_t *s,int near,int preview,uint8_t pv,
    uint8_t valid,uint8_t active,uint32_t tick,float middle,float direction,float low,float high);
#endif
