#include "img_process.h"
#include "steering_control.h"
#include "main.h"
#include <stdint.h>
static uint8_t left[120],mid[120],right[120];
static steering_state_t replay_steering;
__declspec(dllexport) void replay_reset(void) {
    img_process_reset(); steering_reset(&replay_steering,SERVO_MOTOR_MIDDLE,0);
}
__declspec(dllexport) int replay_stripe(void) { return img_get_stripe_active(); }
__declspec(dllexport) void replay_turn(float *out)
{
    steering_result_t r=steering_control(img_get_error(),img_get_preview_error(),
        img_get_preview_valid(),SERVO_MOTOR_MIDDLE,SERVO_DIR,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
    out[0]=r.command;out[1]=r.gain;out[2]=r.curve;
    out[3]=img_get_preview_error();out[4]=img_get_preview_valid();
}
/* Same stateful steering owner as production main; no transition reset. */
__declspec(dllexport) void replay_control_step(uint32_t tick,float *out)
{
    steering_result_t r=steering_update(&replay_steering,img_get_error(),img_get_preview_error(),
        img_get_preview_valid(),img_get_valid(),1,tick,
        SERVO_MOTOR_MIDDLE,SERVO_DIR,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
    out[0]=r.command; out[1]=r.speed_scale; out[2]=r.severity;
    out[3]=replay_steering.preview_count; out[4]=replay_steering.initialized;
}
__declspec(dllexport) void replay_lines(uint8_t *lines)
{
    for (int y=0;y<120;++y) {
        lines[y]=left[y];lines[120+y]=mid[y];lines[240+y]=right[y];
    }
}
__declspec(dllexport) int replay_frame(uint8_t *raw, int *result)
{
    auxiliary_process(raw,120,188,127,left,mid,right);
    result[0]=img_get_valid();result[1]=img_get_error();
    result[2]=img_get_usable_rows();result[3]=img_get_both_rows();
    return result[0] && result[3]>=3 && !img_get_stripe_active() && !img_get_roundabout_candidate();
}

__declspec(dllexport) int replay_island(void) {return img_get_roundabout_candidate();}
__declspec(dllexport) void replay_guide(int side) {img_set_roundabout_guidance(side);}
