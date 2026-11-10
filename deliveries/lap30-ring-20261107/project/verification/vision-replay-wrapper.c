#include "img_process.h"
#include "steering_control.h"
#include "main.h"
#include "roundabout_control.h"
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
__declspec(dllexport) int replay_evidence(void) {return img_get_roundabout_evidence();}
__declspec(dllexport) int replay_corner_row(void) {return img_get_roundabout_corner_row();}
__declspec(dllexport) int replay_entry_error(void) {return img_get_roundabout_entry_error();}
__declspec(dllexport) void replay_guide(int side) {img_set_roundabout_guidance(side);}

static rb_state_t integrated_ring;
static uint8_t replay_odometry_enabled;
__declspec(dllexport) void replay_ring_reset(void) {replay_reset();integrated_ring=(rb_state_t){0};replay_odometry_enabled=0;}
__declspec(dllexport) void replay_ring_step(uint8_t *raw,uint32_t tick,float *out) {
 int entry_was_preparing = img_get_entry_status() == 0;
 int guide=rb_guiding(&integrated_ring) ? -RB_TURN_SIGN:0;
 img_set_roundabout_guidance(guide);
 img_set_roundabout_entry(rb_guiding(&integrated_ring) ? integrated_ring.phase : 0);
 img_set_exit_search((uint8_t)rb_exit_search(&integrated_ring));
 img_set_entry_odometry(integrated_ring.distance_m,integrated_ring.yaw_deg,tick,replay_odometry_enabled);
 auxiliary_process(raw,120,188,127,left,mid,right);
 int valid=img_get_valid();
 if(rb_busy(&integrated_ring) && !img_get_near_valid())valid=0;
 int old_phase=integrated_ring.phase;
 rb_geometry(&integrated_ring,img_get_entry_ready(),img_get_roundabout_corner_row(),img_get_exit_ready(),img_get_exit_error());
 rb_observe(&integrated_ring,img_get_roundabout_candidate(),img_get_near_valid(),img_get_stripe_active(),img_get_road_recovered(),1);
 if(old_phase!=integrated_ring.phase) {
  /* Mirror production: any phase transition recomputes the same frame using
   * the new planner mode, without counting the frame twice in the FSM. */
  guide = rb_guiding(&integrated_ring) ? -RB_TURN_SIGN:0;
  img_set_roundabout_guidance(guide);
  img_set_roundabout_entry(rb_guiding(&integrated_ring) ? integrated_ring.phase:0);
  img_set_exit_search((uint8_t)rb_exit_search(&integrated_ring));
  auxiliary_process(raw,120,188,127,left,mid,right);
  valid=img_get_valid();
  if(!img_get_near_valid())valid=0;
 }
 if(integrated_ring.just_entered && !img_get_entry_path_safe()) {
  rb_reject_entry(&integrated_ring);
  valid=0;
 }
 float e=rb_plan_entry_error(integrated_ring.phase,guide,img_get_error(),
     img_get_roundabout_entry_error(),img_get_preview_error(),
     img_get_roundabout_preview_valid(),valid,img_get_stripe_active());
 if(img_get_entry_path()->count)e=img_get_error();
 int metric_control=ep_metric_error(img_get_entry_path(),&e);
 if(integrated_ring.just_entered)steering_blend_entry(&replay_steering,e);
 else if(entry_was_preparing && img_get_entry_path()->count)
  steering_prime_entry(&replay_steering,e);
 steering_result_t r=steering_update(&replay_steering,(int)e,img_get_preview_error(),!metric_control && img_get_preview_valid(),valid,1,tick,
     SERVO_MOTOR_MIDDLE,SERVO_DIR,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
 out[0]=r.command;out[1]=(float)integrated_ring.phase;out[2]=e;
 out[3]=(float)valid;out[4]=(float)img_get_error();
}

__declspec(dllexport) int replay_road_recovered(void) {return img_get_road_recovered();}

__declspec(dllexport) void replay_ring_tick(int left,int right) {replay_odometry_enabled=1;rb_tick(&integrated_ring,left,right,1);}
__declspec(dllexport) int replay_ring_phase(void) {return integrated_ring.phase;}
/* Host-only scenario initialization. Never exported by firmware. */
__declspec(dllexport) void replay_ring_scenario(int phase,float yaw) {
 integrated_ring.phase=(uint8_t)phase;integrated_ring.yaw_deg=yaw;
}
__declspec(dllexport) void replay_gate_facts(int *out) {
 out[0]=img_get_entry_ready();out[1]=img_get_roundabout_corner_row();
 out[2]=img_get_entry_path()->failure;out[3]=img_get_entry_path()->error;
 out[4]=img_get_exit_ready();out[5]=integrated_ring.commit_confirm;
 out[6]=img_get_exit_error();out[7]=img_get_entry_path()->boundary_source;
}

__declspec(dllexport) int replay_entry_path(uint8_t *out) {
 const entry_path_t *path=img_get_entry_path();
 for(int i=0;i<path->count;++i) {
  const entry_point_t *point=&path->points[i];
  out[i*7]=point->x;out[i*7+1]=point->y;
  out[i*7+2]=point->left_x;out[i*7+3]=point->left_y;
  out[i*7+4]=point->right_x;out[i*7+5]=point->right_y;out[i*7+6]=point->measured;
 }
 return path->count;
}
__declspec(dllexport) int replay_entry_status(void) { return img_get_entry_status(); }
__declspec(dllexport) void replay_entry_mode(int enabled) {img_set_roundabout_entry((uint8_t)enabled);}

__declspec(dllexport) void replay_ring_odometry(float distance_m,float yaw_deg) {
 integrated_ring.distance_m=distance_m;integrated_ring.yaw_deg=yaw_deg;
 replay_odometry_enabled=1;
}
__declspec(dllexport) int replay_bridge_active(void) {return img_get_entry_bridge_guard()->active;}
