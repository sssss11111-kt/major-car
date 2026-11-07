/* Test and visualization adapter. Includes the same production translation units. */
#include "img_process.c"
#include "steering_control.c"
#include "vision-replay-wrapper.c"
__declspec(dllexport) void replay_thresholds(uint8_t *out) { memcpy(out,s_row_thr,120); }
__declspec(dllexport) void replay_entry_meta(int *out) {
 out[0]=s_entry_path.count;out[1]=s_entry_path.goal_x;out[2]=s_entry_path.goal_y;
 out[3]=s_entry_path.inferred;out[4]=s_entry_path.failure;out[5]=s_entry_status;
 out[6]=s_entry_locked;out[7]=s_valid;out[8]=s_near_valid;out[9]=ep_width_age;
}
__declspec(dllexport) int direct_path(uint8_t *raw,int side,uint8_t *out,int *meta) {
 img_set_roundabout_guidance(0);img_set_roundabout_entry(0);
 auxiliary_process(raw,120,188,127,left,mid,right);
 ep_reset();
 int result=ep_build(raw,s_row_thr,side,94,&s_entry_path);
 int n=replay_entry_path(out);replay_entry_meta(meta);
 return result ? n : 0;
}

__declspec(dllexport) void replay_curve(float *out) {memcpy(out,s_entry_path.curve,sizeof s_entry_path.curve);}

__declspec(dllexport) float replay_estimated_width(void) {return s_entry_path.estimated_width;}

__declspec(dllexport) int replay_boundary_source(void) {return s_entry_path.boundary_source;}
__declspec(dllexport) int replay_boundary_support(void) {return s_entry_path.support;}

__declspec(dllexport) void replay_set_odometry(float distance,float yaw,uint32_t tick,int valid) {
 img_set_entry_odometry(distance,yaw,tick,(uint8_t)valid);
}


__declspec(dllexport) void replay_cache(float *out) {
 out[0]=ep_previous_width;out[1]=ep_width_reference_row;out[2]=ep_previous_goal_y;
 out[3]=ep_previous_side;out[4]=ep_width_age;
 for(int i=0;i<3;++i){out[5+i]=ep_previous_outer[i];out[8+i]=ep_previous_inner[i];}
}
__declspec(dllexport) int replay_binary_plan(uint8_t *raw,uint8_t *thresholds,int side,int reset) {
 if(reset)ep_reset();
 return ep_build(raw,thresholds,side,94,&s_entry_path);
}

__declspec(dllexport) void replay_approach(uint8_t *side,uint8_t *edge) {memcpy(side,s_approach_side,120);memcpy(edge,s_approach_edge,120);}

__declspec(dllexport) void replay_approach_model(float *out) { out[0]=s_approach_width_side;out[1]=s_approach_width_age;out[2]=s_approach_width_bottom;out[3]=s_approach_width_slope;out[4]=s_odometry_tick-s_approach_width_tick; }
