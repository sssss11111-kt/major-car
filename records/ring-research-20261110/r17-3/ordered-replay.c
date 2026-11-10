#ifdef REPLAY_CURRENT
#define EP_FIT_DIAGNOSTICS
static float trace[44*5+3];
static void ep_fit_trace(int rows,int observations,int near,const float *target,const float *weight,const int *left,const int *right) {
 trace[0]=(float)rows; trace[1]=(float)observations; trace[2]=(float)near;
 for(int i=0;i<rows;++i){trace[3+5*i]=(float)(105-2*i);trace[4+5*i]=target[i];trace[5+5*i]=weight[i];trace[6+5*i]=(float)left[i];trace[7+5*i]=(float)right[i];}
}
#endif
/* Replay adapter includes the production vision and steering translation units. */
#include "img_process.c"
#include "steering_control.c"
#include "vision-replay-wrapper.c"
__declspec(dllexport) void replay_thresholds(uint8_t *out) {
    memcpy(out, s_row_thr, 120);
}
__declspec(dllexport) void replay_path_meta(int *out) {
    out[0] = s_entry_path.failure;
    out[1] = s_near_valid;
    out[2] = s_entry_path.boundary_source;
}

#ifdef REPLAY_CURRENT
__declspec(dllexport) void replay_current_island(uint8_t *out) {
    memcpy(out, s_ring_l, 120);
    memcpy(out+120, s_ring_r, 120);
    memcpy(out+240, ep_context.active ? s_entry_island_mask : s_ring_mask, 120);
    out[360] = ep_context.fresh_island;
}
__declspec(dllexport) void replay_planner_state(int *out) {
    out[0] = ep_previous_width;
    out[1] = s_ring_hold_age;
    out[2] = s_ring_miny;
    out[3] = s_preview_error;
    out[4] = s_entry_path.goal_x;
    out[5] = s_entry_path.goal_y;
}

#endif

#ifdef REPLAY_CURRENT
__declspec(dllexport) void replay_forward_trace(uint8_t *raw,float *out) {
 entry_path_t result;
 memset(trace,0,sizeof trace);
 ep_build_measured_corridor(raw,s_row_thr,s_ring_guide,94,&result);
 out[0]=result.failure;out[1]=result.support;out[2]=result.goal_y;
 memcpy(out+3,trace,sizeof trace);
}
#endif

#ifdef REPLAY_CURRENT
__declspec(dllexport) void replay_curve_control(float *out) {
 memcpy(out,s_entry_path.curve,sizeof s_entry_path.curve);
}
#endif

#ifdef REPLAY_CURRENT
__declspec(dllexport) void replay_exit_debug(int *out) {
 out[0]=s_exit_ready;out[1]=s_exit_path.count;out[2]=s_exit_path.goal_y;
 out[3]=s_exit_path.failure;out[4]=s_exit_path.preview;out[5]=s_exit_separated;
 out[6]=s_ring_evidence;out[7]=s_entry_path.preview;
 int paired=0;for(int i=0;i<s_exit_path.count;++i)if(s_exit_path.points[i].y<=75 && s_exit_path.points[i].measured==3)++paired;
 out[8]=paired;
}
#endif

#ifdef REPLAY_CURRENT
__declspec(dllexport) void replay_local_debug(int *out) {
 out[0]=s_ring_bend.near_count;out[1]=s_ring_bend.near_chain;out[2]=s_ring_bend.near_bottom;
 out[3]=s_ring_miny;out[4]=s_ring_maxy;out[5]=s_ring_hold_age;out[6]=s_scene_ok;
 out[7]=s_approach_active;out[8]=s_ring_local_confirm;
}
#endif

#ifdef REPLAY_CURRENT
__declspec(dllexport) void replay_local_rows(uint8_t *out) {
 memcpy(out,s_ring_bend.left,120);memcpy(out+120,s_ring_bend.center,120);memcpy(out+240,s_ring_bend.right,120);
}
#endif
