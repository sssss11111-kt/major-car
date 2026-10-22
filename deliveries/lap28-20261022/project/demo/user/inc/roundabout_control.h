#ifndef ROUNDABOUT_CONTROL_H
#define ROUNDABOUT_CONTROL_H
#include <stdint.h>
/* Provisional user dimension; both wheel scale and track need field calibration. */
#define RB_REAR_TRACK_M 0.155f
#define RB_METERS_PER_COUNT 0.00004620f
#define RB_TARGET_COUNTS 80.0f
#define RB_FIRST_EXIT_DEG 270.0f
#define RB_SECOND_EXIT_DEG 360.0f
#define RB_TURN_SIGN 1 /* +1 clockwise; -1 counterclockwise */
/* Encounter order: photo1 then photo2. Positive yaw = clockwise, left wheel faster. */
enum {RB_IDLE,RB_ENTER,RB_ORBIT,RB_EXIT,RB_COOLDOWN};
typedef struct {
 float yaw_deg,distance_m;
 uint32_t ticks;
 uint8_t phase,route,confirm,clear,fault;
} rb_state_t;
static inline int rb_guiding(const volatile rb_state_t *s) {
 return s->phase==RB_ENTER || s->phase==RB_ORBIT;
}
static inline int rb_busy(const volatile rb_state_t *s) {
 return s->phase>=RB_ENTER && s->phase<=RB_EXIT;
}
static inline void rb_tick(volatile rb_state_t *s,int left,int right,int running) {
 if(!running || s->phase==RB_IDLE || s->fault)return;
 /* Invalid encoders are stopped by the existing supervisor, never integrated. */
 if(left<0 || right<0 || left>150 || right>150)return;
 s->distance_m+=(left+right)*0.5f*RB_METERS_PER_COUNT;
 if(s->phase==RB_COOLDOWN)return;
 s->yaw_deg+=RB_TURN_SIGN*(left-right)*RB_METERS_PER_COUNT/RB_REAR_TRACK_M*57.2957795f;
 ++s->ticks;
 if(s->phase==RB_ENTER && s->yaw_deg>=45.0f)s->phase=RB_ORBIT;
 if(rb_guiding(s) && s->yaw_deg>=(s->route==0 ? RB_FIRST_EXIT_DEG:RB_SECOND_EXIT_DEG)) {
  s->phase=RB_EXIT;s->clear=0;
 }
 if(s->ticks>=4500U || s->distance_m>=10.0f)s->fault=1;
}
/* Called once per new camera frame; repetition without motion cannot exit a ring. */
static inline void rb_observe(volatile rb_state_t *s,int island,int near_valid,int stripe,int running) {
 if(!running){s->confirm=0;return;}
 if(s->fault)return;
 if(s->phase==RB_IDLE) {
  if(s->route>=2)return;
  if(island && near_valid && !stripe) {
   if(++s->confirm>=3){s->phase=RB_ENTER;s->yaw_deg=0;s->distance_m=0;s->ticks=0;s->clear=0;}
  }else s->confirm=0;
 }else if(s->phase==RB_EXIT || s->phase==RB_COOLDOWN) {
  if(near_valid && !island && !stripe){if(s->clear<30)++s->clear;}else s->clear=0;
  if(s->phase==RB_EXIT && s->clear>=4) {
   s->phase=RB_COOLDOWN;++s->route;s->distance_m=0;s->clear=0;
  }else if(s->phase==RB_COOLDOWN && s->clear>=15 && s->distance_m>=0.30f) {
   s->phase=RB_IDLE;s->confirm=0;s->clear=0;
  }
 }
}
/* Entry assistance uses a CURRENT measured branch, only while entering.
 * It feeds the existing median/filter/slew owner; no direct servo command.
 * Ordinary bends, stripe recovery, loss and an unobserved branch get zero aid. */
static inline float rb_entry_error(int phase,int guide,float near,float branch,
                                  int branch_valid,int valid,int stripe) {
 float delta;
 if(phase!=RB_ENTER || !guide || !valid || !branch_valid || stripe ||
    branch*guide<=6.0f || (branch-near)*guide<=0.0f)return near;
 delta=(branch-near)*0.35f;
 if(delta>12.0f)delta=12.0f;
 if(delta< -12.0f)delta=-12.0f;
 return near+delta;
}
#endif
