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
#define RB_ALIGN 5 /* Append: preserve recorded phase IDs 0..4. */
#define RB_ENTRY_CORNER_ROW 80
#define RB_EXIT_SEARCH_LEAD_DEG 45.0f
/* Encounter order: photo1 then photo2. Positive yaw = clockwise, left wheel faster. */
enum {RB_IDLE,RB_ENTER,RB_ORBIT,RB_EXIT,RB_COOLDOWN};
typedef struct {
 float yaw_deg,distance_m;
 uint32_t ticks;
 uint8_t phase,route,confirm,clear,fault,just_entered,exit_confirm;
 uint8_t entry_ready,corner,first_corner,commit_confirm,exit_ready;
 int16_t exit_error,last_exit_error;
 float commit_yaw;
} rb_state_t;
static inline int rb_guiding(const volatile rb_state_t *s) {
  return s->phase==RB_ALIGN || s->phase==RB_ENTER || s->phase==RB_ORBIT || s->phase==RB_EXIT;
}
static inline int rb_busy(const volatile rb_state_t *s) {
  return rb_guiding(s);
}
static inline int rb_exit_search(const volatile rb_state_t *s) {
 return s->phase==RB_EXIT || (s->phase==RB_ORBIT &&
  s->yaw_deg-s->commit_yaw >= (s->route==0 ? RB_FIRST_EXIT_DEG:RB_SECOND_EXIT_DEG)-RB_EXIT_SEARCH_LEAD_DEG);
}
/* Fresh image facts, never predictions. Feed before rb_observe, once per frame. */
static inline void rb_geometry(volatile rb_state_t *s,int entry_ready,int corner,int exit_ready,int exit_error) {
 s->entry_ready=(uint8_t)entry_ready;s->corner=(uint8_t)corner;
 s->exit_ready=(uint8_t)exit_ready;s->exit_error=(int16_t)exit_error;
}
/* Pre-entry suspect state: decelerate early before switching path. */
static inline int rb_suspect(const volatile rb_state_t *s) {
 return s->phase==RB_IDLE && s->confirm>=1 && !s->fault;
}
static inline void rb_tick(volatile rb_state_t *s,int left,int right,int running) {
 if(!running || s->phase==RB_IDLE || s->fault)return;
 /* Invalid encoders are stopped by the existing supervisor, never integrated. */
 if(left<0 || right<0 || left>150 || right>150)return;
 s->distance_m+=(left+right)*0.5f*RB_METERS_PER_COUNT;
 if(s->phase==RB_COOLDOWN)return;
 s->yaw_deg+=RB_TURN_SIGN*(left-right)*RB_METERS_PER_COUNT/RB_REAR_TRACK_M*57.2957795f;
 ++s->ticks;
  if(s->phase==RB_ENTER && s->yaw_deg-s->commit_yaw>=45.0f)s->phase=RB_ORBIT;
 /* Yaw only arms visual exit recognition; it must never drop guidance alone. */
 if(s->ticks>=4500U || s->distance_m>=10.0f)s->fault=1;
}
/* Called once per new camera frame; repetition without motion cannot exit a ring. */
static inline void rb_observe(volatile rb_state_t *s,int island,int near_valid,int stripe,int road_recovered,int running) {
 s->just_entered=0;
  if(!running){s->confirm=0;s->exit_confirm=0;s->commit_confirm=0;s->clear=0;return;}
 if(s->fault)return;
 if(s->phase==RB_IDLE) {
  if(s->route>=2)return;
  if(island && near_valid && !stripe) {
   if(++s->confirm>=3){
     s->phase=RB_ALIGN;s->yaw_deg=0;s->distance_m=0;s->ticks=0;s->clear=0;s->exit_confirm=0;
     s->first_corner=s->corner;s->commit_confirm=0;
   }
  }else s->confirm=0;
  }else if(s->phase==RB_ALIGN) {
   if(!island && near_valid && road_recovered && !stripe) {
    if(++s->clear>=5) {
     s->phase=RB_IDLE;s->confirm=s->commit_confirm=s->clear=0;
     s->first_corner=0;return;
    }
   }else s->clear=0;
   /* A visible island starts alignment, not the turn. Close starts are allowed;
    * otherwise require a measured corner moving toward the camera. */
   int progress=s->corner>=96 || (s->corner>=RB_ENTRY_CORNER_ROW &&
                s->first_corner && s->corner>=s->first_corner+3);
   if(!s->first_corner && s->corner)s->first_corner=s->corner;
   if(s->entry_ready && near_valid && !stripe && progress) {
    if(++s->commit_confirm>=2) {
     s->phase=RB_ENTER;s->commit_yaw=s->yaw_deg;s->just_entered=1;s->commit_confirm=0;
    }
   }else s->commit_confirm=0;
  }else if(s->phase==RB_ENTER || s->phase==RB_ORBIT) {
   int delta=s->exit_error-s->last_exit_error;
   int branch=rb_exit_search(s) && s->exit_ready && delta>=-12 && delta<=12;
   s->last_exit_error=s->exit_error;
   /* A latched, current forward branch can commit before the island vanishes.
    * Full ordinary-road recovery remains a conservative fallback. */
   if(branch && near_valid && !stripe) {
    if(++s->exit_confirm>=3){s->phase=RB_EXIT;s->clear=0;s->exit_confirm=0;}
   }else
  if(s->yaw_deg-s->commit_yaw>=(s->route==0 ? RB_FIRST_EXIT_DEG:RB_SECOND_EXIT_DEG) &&
     road_recovered && near_valid && !island && !stripe) {
   if(++s->exit_confirm>=3){s->phase=RB_EXIT;s->clear=0;s->exit_confirm=0;}
  }else s->exit_confirm=0;
 }else if(s->phase==RB_EXIT || s->phase==RB_COOLDOWN) {
  if(road_recovered && near_valid && !island && !stripe){if(s->clear<30)++s->clear;}else s->clear=0;
  if(s->phase==RB_EXIT && s->clear>=4) {
   s->phase=RB_COOLDOWN;++s->route;s->distance_m=0;s->clear=0;
  }else if(s->phase==RB_COOLDOWN && s->clear>=15 && s->distance_m>=0.30f) {
   s->phase=RB_IDLE;s->confirm=0;s->clear=0;
  }
 }
}
/* Legacy scalar entry assistance retained for unit-test compatibility. */
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
/* LAP29 path-driven entry error: uses the corner-constrained target centerline
 * error (entry_target) measured on the current white branch corridor, plus
 * bounded branch preview lookahead when entering/orbiting. */
static inline float rb_plan_entry_error(int phase,int guide,float near,float entry_target,
                                       float branch,int branch_valid,int valid,int stripe) {
 float base, delta;
 if((phase!=RB_ENTER && phase!=RB_ORBIT) || !guide || !valid || stripe)return near;
 /* guide selects which side of the island contains the road; it is NOT
  * the steering sign. Clockwise circulation keeps the island on the right,
  * but the same corridor can bend right after an initial left entry. */
 if(!branch_valid)return near;
 base = entry_target;
 if(phase==RB_ENTER || phase==RB_ORBIT) {
  delta = (branch - base) * 0.55f;
  if(delta > 18.0f)delta = 18.0f;
  if(delta < -18.0f)delta = -18.0f;
  return base + delta;
 }
 return base;
}
#endif
