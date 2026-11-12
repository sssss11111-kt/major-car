#ifndef ROUNDABOUT_CONTROL_H
#define ROUNDABOUT_CONTROL_H
#include <stdint.h>
/* Provisional scale is used for watchdog/bridge diagnostics, never exit timing. */
#define RB_REAR_TRACK_M 0.155f
#define RB_METERS_PER_COUNT 0.00004620f
#define RB_TARGET_COUNTS 80.0f
#define RB_ALIGN 5
#define RB_ENTRY_CORNER_ROW 90
#define RB_VISUAL_CONFIRM_FRAMES 3
/* Preserve recorded phase IDs. Direction is latched from the entrance image. */
enum {RB_IDLE,RB_ENTER,RB_ORBIT,RB_EXIT,RB_COOLDOWN};
typedef struct {
 float yaw_deg,distance_m;
 uint32_t ticks;
 uint8_t phase,route,confirm,clear,fault,just_entered,exit_confirm;
 uint8_t entry_ready,corner,first_corner,commit_confirm,exit_ready;
 int16_t exit_error,last_exit_error;
 float commit_yaw; /* diagnostic only */
 int8_t turn_sign,observed_side;
 uint8_t inside,inside_confirm,exit_armed;
 uint8_t exit_tip_row,exit_tip_x,exit_first_row,exit_last_row,exit_last_x,exit_gap;
} rb_state_t;
static inline int rb_guiding(const volatile rb_state_t *s) {
 return s->phase==RB_ALIGN || s->phase==RB_ENTER || s->phase==RB_ORBIT || s->phase==RB_EXIT;
}
static inline int rb_busy(const volatile rb_state_t *s) { return rb_guiding(s); }
static inline int rb_exit_search(const volatile rb_state_t *s) {
 /* Observe openings before arming too; absence must be measured, not gated. */
 return s->phase==RB_EXIT || s->phase==RB_ORBIT;
}
/* Current-image evidence: +1 island on entrance right, -1 on left, 0 unknown.
 * inside means a continuous current inner bank reaches the near bottom, with
 * an ordinary near corridor; a stored/white-only path cannot establish it. */
static inline void rb_visual(volatile rb_state_t *s,int side,int inside) {
 s->observed_side=(int8_t)(side==1 || side==-1 ? side:0);
 s->inside=(uint8_t)inside;
}
static inline void rb_exit_tip(volatile rb_state_t *s,int row,int x) {
 s->exit_tip_row=(uint8_t)row;s->exit_tip_x=(uint8_t)x;
}
static inline void rb_geometry(volatile rb_state_t *s,int entry_ready,int corner,int exit_ready,int exit_error) {
 s->entry_ready=(uint8_t)entry_ready;s->corner=(uint8_t)corner;
 s->exit_ready=(uint8_t)exit_ready;s->exit_error=(int16_t)exit_error;
}
static inline int rb_suspect(const volatile rb_state_t *s) {
 return s->phase==RB_IDLE && s->confirm>=1 && !s->fault;
}
static inline void rb_tick(volatile rb_state_t *s,int left,int right,int running) {
 if(!running || s->phase==RB_IDLE || s->fault)return;
 if(left<0 || right<0 || left>150 || right>150)return;
 s->distance_m+=(left+right)*0.5f*RB_METERS_PER_COUNT;
 if(s->phase==RB_COOLDOWN)return;
 /* This estimate bounds the existing short loss bridge; no FSM transition
  * below uses yaw or distance to select an entrance, orbit or exit. */
 s->yaw_deg+=s->turn_sign*(left-right)*RB_METERS_PER_COUNT/RB_REAR_TRACK_M*57.2957795f;
 ++s->ticks;
 if(s->ticks>=4500U || s->distance_m>=10.0f)s->fault=1;
}
static inline void rb_observe(volatile rb_state_t *s,int island,int near_valid,int stripe,int road_recovered,int running) {
 s->just_entered=0;
 if(!running){s->confirm=s->exit_confirm=s->commit_confirm=s->inside_confirm=s->clear=0;return;}
 if(s->fault)return;
 if(s->phase==RB_IDLE) {
  if(island && near_valid && !stripe && s->observed_side) {
   if(s->turn_sign!=s->observed_side){s->turn_sign=s->observed_side;s->confirm=0;}
   if(++s->confirm>=RB_VISUAL_CONFIRM_FRAMES) {
    s->phase=RB_ALIGN;s->yaw_deg=0;s->distance_m=0;s->ticks=0;
    s->clear=s->exit_confirm=s->inside_confirm=s->exit_armed=0;
    s->exit_first_row=s->exit_last_row=s->exit_gap=0;
    s->first_corner=s->corner;s->commit_confirm=0;
   }
  }else s->confirm=0;
 }else if(s->phase==RB_ALIGN) {
  if(!island && near_valid && road_recovered && !stripe) {
   if(++s->clear>=5){s->phase=RB_IDLE;s->confirm=s->commit_confirm=s->clear=0;s->turn_sign=0;return;}
  }else s->clear=0;
  int progress=s->corner>=96 || (s->corner>=RB_ENTRY_CORNER_ROW &&
      s->first_corner && s->corner>=s->first_corner+3);
  if(!s->first_corner && s->corner)s->first_corner=s->corner;
  if(s->entry_ready && near_valid && !stripe && progress) {
   if(++s->commit_confirm>=2){s->phase=RB_ENTER;s->commit_yaw=s->yaw_deg;
    s->just_entered=1;s->commit_confirm=s->inside_confirm=0;}
  }else s->commit_confirm=0;
 }else if(s->phase==RB_ENTER) {
  if(s->inside && near_valid && !stripe) {
   if(++s->inside_confirm>=RB_VISUAL_CONFIRM_FRAMES) {
    s->phase=RB_ORBIT;s->inside_confirm=0;s->exit_armed=0;
   }
  }else s->inside_confirm=0;
 }else if(s->phase==RB_ORBIT) {
  /* First observe a clean inner arc after entry. A stationary entrance or an
   * opening seen during entry cannot arm/commit exit, regardless of yaw. */
  if(!s->exit_armed) {
   if(s->inside && near_valid && !stripe && !s->exit_ready) {
    if(++s->inside_confirm>=RB_VISUAL_CONFIRM_FRAMES)s->exit_armed=1;
   }else s->inside_confirm=0;
   s->exit_confirm=0;return;
  }
  if(s->exit_ready && near_valid && !stripe && s->exit_tip_row) {
   int dy=(int)s->exit_tip_row-s->exit_last_row;
   int dx=(int)s->exit_tip_x-s->exit_last_x;
   if(!s->exit_first_row || (s->exit_last_row && (dy< -4 || dy>16 || dx< -24 || dx>24))) {
    s->exit_first_row=s->exit_tip_row;s->exit_confirm=0;
   }
   s->exit_last_row=s->exit_tip_row;s->exit_last_x=s->exit_tip_x;s->exit_gap=0;
   /* The same outer-bank corner must advance toward the camera into the
    * preview window. Repeating a stationary image never completes a lap. */
   if(s->exit_tip_row>=55 && s->exit_tip_row>=s->exit_first_row+3) {
    if(++s->exit_confirm>=RB_VISUAL_CONFIRM_FRAMES) {
     s->phase=RB_EXIT;s->clear=s->exit_confirm=0;
    }
   }else s->exit_confirm=0;
  }else {
   s->exit_confirm=0;
   if(++s->exit_gap>=3){s->exit_first_row=s->exit_last_row=0;s->exit_gap=3;}
  }
 }else if(s->phase==RB_EXIT || s->phase==RB_COOLDOWN) {
  if(road_recovered && near_valid && !island && !stripe){if(s->clear<30)++s->clear;}else s->clear=0;
  if(s->phase==RB_EXIT && s->clear>=4){s->phase=RB_COOLDOWN;
   if(s->route<255)++s->route;s->distance_m=0;s->clear=0;
  }else if(s->phase==RB_COOLDOWN && s->clear>=15){s->phase=RB_IDLE;
   s->confirm=s->clear=s->exit_armed=0;s->turn_sign=0;}
 }
}
static inline void rb_reject_entry(volatile rb_state_t *s) {
 if(s->phase==RB_ENTER && s->just_entered){s->phase=RB_ALIGN;
  s->just_entered=0;s->commit_confirm=s->inside_confirm=0;}
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
 if(phase!=RB_ENTER || !guide || !valid || stripe)return near;
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
