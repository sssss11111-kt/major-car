#include <assert.h>
#include <stdio.h>
#include "roundabout_control.h"
static void frame(rb_state_t *s,int side,int inside,int entry,int corner,int exit,int error,int island,int near,int stripe,int road) {
 rb_visual(s,side,inside);rb_geometry(s,entry,corner,exit,error);
 rb_observe(s,island,near,stripe,road,1);
}
static void enter(rb_state_t *s,int side) {
 for(int i=0;i<3;++i)frame(s,side,0,0,70,0,0,1,1,0,0);
 assert(s->phase==RB_ALIGN && s->turn_sign==side);
 for(int i=0;i<10;++i)frame(s,-side,0,1,70,1,20,1,1,0,0);
 assert(s->phase==RB_ALIGN && s->turn_sign==side); /* no remote corner or direction flip */
 frame(s,side,0,1,92,0,0,1,1,0,0);
 frame(s,side,0,1,93,0,0,1,1,0,0);
 assert(s->phase==RB_ENTER && s->just_entered);
}
static void orbit(rb_state_t *s,int side) {
 for(int i=0;i<3;++i)frame(s,side,1,0,0,0,0,0,1,0,0);
 assert(s->phase==RB_ORBIT && !s->exit_armed && rb_exit_search(s));
}
int main(void) {
 for(int side=-1;side<=1;side+=2) {
  rb_state_t s={0};
  for(int i=0;i<10;++i)frame(&s,0,0,1,100,0,0,1,1,0,0);
  assert(s.phase==RB_IDLE); /* centered/unknown island cannot guess direction */
  for(int i=0;i<10;++i)frame(&s,i%2?side:-side,0,1,100,0,0,1,1,0,0);
  assert(s.phase==RB_IDLE); /* ambiguous side must not accumulate */
  s=(rb_state_t){0};enter(&s,side);
  for(int i=0;i<200;++i)rb_tick(&s,80,40,1);
  assert(s.phase==RB_ENTER); /* any encoder angle leaves visual phase unchanged */
  for(int i=0;i<20;++i)frame(&s,side,0,1,100,1,20,1,1,0,0);
  assert(s.phase==RB_ENTER); /* repeated entrance cannot declare a lap */
  frame(&s,side,1,0,0,0,0,0,1,0,0);
  frame(&s,side,1,0,0,0,0,0,0,0,0);
  assert(!s.inside_confirm);
  orbit(&s,side);
  for(int i=0;i<10;++i)frame(&s,side,1,0,0,1,20,1,1,0,0);
  assert(!s.exit_armed && s.phase==RB_ORBIT); /* visible opening blocks clean-arc arm */
  for(int i=0;i<3;++i)frame(&s,side,1,0,0,0,0,0,1,0,0);
  assert(s.exit_armed);
  s.yaw_deg=0;s.distance_m=0;
  for(int i=0;i<10;++i)frame(&s,side,0,0,0,0,0,0,1,0,1);
  assert(s.phase==RB_ORBIT); /* plain normal road is not an independently observed exit */
  rb_exit_tip(&s,60,110);
  for(int i=0;i<10;++i)frame(&s,side,0,0,0,1,20,1,1,0,0);
  assert(s.phase==RB_ORBIT && !s.exit_confirm); /* stationary corner */
  rb_exit_tip(&s,64,105);frame(&s,side,0,0,0,1,20,1,1,0,0);
  assert(s.exit_confirm==1);
  frame(&s,side,0,0,0,1,20,1,1,1,0);assert(!s.exit_confirm);
  for(int i=0;i<3;++i) {
   rb_exit_tip(&s,65+i*2,103-i*3);
   frame(&s,side,0,0,0,1,20+i,1,1,0,0);
  }
  assert(s.phase==RB_EXIT && s.yaw_deg==0); /* visual decision, no movement threshold */
  frame(&s,side,0,0,0,0,0,0,0,0,0);assert(s.phase==RB_EXIT);
  for(int i=0;i<4;++i)frame(&s,0,0,0,0,0,0,0,1,0,1);
  assert(s.phase==RB_COOLDOWN);
  for(int i=0;i<15;++i)frame(&s,0,0,0,0,0,0,0,1,0,1);
  assert(s.phase==RB_IDLE && s.turn_sign==0 && s.route==1);
  enter(&s,-side);assert(s.turn_sign==-side);
  rb_reject_entry(&s);assert(s.phase==RB_ALIGN && !s.just_entered);
 }
 rb_state_t timeout={0};enter(&timeout,1);
 for(int i=0;i<4500;++i)rb_tick(&timeout,0,0,1);
 assert(timeout.fault);
 assert(rb_plan_entry_error(RB_ORBIT,-1,4,30,80,1,1,0)==4);
 puts("PASS: mirrored visual acquisition, alignment, arc handoff, opening rejection, visual exit, cooldown, watchdog; no yaw phase trigger");
 return 0;
}
