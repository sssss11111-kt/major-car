#include <assert.h>
#include <stdio.h>
#include "roundabout_control.h"
static void observe(rb_state_t *s,int corner,int ready,int island,int road) {
 rb_geometry(s,ready,corner,0,0);rb_observe(s,island,1,0,road,1);
}
static void enter(rb_state_t *s) {
 for(int i=0;i<3;++i)observe(s,70,0,1,0);
 assert(s->phase==RB_ALIGN && !s->just_entered);
 for(int i=0;i<20;++i)observe(s,70,1,1,0);
 assert(s->phase==RB_ALIGN); /* repetition of distant corner never commits */
 for(int row=82;row<=89;++row)observe(s,row,1,1,0);
 assert(s->phase==RB_ALIGN && !s->commit_confirm); /* far path still cuts the throat */
 observe(s,90,1,1,0);assert(s->phase==RB_ALIGN && s->commit_confirm==1);
 observe(s,91,0,1,0);assert(!s->commit_confirm);
 observe(s,92,1,1,0);observe(s,93,1,1,0);
 assert(s->phase==RB_ENTER && s->just_entered);
}
int main(void) {
 rb_state_t s={0};
 for(int i=0;i<5;++i)rb_observe(&s,1,1,1,0,1);
 assert(s.phase==RB_IDLE);
 rb_observe(&s,1,1,0,0,0);assert(s.phase==RB_IDLE);
 enter(&s);
 for(int i=0;i<100;++i){rb_tick(&s,80,80,1);observe(&s,100,1,1,0);}
 assert(s.phase==RB_ENTER && s.yaw_deg==0);
 for(int i=0;i<40;++i)rb_tick(&s,60-20*RB_TURN_SIGN,60+20*RB_TURN_SIGN,1);
 assert(s.yaw_deg<0);
 for(int i=0;i<440;++i)rb_tick(&s,60+20*RB_TURN_SIGN,60-20*RB_TURN_SIGN,1);
 assert(s.phase==RB_ORBIT && rb_exit_search(&s));
 for(int i=0;i<20;++i)observe(&s,0,0,0,0);
 assert(s.phase==RB_ORBIT); /* absent island alone never exits */
 for(int i=0;i<3;++i)observe(&s,0,0,0,1);
 assert(s.phase==RB_EXIT && rb_guiding(&s));
 for(int i=0;i<4;++i)observe(&s,0,0,0,1);
 assert(s.phase==RB_COOLDOWN && s.route==1);
 for(int i=0;i<40;++i)observe(&s,0,0,0,1);
 assert(s.phase==RB_COOLDOWN); /* distance required */
 for(int i=0;i<100;++i){rb_tick(&s,80,80,1);observe(&s,0,0,0,1);}
 assert(s.phase==RB_IDLE);
 enter(&s);
 s.phase=RB_ORBIT;s.yaw_deg=300;
 assert(!rb_exit_search(&s));
 s.yaw_deg=315;assert(rb_exit_search(&s));
 /* A current branch can commit while the island remains visible, but a jump,
  * stripe or stopped controller must break consecutive confirmation. */
 for(int blocker=0;blocker<3;++blocker) {
  rb_state_t q={0};q.phase=RB_ORBIT;q.yaw_deg=230;
  for(int i=0;i<2;++i){rb_geometry(&q,0,0,1,8);rb_observe(&q,1,1,0,0,1);}
  assert(q.phase==RB_ORBIT && q.exit_confirm==2);
  rb_geometry(&q,0,0,1,blocker==0?60:8);
  rb_observe(&q,1,1,blocker==1,0,blocker!=2);
  assert(q.phase==RB_ORBIT && !q.exit_confirm);
 }
 rb_state_t q={0};q.phase=RB_ORBIT;q.yaw_deg=224;
 for(int i=0;i<5;++i){rb_geometry(&q,0,0,1,8);rb_observe(&q,1,1,0,0,1);}
 assert(q.phase==RB_ORBIT);
 q.yaw_deg=230;
 for(int i=0;i<3;++i){rb_geometry(&q,0,0,1,8);rb_observe(&q,1,1,0,0,1);}
 assert(q.phase==RB_EXIT);
 for(int i=0;i<10;++i)rb_observe(&q,1,1,0,0,1);
 assert(q.phase==RB_EXIT); /* commitment is retained through incomplete recovery */
 rb_state_t rejected={0};enter(&rejected);
 rb_reject_entry(&rejected);
 assert(rejected.phase==RB_ALIGN && !rejected.just_entered && !rejected.commit_confirm);
 observe(&rejected,98,1,1,0);
 assert(rejected.phase==RB_ALIGN && rejected.commit_confirm==1);
 observe(&rejected,99,1,1,0);
 assert(rejected.phase==RB_ENTER && rejected.just_entered);
 rb_state_t timeout={0};enter(&timeout);
 for(int i=0;i<4500;++i)rb_tick(&timeout,0,0,1);
 assert(timeout.fault);
 assert(rb_plan_entry_error(RB_ORBIT,-1,0,4,30,1,1,0)>15);
 assert(rb_entry_error(RB_ENTER,-1,0,-36,1,1,0)==-12);
 rb_state_t recovered={0};recovered.phase=RB_ALIGN;
 for(int i=0;i<5;++i)observe(&recovered,0,0,0,1);
 assert(recovered.phase==RB_IDLE && recovered.route==0);
 rb_state_t anchored={0};anchored.phase=RB_ORBIT;anchored.commit_yaw=30;
 anchored.yaw_deg=250;assert(!rb_exit_search(&anchored));
 anchored.yaw_deg=260;assert(rb_exit_search(&anchored));
 puts("PASS R17: alignment, corner progress, consecutive evidence, signed yaw, exit branch lock, recovery, cooldown, timeout");
 return 0;
}
