#include <assert.h>
#include <stdio.h>
#include "roundabout_control.h"
int main(void) {
 rb_state_t s={0};
 assert(rb_entry_error(RB_ENTER,-1,0,-36,1,1,0)==-12);
 assert(rb_entry_error(RB_ENTER,1,0,36,1,1,0)==12);
 assert(rb_entry_error(RB_IDLE,-1,0,-36,1,1,0)==0);
 assert(rb_entry_error(RB_ORBIT,-1,0,-36,1,1,0)==0);
 assert(rb_entry_error(RB_ENTER,0,0,-36,1,1,0)==0);
 assert(rb_entry_error(RB_ENTER,-1,0,-36,0,1,0)==0);
 assert(rb_entry_error(RB_ENTER,-1,0,-36,1,0,0)==0);
 assert(rb_entry_error(RB_ENTER,-1,0,-36,1,1,1)==0);
 assert(rb_entry_error(RB_ENTER,-1,-40,-20,1,1,0)==-40);
 /* A stripe, a transient split or idle camera must not start a lap. */
 for(int i=0;i<5;i++)rb_observe(&s,1,1,1,0,1);
 assert(!rb_guiding(&s));
 rb_observe(&s,1,1,0,0,0);assert(!rb_guiding(&s));
 rb_observe(&s,1,1,0,0,1);rb_observe(&s,0,1,0,0,1);assert(!rb_guiding(&s));
 for(int i=0;i<3;i++)rb_observe(&s,1,1,0,0,1);
 assert(s.phase==RB_ENTER);
 /* Equal wheel counts cannot make a lap, and repeated images cannot either. */
 for(int i=0;i<100;i++){rb_tick(&s,80,80,1);rb_observe(&s,1,1,0,0,1);}
 assert(rb_guiding(&s)&&s.yaw_deg==0);
 /* Wrong-way movement must not satisfy the clockwise lap gate. */
 for(int i=0;i<40;i++)rb_tick(&s,40,80,1);
 assert(s.yaw_deg<0&&rb_guiding(&s));
 for(int i=0;i<440;i++)rb_tick(&s,80,40,1);
 assert(s.phase==RB_ORBIT && s.route==0); /* yaw alone retains guidance */
 for(int i=0;i<10;i++)rb_observe(&s,0,1,0,0,1);
 assert(rb_guiding(&s)); /* missing island alone is not an exit */
 rb_observe(&s,0,1,0,1,1);rb_observe(&s,0,1,0,0,1);
 assert(s.exit_confirm==0 && rb_guiding(&s));
 for(int i=0;i<3;i++)rb_observe(&s,0,1,0,1,1);
 assert(s.phase==RB_EXIT);
 /* Yaw alone cannot complete exit: it needs clean, current, near-field vision. */
 for(int i=0;i<10;i++)rb_observe(&s,0,0,0,0,1);
 assert(s.phase==RB_EXIT);
 for(int i=0;i<4;i++)rb_observe(&s,0,1,0,1,1);
 assert(s.phase==RB_COOLDOWN&&s.route==1);
 for(int i=0;i<200;i++){rb_tick(&s,80,80,1);rb_observe(&s,0,1,0,1,1);}
 assert(s.phase==RB_IDLE);
 for(int i=0;i<3;i++)rb_observe(&s,1,1,0,0,1);
 for(int i=0;i<400;i++)rb_tick(&s,80,40,1);
 assert(rb_guiding(&s)); /* second exit must wait for 360 degrees */
 for(int i=0;i<140;i++)rb_tick(&s,80,40,1);
 assert(rb_guiding(&s));
 for(int i=0;i<3;i++)rb_observe(&s,0,1,0,1,1);
 assert(s.phase==RB_EXIT);
 rb_state_t stalled={0};for(int i=0;i<3;i++)rb_observe(&stalled,1,1,0,0,1);
 for(int i=0;i<4500;i++)rb_tick(&stalled,0,0,1);
 assert(stalled.fault);
 assert(rb_plan_entry_error(RB_ENTER,-1,-10,-28,-36,1,1,0)<-28.0f);
 assert(rb_plan_entry_error(RB_ORBIT,-1,-10,-28,-36,1,1,0)<-28.0f);
 /* Clockwise guide=-1 denotes the road left of the island, not a permanent
  * left steering demand. Current measured right bends must remain actionable. */
 assert(rb_plan_entry_error(RB_ENTER,-1,0,4,30,1,1,0)>15.0f);
 assert(rb_plan_entry_error(RB_ORBIT,-1,0,4,30,1,1,0)>15.0f);
 assert(rb_plan_entry_error(RB_ENTER,1,0,-4,-30,1,1,0)<-15.0f);
 assert(rb_plan_entry_error(RB_ORBIT,-1,7,4,30,0,1,0)==7.0f);
 assert(rb_plan_entry_error(RB_ORBIT,-1,7,4,30,1,0,0)==7.0f);
 assert(rb_plan_entry_error(RB_ORBIT,-1,7,4,30,1,1,1)==7.0f);
 assert(rb_plan_entry_error(RB_ORBIT,-1,0,0,94,1,1,0)==18.0f);
 assert(rb_plan_entry_error(RB_IDLE,-1,-10,-28,-36,1,1,0)==-10.0f);
 {
  rb_state_t sp={0};
  rb_observe(&sp,1,1,0,0,1); assert(rb_suspect(&sp) && !sp.just_entered);
  rb_observe(&sp,1,1,0,0,1); assert(rb_suspect(&sp) && !sp.just_entered);
  rb_observe(&sp,1,1,0,0,1); assert(!rb_suspect(&sp) && sp.phase==RB_ENTER && sp.just_entered);
  rb_observe(&sp,1,1,0,0,1); assert(!sp.just_entered);
 }
 /* Every blocker breaks consecutiveness, not just a missing road flag. */
 for(int blocker=0;blocker<4;++blocker) {
  rb_state_t q={0};q.phase=RB_ORBIT;q.yaw_deg=300;
  rb_observe(&q,0,1,0,1,1);rb_observe(&q,0,1,0,1,1);
  assert(q.exit_confirm==2 && rb_guiding(&q));
  rb_observe(&q,blocker==0,blocker!=1,blocker==2,1,blocker!=3);
  assert(q.exit_confirm==0 && rb_guiding(&q));
  rb_observe(&q,0,1,0,1,1);assert(rb_guiding(&q));
 }
 {
  rb_state_t q={0};q.phase=RB_ORBIT;q.yaw_deg=269;
  for(int i=0;i<10;i++)rb_observe(&q,0,1,0,1,1);
  assert(rb_guiding(&q) && q.exit_confirm==0);
  q.phase=RB_EXIT;
  for(int i=0;i<3;i++)rb_observe(&q,0,1,0,1,1);
  rb_observe(&q,0,1,0,0,1);assert(q.clear==0 && q.phase==RB_EXIT);
  for(int i=0;i<4;i++)rb_observe(&q,0,1,0,1,1);
  assert(q.phase==RB_COOLDOWN);
  for(int i=0;i<40;i++)rb_observe(&q,0,1,0,1,1);
  assert(q.phase==RB_COOLDOWN); /* no cooldown distance, no rearming */
 }
 printf("PASS: roundabout entry, signed yaw, 270/360 visual exit gates, clean exit, cooldown and timeout\n");
 return 0;
}
