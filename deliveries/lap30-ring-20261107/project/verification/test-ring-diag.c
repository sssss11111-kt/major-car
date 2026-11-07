#include <assert.h>
#include <stdio.h>
#include "ring_diag.h"
#include "roundabout_control.h"
int main(void) {
 rd29_state_t s={0};uint32_t slot;uint32_t ids[RD29_CAPACITY]={0};
 assert(sizeof(rd29_frame_t)==23080U);
 assert(!rd29_reserve(&s,0,1,0,1,&slot));
 for(uint32_t i=1;i<=200;i++) {
  assert(rd29_reserve(&s,1,0,0,i,&slot));ids[slot]=i;rd29_commit(&s,0);
 }
 assert(s.count==32 && !s.triggered);
 assert(ids[rd29_index(&s,0)]==169 && ids[rd29_index(&s,31)]==200);
 for(uint32_t i=201;i<=264;i++) {
  assert(rd29_reserve(&s,1,i==201,0,i,&slot));ids[slot]=i;rd29_commit(&s,0);
 }
 assert(s.frozen && s.count==96 && s.trigger_frame==201 && s.reason==1);
 for(uint32_t i=0;i<96;i++)assert(ids[rd29_index(&s,i)]==169+i);
 assert(!rd29_reserve(&s,1,1,0,265,&slot));
 s=(rd29_state_t){0};
 assert(rd29_reserve(&s,1,0,4,7,&slot));rd29_commit(&s,1);
 assert(s.frozen && s.count==1 && s.reason==2);
 s=(rd29_state_t){0};rd29_freeze(&s);assert(!s.frozen);
 assert(rd29_reserve(&s,1,0,0,9,&slot));rd29_commit(&s,0);rd29_freeze(&s);
 assert(s.frozen && s.count==1 && !s.triggered);
 assert(rd29_crc("123456789",9)==0xcbf43926U);
 /* Reproduce the user's hypothesis with the actual unchanged state machine. */
 rb_state_t rb={0};
 for(int i=0;i<20;i++)rb_observe(&rb,1,0,0,0,1);
 assert(rb.phase==RB_IDLE && rb.confirm==0);
 rb_observe(&rb,1,1,0,0,1);assert(rb.confirm==1);
 rb_observe(&rb,1,0,0,0,1);assert(rb.confirm==0);
 for(int i=0;i<3;i++)rb_observe(&rb,1,1,0,0,1);
 assert(rb.phase==RB_ENTER && rb.just_entered);
 puts("PASS: chronological pretrigger/posttrigger, no overwrite after freeze, stop and no-candidate export, CRC, actual confirmation validity gate");
 return 0;
}
