#include "car_core.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"GROUND FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static control_state_t fresh(void){control_state_t s;control_config_t c={0};c.servo_center_us=1500;c.heartbeat_timeout_ms=100;control_init(&s,&c);return s;}
static void step(control_state_t *s,unsigned t,int l,int r){s->heartbeat_ms=t;control_drive_keepalive(s,t);control_tick(s,t,l,r);}
int main(void){
 control_state_t s=fresh();CHECK(!control_ground_test(&s,899,0));CHECK(!control_ground_test(&s,2001,0));
 s.cfg.simulated=1;CHECK(!control_ground_test(&s,1000,0));s=fresh();
 CHECK(control_ground_test(&s,1000,0));CHECK(s.duty[0]==0&&s.duty[1]==0&&s.servo_us==1500);
 for(unsigned t=10;t<200;t+=10){step(&s,t,0,0);CHECK(s.duty[0]==0&&s.duty[1]==0);}
 for(unsigned t=200;t<400;t+=10){step(&s,t,20,20);CHECK(s.duty[0]==3000&&s.duty[1]==3000);}
 step(&s,400,20,20);CHECK(s.ground_phase==2&&s.duty[0]==1000&&s.duty[1]==1000);
 for(unsigned t=410;t<=1900;t+=10){step(&s,t,20,20);CHECK(s.servo_us==1500);}
 CHECK(s.mode==CAR_IDLE&&s.ground_done==1&&s.ground_elapsed==1900&&s.duty[0]==0&&s.duty[1]==0);
 CHECK(s.low_n[0]==50&&s.low_n[1]==50&&s.low_sum[0]==1000&&s.low_sum[1]==1000);
 control_stop(&s);CHECK(!s.ground_done);
 for(unsigned wheel=0;wheel<2;wheel++){s=fresh();CHECK(control_ground_test(&s,1000,0));for(unsigned t=10;t<=500;t+=10)step(&s,t,wheel?20:0,wheel?0:20);CHECK(s.fault==FAULT_ENCODER&&s.duty[0]==0&&s.duty[1]==0&&!s.ground_done);}
 s=fresh();CHECK(control_ground_test(&s,1000,0));for(unsigned t=10;t<=400;t+=10)step(&s,t,20,20);for(unsigned t=410;t<=900;t+=10)step(&s,t,0,20);CHECK(s.fault==FAULT_ENCODER&&s.duty[0]==0&&s.duty[1]==0);
 for(unsigned bad=0;bad<2;bad++){s=fresh();CHECK(control_ground_test(&s,1000,0));step(&s,210,bad?121:-6,20);CHECK(s.fault==FAULT_ENCODER);}
 s=fresh();CHECK(control_ground_test(&s,1000,0));control_tick(&s,110,0,0);CHECK(s.fault==FAULT_FOREGROUND);
 s=fresh();CHECK(control_ground_test(&s,1000,0));s.heartbeat_ms=500;control_tick(&s,500,20,20);CHECK(s.fault==FAULT_LINK);
 s=fresh();CHECK(control_ground_test(&s,1000,0));step(&s,2000,20,20);CHECK(s.mode==CAR_IDLE&&!s.ground_done);
 s=fresh();s.heartbeat_ms=0xfffffff0u;CHECK(control_ground_test(&s,1000,0xfffffff0u));for(unsigned d=10;d<=1900;d+=10)step(&s,0xfffffff0u+d,20,20);CHECK(s.ground_done&&s.ground_elapsed==1900);
 s=fresh();CHECK(control_ground_test(&s,2000,0));for(unsigned t=10;t<=400;t+=10)step(&s,t,20,20);CHECK(s.duty[0]==2000&&s.duty[1]==2000);

 s=fresh();CHECK(control_ground_closed_test(&s,0));for(unsigned t=10;t<=400;t+=10)step(&s,t,20,20);
 CHECK(s.low_regulating==3&&s.duty[0]==1500&&s.duty[1]==1500);
 float before=s.duty[0];step(&s,410,50,20);CHECK(s.duty[0]<before&&s.duty[1]<=1500);
 for(unsigned t=420;t<=1900;t+=10)step(&s,t,20,20);
 CHECK(s.ground_done&&s.low_error2[0]==0&&s.low_error2[1]==0);
 s=fresh();CHECK(control_ground_closed_test(&s,0));for(unsigned t=10;t<=400;t+=10)step(&s,t,20,20);
 for(unsigned t=410;t<=1800;t+=10)step(&s,t,120,120);CHECK(s.mode==CAR_GROUND_TEST&&s.duty[0]<1206&&s.duty[1]<1198&&s.integral[0]>=-500&&s.integral[1]>=-500);
 puts("Ground hold: settle/kick/both feedback/drop/stats/stall/guard/lease/timeout/wrap PASS");return 0;
}
