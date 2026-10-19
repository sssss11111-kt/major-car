#include "car_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"LOW FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static control_state_t fresh(void){control_state_t s;control_config_t c={0};c.servo_center_us=1500;c.heartbeat_timeout_ms=100;
 c.low_kp=4;c.low_ki=20;c.low_ff_offset[0]=430;c.low_ff_offset[1]=560;c.low_ff_slope[0]=6.5f;c.low_ff_slope[1]=7;
 control_init(&s,&c);return s;}
static void step(control_state_t *s,uint32_t t,int l,int r){s->heartbeat_ms=t;control_drive_keepalive(s,t);control_tick(s,t,l,r);}
int main(void){
 for(unsigned motor=1;motor<=3;motor++){
  control_state_t s=fresh();CHECK(control_low_test(&s,motor,20,0));
  step(&s,10,0,0);CHECK(s.duty[0]==(motor==2?0:3000) && s.duty[1]==(motor==1?0:3000));
  for(unsigned t=20;t<=40;t+=10)step(&s,t,motor==2?0:20,motor==1?0:20);
  CHECK(s.low_confirmed==motor);CHECK(s.duty[0]==(motor==2?0:560));CHECK(s.duty[1]==(motor==1?0:700));
  for(unsigned t=50;t<=6000;t+=10)step(&s,t,motor==2?0:20,motor==1?0:20);
  CHECK(s.mode==CAR_IDLE && s.low_done==1 && s.low_elapsed==6000 && s.duty[0]==0 && s.duty[1]==0);
  for(unsigned w=0;w<2;w++){CHECK(s.low_n[w]==((motor&(1u<<w))?300:0));if(motor&(1u<<w))CHECK(s.low_sum[w]==6000 && s.low_error2[w]==0 && s.low_abs_error[w]==0 && s.low_min[w]==20 && s.low_max[w]==20);}
  unsigned id=s.low_id;control_stop(&s);CHECK(!s.low_done);CHECK(control_low_test(&s,motor,30,6000) && s.low_id!=id && !s.low_n[0]);
 }
 control_state_t s=fresh();CHECK(!control_low_test(&s,0,20,0)&&!control_low_test(&s,4,20,0)&&!control_low_test(&s,1,0,0)&&!control_low_test(&s,1,41,0));
 s.cfg.simulated=1;CHECK(!control_low_test(&s,1,20,0));
 s=fresh();CHECK(control_low_test(&s,3,20,0));for(unsigned t=10;t<=200;t+=10)step(&s,t,20,0);CHECK(s.fault==FAULT_ENCODER && !s.low_done && s.duty[0]==0);
 for(unsigned bad=0;bad<3;bad++){s=fresh();CHECK(control_low_test(&s,1,20,0));step(&s,10,bad==0?-6:bad==1?451:20,bad==2?11:0);CHECK(s.fault==FAULT_ENCODER);}
 s=fresh();CHECK(control_low_test(&s,1,20,0));control_tick(&s,110,0,0);CHECK(s.fault==FAULT_FOREGROUND);
 s=fresh();CHECK(control_low_test(&s,1,20,0));s.heartbeat_ms=500;control_tick(&s,500,20,0);CHECK(s.fault==FAULT_LINK);
 s=fresh();CHECK(control_low_test(&s,1,20,0));for(unsigned t=10;t<=30;t+=10)step(&s,t,20,0);
 step(&s,40,100,0);CHECK(s.duty[0]<560);step(&s,50,10,0);CHECK(s.duty[0]>560);
 for(unsigned t=60;t<=550;t+=10)step(&s,t,0,0);CHECK(s.fault==FAULT_ENCODER && !s.low_done);
 s=fresh();CHECK(control_low_test(&s,1,20,0));for(unsigned t=10;t<=1000;t+=10)step(&s,t,t<=30?20:450,0);CHECK(s.mode==CAR_LOW_SPEED && fabsf(s.integral[0])<=500 && s.duty[0]==0);
 s=fresh();s.heartbeat_ms=0xfffffff0u;CHECK(control_low_test(&s,2,20,0xfffffff0u));for(unsigned dt=10;dt<=6000;dt+=10)step(&s,0xfffffff0u+dt,0,20);CHECK(s.low_done && s.low_n[1]==300);
 puts("Low speed: kick / independent feedforward PI / full statistics / isolation / saturation / stall / lease / wrap PASS");return 0;
}
