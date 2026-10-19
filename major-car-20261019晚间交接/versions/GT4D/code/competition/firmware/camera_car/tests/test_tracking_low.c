#include "car_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"TRACK LOW FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static control_state_t fresh(void){control_state_t s;control_config_t c={0};
 c.speed_target=20;c.pwm_max=3000;c.tracking_low_speed=1;c.low_kp=1;c.low_ki=20;
 c.low_ff_offset[0]=570;c.low_ff_offset[1]=690;c.low_ff_slope[0]=6.5f;c.low_ff_slope[1]=7;
 c.steer_kp=3;c.steer_sign=-1;c.servo_center_us=1500;c.servo_min_us=1350;c.servo_max_us=1650;
 c.frame_timeout_ms=150;c.heartbeat_timeout_ms=100;c.tracking_settle_ms=200;c.tracking_duration_ms=3000;c.tracking_lease_ms=500;
 control_init(&s,&c);return s;}
static void frame(control_state_t *s,uint32_t t,int valid,float error){vision_result_t f={0};f.valid=(uint8_t)valid;f.error=error;s->heartbeat_ms=t;control_frame(s,&f,t);}
static void start(control_state_t *s){frame(s,20,1,0);frame(s,40,1,0);frame(s,60,1,0);CHECK(control_start(s,60));}
static void step(control_state_t *s,uint32_t t,int l,int r){frame(s,t,1,0);control_drive_keepalive(s,t);control_tick(s,t,l,r);}
int main(void){
 control_state_t s=fresh();start(&s);
 for(unsigned t=70;t<260;t+=10){step(&s,t,0,0);CHECK(s.duty[0]==0 && s.duty[1]==0);}
 for(unsigned t=260;t<460;t+=10){step(&s,t,20,20);CHECK(s.duty[0]==3000 && s.duty[1]==3000);}
 step(&s,460,100,100);CHECK(s.duty[0]==700 && s.duty[1]==830 && s.low_regulating==0);
 for(unsigned t=470;t<1260;t+=10)step(&s,t,28,18);
 step(&s,1260,28,18);CHECK(s.low_regulating==3 && s.duty[0]==700 && s.duty[1]==830);
 frame(&s,1270,1,20);control_drive_keepalive(&s,1270);control_tick(&s,1270,20,20);CHECK(s.servo_us==1440 && s.duty[0]<=1500 && s.duty[1]<=1500);
 for(unsigned t=1280;t<=3260;t+=10)step(&s,t,20,20);
 CHECK(s.mode==CAR_IDLE && s.trial_done && s.trial_elapsed_ms==3200 && s.duty[0]==0 && s.duty[1]==0);
 s=fresh();start(&s);for(unsigned t=70;t<=570;t+=10){step(&s,t,t>=270?20:0,0);if(s.fault)break;}CHECK(s.fault==FAULT_ENCODER && !s.trial_done && s.duty[0]==0);
 s=fresh();start(&s);for(unsigned t=70;t<=560;t+=10){frame(&s,t,1,0);control_tick(&s,t,20,20);}CHECK(s.fault==FAULT_LINK);
 s=fresh();start(&s);for(unsigned t=70;t<=90;t+=10){frame(&s,t,0,0);control_tick(&s,t,0,0);}CHECK(s.fault==FAULT_LINE);
 s=fresh();start(&s);s.cfg.speed_target=0;for(unsigned t=70;t<=700;t+=10){step(&s,t,0,0);CHECK(s.duty[0]==0 && s.duty[1]==0);}s.cfg.speed_target=20;step(&s,710,0,0);CHECK(s.duty[0]==3000 && !s.low_regulating);
 s=fresh();s.cfg.low_ki=0;frame(&s,20,1,0);frame(&s,40,1,0);frame(&s,60,1,0);CHECK(!control_start(&s,60));

 s=fresh();s.cfg.tracking_ground=1;start(&s);
 for(unsigned t=70;t<460;t+=10)step(&s,t,t>=260?20:0,t>=260?20:0);
 step(&s,460,20,20);CHECK(s.low_regulating==3&&fabsf(s.duty[0]-1206)<1&&fabsf(s.duty[1]-1198)<1);
 frame(&s,470,1,20);control_drive_keepalive(&s,470);control_tick(&s,470,20,20);CHECK(s.servo_us==1440&&s.duty[0]<=1500);
 frame(&s,480,0,0);frame(&s,490,0,0);frame(&s,500,0,0);CHECK(s.fault==FAULT_LINE&&s.duty[0]==0&&s.duty[1]==0);

 s=fresh();s.cfg.tracking_ground=1;s.cfg.ground_tracking_cap=3000;s.cfg.speed_target=30;s.cfg.tracking_duration_ms=5000;start(&s);
 for(unsigned t=70;t<=460;t+=10)step(&s,t,20,20);
 for(unsigned t=470;t<=860;t+=10)step(&s,t,0,0);
 CHECK(s.mode==CAR_RUNNING&&s.duty[0]>1500&&s.duty[0]<=3000);
 step(&s,870,121,20);CHECK(s.fault==FAULT_ENCODER&&s.duty[0]==0&&s.duty[1]==0);
 puts("Tracking calibrated controller: settle/kick/feedforward/bumpless PI/steering/expiry/guards/zero restart PASS");return 0;
}
