/* STEP-01..06 integration contracts: production core, no physical claim. */
#include "car_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define CHECK(x) do { if(!(x)) {fprintf(stderr,"FAIL tracking:%d %s\n",__LINE__,#x);exit(1);} } while(0)
static control_config_t config(void) {
    control_config_t c={0};
    c.speed_target=20;c.speed_kp=4;c.speed_ki=20;c.pwm_max=3000;
    c.tracking_pwm_bias[0]=2200;c.tracking_pwm_bias[1]=1500;
    c.steer_kp=3;c.steer_kd=.08f;c.steer_sign=-1;
    c.servo_center_us=1500;c.servo_min_us=1350;c.servo_max_us=1650;
    c.frame_timeout_ms=150;c.heartbeat_timeout_ms=100;
    c.tracking_settle_ms=200;c.tracking_duration_ms=3000;c.tracking_lease_ms=500;return c;
}
static void frame(control_state_t *s,uint32_t t,float e) {
    vision_result_t f={0};f.valid=1;f.error=e;s->heartbeat_ms=t;control_frame(s,&f,t);
}
static void ready(control_state_t *s) {frame(s,20,0);frame(s,40,0);frame(s,60,0);}
static void speed_tests(void) {
    control_state_t s;control_config_t c=config();control_init(&s,&c);ready(&s);
    CHECK(control_start(&s,60));frame(&s,270,25);control_tick(&s,270,0,0);
    CHECK(s.duty[0]==3000 && s.duty[1]==3000);
    float steer=0;
    for(uint32_t t=280;t<=300;t+=10){frame(&s,t,25);steer=s.servo_us;control_tick(&s,t,6,6);CHECK(fabsf(s.servo_us-steer)<.1f);}
    CHECK(s.bench_started==3 && s.duty[0]<2600 && s.duty[0]>0 && s.duty[1]<2000);
    CHECK(fabsf(s.servo_us-steer)<.1f); /* Speed update must not center vision steering. */
    control_stop(&s);ready(&s);CHECK(control_start(&s,60));
    for(uint32_t t=270;t<=470 && s.mode!=CAR_FAULT;t+=10){frame(&s,t,0);control_tick(&s,t,6,0);}
    CHECK(s.mode==CAR_FAULT && s.fault==FAULT_ENCODER && s.duty[0]==0 && s.duty[1]==0);
    control_init(&s,&c);ready(&s);CHECK(control_start(&s,60));s.cfg.speed_target=0;
    for(uint32_t t=70;t<=350;t+=10){frame(&s,t,0);control_drive_keepalive(&s,t);control_tick(&s,t,0,0);CHECK(s.duty[0]==0 && s.duty[1]==0);}
    s.cfg.speed_target=20;frame(&s,360,0);control_tick(&s,360,0,0);CHECK(s.duty[0]==3000 && s.duty[1]==3000);
    puts("STEP-01 shared tracking speed/startup/steering isolation: PASS");
}
static void start_guard_tests(void) {
    control_state_t s;control_config_t c=config();control_init(&s,&c);
    frame(&s,20,25);frame(&s,40,25);frame(&s,60,25);CHECK(control_start(&s,60));
    CHECK(fabsf(s.servo_us-1425)<.1f); /* Use current vision immediately, no neutral kick. */
    frame(&s,70,25);control_tick(&s,70,0,0);CHECK(s.duty[0]==0 && s.duty[1]==0);
    for(uint32_t t=80;t<=260;t+=10){frame(&s,t,25);control_tick(&s,t,0,0);}
    CHECK(s.duty[0]==3000);
    control_stop(&s);CHECK(s.duty[0]==0 && s.servo_us==1500);
    puts("STEP-02 visual steering before startup: PASS");
}
static void trial_guard_tests(void) {
    control_state_t s;control_config_t c=config();
    control_init(&s,&c);ready(&s);CHECK(control_start(&s,60));
    for(uint32_t t=70;t<=560;t+=10){frame(&s,t,0);control_tick(&s,t,t>=270?20:0,t>=270?20:0);}
    CHECK(s.mode==CAR_FAULT && s.fault==FAULT_LINK && s.duty[0]==0);
    control_drive_keepalive(&s,570);CHECK(s.mode==CAR_FAULT);
    control_init(&s,&c);ready(&s);CHECK(control_start(&s,60));
    for(uint32_t t=70;t<=3260;t+=10){frame(&s,t,0);control_drive_keepalive(&s,t);control_tick(&s,t,t>=270?20:0,t>=270?20:0);}
    CHECK(s.mode==CAR_IDLE && s.duty[0]==0 && s.duty[1]==0 && s.servo_us==1500);
    CHECK(s.trial_done==1 && s.trial_id==1 && s.trial_elapsed_ms==3200);
    control_drive_keepalive(&s,3270);CHECK(s.mode==CAR_IDLE && s.trial_done==1);
    control_stop(&s);CHECK(s.trial_done==0 && s.trial_id==1 && !control_tracking_ready(&s,3270));
    frame(&s,3280,0);frame(&s,3300,0);CHECK(!control_tracking_ready(&s,3300));
    frame(&s,3320,0);CHECK(control_tracking_ready(&s,3320));CHECK(control_start(&s,3320));
    CHECK(s.trial_id==2 && s.trial_done==0);
    control_fault(&s,FAULT_LINE);CHECK(s.trial_done==0 && s.trial_id==2);
    control_init(&s,&c);uint32_t start=0xffffff80u;
    frame(&s,start-40,0);frame(&s,start-20,0);frame(&s,start,0);CHECK(control_start(&s,start));
    for(uint32_t d=10;d<=3200;d+=10){uint32_t t=start+d;frame(&s,t,0);control_drive_keepalive(&s,t);control_tick(&s,t,d>=210?20:0,d>=210?20:0);}
    CHECK(s.mode==CAR_IDLE && s.duty[0]==0);
    control_init(&s,&c);ready(&s);CHECK(control_start(&s,60));
    vision_result_t bad={0};for(uint32_t t=70;t<=90;t+=10){s.heartbeat_ms=t;control_frame(&s,&bad,t);control_tick(&s,t,0,0);}
    CHECK(s.mode==CAR_FAULT && s.fault==FAULT_LINE && s.duty[0]==0);
    puts("STEP-02 trial duration/link/wrap/line loss during preparation: PASS");
}
/* STEP-04: capture age, not completion time, authorizes tracking. */
extern void control_frame_at(control_state_t *,const vision_result_t *,uint32_t,uint32_t);
static void capture_age_tests(void) {
    control_state_t s;control_config_t c=config();vision_result_t f={0};f.valid=1;
    control_init(&s,&c);control_frame_at(&s,&f,20,180);CHECK(s.good_frames==0);
    control_frame_at(&s,&f,200,200);control_frame_at(&s,&f,220,220);control_frame_at(&s,&f,240,240);
    s.heartbeat_ms=240;CHECK(control_start(&s,240));
    control_frame_at(&s,&f,250,410);CHECK(s.mode==CAR_FAULT && s.fault==FAULT_FRAME && s.duty[0]==0);
    control_init(&s,&c);control_frame_at(&s,&f,0xfffffff0u,20);CHECK(s.good_frames==1);
    control_frame_at(&s,&f,0xfffffff0u,30);CHECK(s.good_frames==1); /* Same capture is not three frames. */
    control_frame_at(&s,&f,0xffffff00u,40);CHECK(s.good_frames==0);
    puts("STEP-04 capture age/stale rejection/duplicate/wrap: PASS");
}
static void preparation_encoder_tests(void) {
    const int bad[]={-6,451};
    for(unsigned i=0;i<2;i++) {
        control_state_t s;control_config_t c=config();control_init(&s,&c);ready(&s);CHECK(control_start(&s,60));
        frame(&s,70,0);control_tick(&s,70,(int16_t)bad[i],0);
        CHECK(s.mode==CAR_FAULT && s.fault==FAULT_ENCODER && s.duty[0]==0 && s.duty[1]==0);
    }
    puts("STEP-06 encoder checks during preparation: PASS");
}
/* T2 STEP-02: compare identical feedback, verify authority to reduce PWM. */
static void tracking_bias_tests(void) {
    control_config_t a=config(),b=config();control_state_t old,boost;
    a.tracking_pwm_bias[0]=1500;
    control_init(&old,&a);control_init(&boost,&b);ready(&old);ready(&boost);
    CHECK(control_start(&old,60) && control_start(&boost,60));
    for(uint32_t t=270;t<=300;t+=10) {
        frame(&old,t,0);frame(&boost,t,0);
        control_tick(&old,t,6,6);control_tick(&boost,t,6,6);
    }
    CHECK(fabsf(boost.duty[0]-old.duty[0]-700)<.1f);
    CHECK(fabsf(boost.duty[1]-old.duty[1])<.1f);
    frame(&boost,310,0);control_tick(&boost,310,300,300);
    CHECK(boost.duty[0]<1500); /* No forced minimum when feedback exceeds target. */
    control_stop(&boost);CHECK(boost.duty[0]==0 && boost.duty[1]==0);
    control_init(&boost,&b);ready(&boost);CHECK(control_start(&boost,60));
    for(uint32_t t=270;t<=760;t+=10) {
        frame(&boost,t,0);control_drive_keepalive(&boost,t);
        control_tick(&boost,t,6,6);
        CHECK(boost.duty[0]>=0 && boost.duty[0]<=3000);
    }
    for(uint32_t t=770;t<=1260 && boost.mode==CAR_RUNNING;t+=10) {
        frame(&boost,t,0);control_drive_keepalive(&boost,t);
        control_tick(&boost,t,0,6);
    }
    CHECK(boost.mode==CAR_FAULT && boost.fault==FAULT_ENCODER);
    CHECK(boost.duty[0]==0 && boost.duty[1]==0);
    control_init(&boost,&b);ready(&boost);CHECK(control_start(&boost,60));
    boost.cfg.speed_target=0;frame(&boost,270,0);control_tick(&boost,270,0,0);
    CHECK(boost.duty[0]==0 && boost.duty[1]==0);
    puts("T2 bias isolation / overspeed reduction / stall / zero target: PASS");
}
int main(void) {tracking_bias_tests();preparation_encoder_tests();capture_age_tests();start_guard_tests();speed_tests();trial_guard_tests();return 0;}
