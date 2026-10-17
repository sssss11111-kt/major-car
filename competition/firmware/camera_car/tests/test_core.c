#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "car_core.h"
#undef assert
#define assert(expr) do { if (!(expr)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#expr); exit(1); } } while(0)
static uint8_t image[CAR_W*CAR_H], binary[CAR_W*CAR_H];
static vision_state_t vs;
static vision_config_t vc;
static vision_result_t vr;
/* Hand-set boundaries: 50 and 138 => center 94, image center 93.5. */
static void track(int shift, int dark, int light) {
    for (int y=0;y<CAR_H;y++) for(int x=0;x<CAR_W;x++)
        image[y*CAR_W+x]=(uint8_t)(x>=50+shift && x<=138+shift?light:dark);
}
static void vision_tests(void) {
    vision_config_default(&vc); vision_reset(&vs);
    track(0,30,210); vision_process(&vs,&vc,image,binary,&vr);
    assert(vr.valid && vr.paired_rows>=8 && fabsf(vr.error)<1.0f);
    vision_reset(&vs); track(18,30,210);
    vision_process(&vs,&vc,image,binary,&vr);
    assert(vr.valid && vr.error>17 && vr.error<20);
    vision_reset(&vs); track(-18,30,210);
    vision_process(&vs,&vc,image,binary,&vr);
    assert(vr.valid && vr.error< -16);
    vision_reset(&vs); track(0,10,70);
    vision_process(&vs,&vc,image,binary,&vr);
    assert(vr.valid && fabsf(vr.error)<1);
    vc.otsu=0; vc.fixed_threshold=100; track(0,30,210);
    vision_process(&vs,&vc,image,binary,&vr); assert(vr.valid);
    vc.white_road=0; track(0,210,30);
    vision_process(&vs,&vc,image,binary,&vr); assert(vr.valid);
    vision_config_default(&vc);
    for(int v=0;v<=255;v+=255) {
        memset(image,v,sizeof image); vision_process(&vs,&vc,image,binary,&vr);
        assert(!vr.valid && vr.paired_rows==0);
    }
    track(0,110,118); vision_process(&vs,&vc,image,binary,&vr); assert(!vr.valid);
    vision_reset(&vs); track(0,30,210);
    vision_process(&vs,&vc,image,binary,&vr);
    for(int y=0;y<CAR_H;y++) for(int x=138;x<CAR_W;x++) image[y*CAR_W+x]=210;
    vision_process(&vs,&vc,image,binary,&vr);
    assert(vr.valid_rows>0 && vr.paired_rows==0 && !vr.valid);
    /* Noise isolated to one pixel must not become a false black boundary. */
    vision_reset(&vs); track(0,30,210);
    for(int y=0;y<CAR_H;y++) image[y*CAR_W+85]=30;
    vision_process(&vs,&vc,image,binary,&vr);
    assert(vr.valid && fabsf(vr.error)<1);
}
static control_config_t config(void) {
    control_config_t c={0};
    c.speed_target=20; c.speed_kp=40; c.speed_ki=100; c.pwm_max=2000;
    c.steer_kp=4; c.steer_kd=0.1f; c.steer_sign=1;
    c.servo_center_us=1500; c.servo_min_us=1200; c.servo_max_us=1800;
    c.frame_timeout_ms=150; c.heartbeat_timeout_ms=100;
    return c;
}
static void feed_good(control_state_t *c, uint32_t now) {
    vision_result_t f={0}; f.valid=1; f.paired_rows=12; f.scanned_rows=12;
    control_frame(c,&f,now-40); control_frame(c,&f,now-20); control_frame(c,&f,now);
    c->heartbeat_ms=now;
}
static void control_tests(void) {
    control_state_t c; control_config_t cfg=config();
    control_init(&c,&cfg); control_tick(&c,10,0,0);
    assert(c.duty[0]==0 && c.duty[1]==0 && !control_start(&c,10));
    feed_good(&c,100); assert(control_start(&c,100));
    control_tick(&c,110,0,0); assert(c.duty[0]>0 && c.duty[0]<=2000);
    c.heartbeat_ms=260; control_tick(&c,260,0,0);
    assert(c.mode==CAR_FAULT && c.fault==FAULT_FRAME && c.duty[0]==0);
    /* Old validity must not authorize restart after fault acknowledgement. */
    control_stop(&c); assert(!control_start(&c,260));
    control_init(&c,&cfg); feed_good(&c,100); assert(control_start(&c,100));
    control_tick(&c,201,0,0); assert(c.fault==FAULT_FOREGROUND);
    control_init(&c,&cfg); feed_good(&c,100); assert(control_start(&c,100));
    vision_result_t bad={0};
    control_frame(&c,&bad,120); control_frame(&c,&bad,140); control_frame(&c,&bad,160);
    assert(c.mode==CAR_FAULT && c.fault==FAULT_LINE && c.duty[1]==0);
    control_init(&c,&cfg); feed_good(&c,100); assert(control_start(&c,100));
    for(uint32_t t=110;t<30000;t+=10) {
        feed_good(&c,t); control_tick(&c,t,10,10);
        assert(c.duty[0]<=2000 && fabsf(c.integral[0])<=2000);
    }
    /* Recovered feedback must release saturation promptly. */
    feed_good(&c,30000); control_tick(&c,30000,100,100); assert(c.duty[0]==0);
    control_init(&c,&cfg); feed_good(&c,0xfffffff0u);
    assert(control_start(&c,0xfffffff0u));
    c.heartbeat_ms=20; control_tick(&c,20,0,0); assert(c.mode==CAR_RUNNING);
    c.heartbeat_ms=200; control_tick(&c,200,0,0); assert(c.fault==FAULT_FRAME);
    cfg.simulated=1; control_init(&c,&cfg); feed_good(&c,100);
    assert(!control_start(&c,100) && !control_test(&c,1,100));
    cfg.simulated=0; control_init(&c,&cfg); c.heartbeat_ms=100;
    assert(control_test(&c,1,100)); control_tick(&c,110,0,0);
    assert(c.duty[0]==3000 && c.duty[1]==0);
    for(uint32_t t=120;t<2100;t+=10) {
        c.heartbeat_ms=t; control_tick(&c,t,0,0);
        assert(c.mode==CAR_TEST && c.duty[0]==3000 && c.duty[1]==0);
    }
    c.heartbeat_ms=2100; control_tick(&c,2100,0,0);
    assert(c.mode==CAR_IDLE && c.duty[0]==0 && c.duty[1]==0);
    /* Immediate stop and foreground watchdog still override the longer test. */
    assert(control_test(&c,1,2200)); c.heartbeat_ms=2200;
    control_tick(&c,2210,0,0); control_stop(&c);
    control_tick(&c,2220,0,0);
    assert(c.mode==CAR_IDLE && c.duty[0]==0 && c.duty[1]==0);
    assert(control_test(&c,1,2300)); c.heartbeat_ms=2300;
    control_tick(&c,2401,0,0);
    assert(c.mode==CAR_FAULT && c.fault==FAULT_FOREGROUND && c.duty[0]==0);
    control_stop(&c); c.heartbeat_ms=2500;
    assert(control_test(&c,2,2500)); control_tick(&c,2510,0,0);
    assert(c.duty[0]==0 && c.duty[1]==3000);
    for(uint32_t t=2520;t<4500;t+=10) {
        c.heartbeat_ms=t; control_tick(&c,t,0,0);
        assert(c.mode==CAR_TEST && c.duty[0]==0 && c.duty[1]==3000);
    }
    c.heartbeat_ms=4500; control_tick(&c,4500,0,0);
    assert(c.mode==CAR_IDLE && c.duty[0]==0 && c.duty[1]==0);
    assert(control_test(&c,2,4600)); c.heartbeat_ms=4600;
    control_tick(&c,4610,0,0); control_stop(&c);
    control_tick(&c,4620,0,0);
    assert(c.mode==CAR_IDLE && c.duty[0]==0 && c.duty[1]==0);
    assert(control_test(&c,2,4700)); c.heartbeat_ms=4700;
    control_tick(&c,4801,0,0);
    assert(c.mode==CAR_FAULT && c.fault==FAULT_FOREGROUND && c.duty[1]==0);
    control_init(&c,&cfg); feed_good(&c,100); assert(control_start(&c,100));
    vision_result_t f={0}; f.valid=1; f.error=1000;
    control_frame(&c,&f,120); assert(c.servo_us<=1800 && c.servo_us>=1200);
    control_stop(&c); assert(c.duty[0]==0 && c.servo_us==1500);
    control_init(&c,&cfg); feed_good(&c,100); assert(control_start(&c,100));
    for(uint32_t t=110;t<2500 && c.mode==CAR_RUNNING;t+=10) {
        feed_good(&c,t); control_tick(&c,t,0,0);
    }
    assert(c.mode==CAR_FAULT && c.fault==FAULT_ENCODER && c.duty[0]==0);
    /* A gap cannot be hidden by a single newly arrived frame. */
    control_init(&c,&cfg); feed_good(&c,100);
    control_frame(&c,&f,400); c.heartbeat_ms=400; assert(!control_start(&c,400));
    puts("control tests: PASS");
}
int main(int argc, char **argv) {
    if(argc>1 && strcmp(argv[1],"control")==0) { control_tests(); return 0; }
    vision_tests(); puts("vision tests: PASS"); control_tests(); return 0;
}
