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
static void speed_bench_tests(void) {
    control_state_t s; control_config_t cfg=config();
    control_init(&s,&cfg); s.heartbeat_ms=100;
    assert(control_speed_test_right(&s,100));
    assert(!control_speed_test_right(&s,100));
    control_tick(&s,110,0,0);
    assert(s.mode==CAR_SPEED_TEST && s.duty[0]==0 && s.duty[1]==3000);
    s.heartbeat_ms=120; control_tick(&s,120,0,30);
    assert(s.bench_started && s.duty[0]==0 && s.duty[1]<3000);
    /* Simulated dead-zone motor: sanity check only, not physical tuning proof. */
    float speed=30;
    for(uint32_t t=130;t<4100;t+=10) {
        float equilibrium=s.duty[1]>1200?(s.duty[1]-1200)*0.18f:0;
        speed+=(equilibrium-speed)*0.125f;
        s.heartbeat_ms=t; control_tick(&s,t,0,(int16_t)speed);
        assert(s.mode==CAR_SPEED_TEST && s.duty[0]==0 && s.duty[1]>=0 && s.duty[1]<=3000);
    }
    assert(s.bench_target==80 && speed>60 && speed<100);
    s.heartbeat_ms=4100; control_tick(&s,4100,0,(int16_t)speed);
    assert(s.mode==CAR_IDLE && s.duty[0]==0 && s.duty[1]==0);
    /* Startup without feedback cannot continue after 200ms. */
    control_init(&s,&cfg); assert(control_speed_test_right(&s,0));
    for(uint32_t t=10;t<=200;t+=10){s.heartbeat_ms=t;control_tick(&s,t,0,0);}
    assert(s.mode==CAR_FAULT && s.fault==FAULT_ENCODER && s.duty[1]==0);
    control_init(&s,&cfg); assert(control_speed_test_right(&s,0));
    s.heartbeat_ms=10;control_tick(&s,10,0,-6);
    assert(s.mode==CAR_FAULT && s.duty[1]==0);
    control_init(&s,&cfg); assert(control_speed_test_right(&s,0));
    s.heartbeat_ms=10;control_tick(&s,10,0,451);
    assert(s.mode==CAR_FAULT && s.duty[1]==0);
    control_init(&s,&cfg); assert(control_speed_test_right(&s,0));
    s.heartbeat_ms=10;control_tick(&s,10,11,30);
    assert(s.mode==CAR_FAULT && s.duty[1]==0);
    control_init(&s,&cfg); assert(control_speed_test_right(&s,0));
    control_tick(&s,101,0,0);assert(s.fault==FAULT_FOREGROUND && s.duty[1]==0);
    control_init(&s,&cfg); assert(control_speed_test_right(&s,0));
    s.heartbeat_ms=10;control_tick(&s,10,0,0);control_stop(&s);
    control_tick(&s,20,0,0);assert(s.mode==CAR_IDLE && s.duty[1]==0);
    cfg.simulated=1;control_init(&s,&cfg);assert(!control_speed_test_right(&s,0));
    puts("right speed bench guards/model: PASS (model is not hardware validation)");
}
static void dual_speed_tests(void) {
    control_state_t s; control_config_t cfg=config();
    control_init(&s,&cfg);assert(control_speed_test(&s,3,0));
    s.heartbeat_ms=10;control_tick(&s,10,0,0);
    assert(s.duty[0]==3000 && s.duty[1]==3000);
    float l=30,r=35;
    for(uint32_t t=20;t<4000;t+=10) {
        if(t>20){
            float le=s.duty[0]>1200?(s.duty[0]-1200)*0.16f:0;
            float re=s.duty[1]>1200?(s.duty[1]-1200)*0.18f:0;
            l+=(le-l)*0.125f;r+=(re-r)*0.125f;
        }
        s.heartbeat_ms=t;control_tick(&s,t,(int16_t)l,(int16_t)r);
        assert(s.mode==CAR_SPEED_TEST && s.duty[0]>=0 && s.duty[0]<=3000 && s.duty[1]>=0 && s.duty[1]<=3000);
    }
    assert(l>60 && l<100 && r>60 && r<100 && fabsf(l-r)<10);
    s.heartbeat_ms=4000;control_tick(&s,4000,(int16_t)l,(int16_t)r);
    assert(s.mode==CAR_IDLE && s.duty[0]==0 && s.duty[1]==0);
    for(int side=0;side<2;side++) {
        control_init(&s,&cfg);assert(control_speed_test(&s,3,0));
        s.heartbeat_ms=10;control_tick(&s,10,side==0?-6:30,side==1?-6:30);
        assert(s.fault==FAULT_ENCODER && s.duty[0]==0 && s.duty[1]==0);
        control_init(&s,&cfg);assert(control_speed_test(&s,3,0));
        s.heartbeat_ms=10;control_tick(&s,10,side==0?451:30,side==1?451:30);
        assert(s.fault==FAULT_ENCODER && s.duty[0]==0 && s.duty[1]==0);
        control_init(&s,&cfg);assert(control_speed_test(&s,3,0));
        for(uint32_t t=10;t<=200;t+=10){s.heartbeat_ms=t;control_tick(&s,t,side==0?0:30,side==1?0:30);}
        assert(s.fault==FAULT_ENCODER && s.duty[0]==0 && s.duty[1]==0);
        /* One wheel loses feedback after both started: stop both outputs. */
        control_init(&s,&cfg);assert(control_speed_test(&s,3,0));
        s.heartbeat_ms=10;control_tick(&s,10,30,30);
        assert(s.bench_started==3);
        for(uint32_t t=20;t<=510;t+=10){s.heartbeat_ms=t;control_tick(&s,t,side==0?0:80,side==1?0:80);}
        assert(s.fault==FAULT_ENCODER && s.duty[0]==0 && s.duty[1]==0);
    }
    control_init(&s,&cfg);assert(control_speed_test(&s,3,0));
    control_tick(&s,101,0,0);assert(s.fault==FAULT_FOREGROUND && s.duty[0]==0 && s.duty[1]==0);
    control_init(&s,&cfg);assert(control_speed_test(&s,3,0));
    s.heartbeat_ms=10;control_tick(&s,10,0,0);control_stop(&s);
    control_tick(&s,20,0,0);assert(s.mode==CAR_IDLE && s.duty[0]==0 && s.duty[1]==0);
    control_init(&s,&cfg);assert(control_speed_test(&s,3,0xfffffff0u));
    s.heartbeat_ms=20;control_tick(&s,20,30,30);assert(s.mode==CAR_SPEED_TEST);
    s.heartbeat_ms=3984;control_tick(&s,3984,80,80);assert(s.mode==CAR_IDLE);
    puts("dual speed bench guards/model: PASS (model is not hardware validation)");
}
/* Ground mode must work with no vision frames and stop independently of vision. */
extern int control_drive_test(control_state_t *,int,uint32_t);
static void long_drive_tests(void) {
    control_state_t s;control_config_t cfg=config();vision_result_t f={0};
    control_init(&s,&cfg);assert(control_drive_long_test(&s,0));
    assert(!control_drive_long_test(&s,0));
    for(uint32_t t=10;t<10200;t+=10) {
        s.heartbeat_ms=t;if(t%100==0)control_drive_keepalive(&s,t);
        control_frame(&s,&f,t);control_tick(&s,t,t<210?0:40,t<210?0:40);
        assert(s.mode==CAR_DRIVE_TEST && s.fault==FAULT_NONE && s.servo_us==cfg.servo_center_us);
        assert(s.duty[0]>=0 && s.duty[0]<=3000 && s.duty[1]>=0 && s.duty[1]<=3000);
    }
    s.heartbeat_ms=10200;control_tick(&s,10200,40,40);
    assert(s.mode==CAR_IDLE && s.duty[0]==0 && s.duty[1]==0 && s.servo_us==cfg.servo_center_us);
    control_init(&s,&cfg);assert(control_drive_long_test(&s,0));
    for(uint32_t t=10;t<=500;t+=10){s.heartbeat_ms=t;control_tick(&s,t,t<210?0:40,t<210?0:40);}
    assert(s.mode==CAR_FAULT && s.fault==FAULT_LINK && s.duty[0]==0 && s.duty[1]==0);
    control_drive_keepalive(&s,510);assert(s.mode==CAR_FAULT);
    control_stop(&s);assert(control_drive_long_test(&s,510));control_stop(&s);
    control_drive_keepalive(&s,520);assert(s.mode==CAR_IDLE && s.duty[0]==0);
    cfg.simulated=1;control_init(&s,&cfg);assert(!control_drive_long_test(&s,0));
    puts("10s drive timeout, lease, stop, no-vision guards: PASS");
}
static void drive_tests(void) {
    control_state_t s;control_config_t cfg=config();vision_result_t f={0};
    for(int turn=-1;turn<=1;turn++) {
        control_init(&s,&cfg);
        assert(!control_drive_test(&s,2,0));
        assert(control_drive_test(&s,turn,0));
        float expected=turn<0?cfg.servo_max_us:(turn>0?cfg.servo_min_us:cfg.servo_center_us);
        assert(s.servo_us==expected && s.duty[0]==0 && s.duty[1]==0);
        assert(!control_test(&s,1,0) && !control_start(&s,0) && !control_speed_test(&s,3,0));
        for(uint32_t t=10;t<=1190;t+=10) {
            s.heartbeat_ms=t;control_frame(&s,&f,t);
            control_tick(&s,t,t<210?0:40,t<210?0:40);
            assert(s.mode==CAR_DRIVE_TEST && s.servo_us==expected);
            assert(s.duty[0]>=0 && s.duty[0]<=3000 && s.duty[1]>=0 && s.duty[1]<=3000);
            if(t<200)assert(s.duty[0]==0 && s.duty[1]==0);
            else assert(s.duty[0]>0 && s.duty[1]>0);
        }
        s.heartbeat_ms=1200;control_tick(&s,1200,40,40);
        assert(s.mode==CAR_IDLE && s.duty[0]==0 && s.duty[1]==0 && s.servo_us==cfg.servo_center_us);
    }
    control_init(&s,&cfg);assert(control_drive_test(&s,0,0));
    for(uint32_t t=10;t<=400;t+=10){s.heartbeat_ms=t;control_tick(&s,t,0,0);}
    assert(s.fault==FAULT_ENCODER && s.duty[0]==0 && s.duty[1]==0);
    control_stop(&s);assert(control_drive_test(&s,-1,410));s.heartbeat_ms=610;
    control_tick(&s,610,-6,30);assert(s.fault==FAULT_ENCODER && s.servo_us==cfg.servo_center_us);
    control_stop(&s);assert(control_drive_test(&s,1,620));
    control_tick(&s,820,40,40);assert(s.fault==FAULT_FOREGROUND && s.duty[0]==0 && s.duty[1]==0);
    control_stop(&s);s.heartbeat_ms=1000;assert(control_drive_test(&s,1,1000));
    control_stop(&s);assert(s.mode==CAR_IDLE && s.servo_us==cfg.servo_center_us);
    control_init(&s,&cfg);s.heartbeat_ms=0xfffffff0u;
    assert(control_drive_test(&s,0,0xfffffff0u));
    s.heartbeat_ms=4;control_tick(&s,4,0,0);assert(s.mode==CAR_DRIVE_TEST && s.duty[0]==0);
    s.heartbeat_ms=1184;control_tick(&s,1184,40,40);assert(s.mode==CAR_IDLE);
    cfg.simulated=1;control_init(&s,&cfg);assert(!control_drive_test(&s,0,0));
    /* Reproduce loaded turn: inner wheel moves at 16, outer at 28, both below/around old threshold. */
    cfg.simulated=0;control_init(&s,&cfg);assert(control_drive_test(&s,-1,0));
    for(uint32_t t=10;t<1200;t+=10) {
        s.heartbeat_ms=t;control_tick(&s,t,t<250?0:16,t<250?0:28);
        assert(s.mode==CAR_DRIVE_TEST && s.fault==FAULT_NONE);
    }
    assert(s.bench_started==3);
    s.heartbeat_ms=1200;control_tick(&s,1200,16,28);assert(s.mode==CAR_IDLE);
    /* Steering-induced low encoder counts alone must not pass startup. */
    control_init(&s,&cfg);assert(control_drive_test(&s,1,0));
    for(uint32_t t=10;t<=400;t+=10){s.heartbeat_ms=t;control_tick(&s,t,4,4);}
    assert(s.fault==FAULT_ENCODER && s.duty[0]==0 && s.duty[1]==0);
    puts("ground drive guards/no-vision/timeout: PASS");
}
static void static_steer_tests(void) {
    control_state_t s; control_config_t cfg=config();
    vision_result_t f={0}; f.valid=1; f.error=20;
    control_init(&s,&cfg); assert(!control_steer_test(&s,0));
    for(uint32_t t=10;t<=30;t+=10) {s.heartbeat_ms=t;control_frame(&s,&f,t);}
    assert(control_steer_test(&s,30));
    f.error=25;control_frame(&s,&f,40);s.heartbeat_ms=40;
    control_tick(&s,40,0,0);
    assert(s.mode==CAR_STEER_TEST && s.duty[0]==0 && s.duty[1]==0);
    assert(s.servo_us!=cfg.servo_center_us);
    assert(!control_start(&s,40) && !control_test(&s,1,40) && !control_speed_test(&s,3,40));
    for(uint32_t t=50;t<=30030;t+=10) {
        s.heartbeat_ms=t;control_frame(&s,&f,t);control_tick(&s,t,0,0);
        assert(s.duty[0]==0 && s.duty[1]==0);
    }
    assert(s.mode==CAR_IDLE && s.servo_us==cfg.servo_center_us);
    for(uint32_t t=30040;t<=30060;t+=10){s.heartbeat_ms=t;control_frame(&s,&f,t);}
    assert(control_steer_test(&s,30060));
    f.valid=0;for(uint32_t t=30070;t<=30090;t+=10)control_frame(&s,&f,t);
    assert(s.fault==FAULT_LINE && s.servo_us==cfg.servo_center_us && s.duty[0]==0 && s.duty[1]==0);
    puts("static steering guards: PASS");
}
int main(int argc, char **argv) {
    long_drive_tests();
    drive_tests();
    static_steer_tests();
    dual_speed_tests();
    control_state_t l; control_config_t lc=config();
    control_init(&l,&lc);
    assert(!control_speed_test(&l,0,0) && !control_speed_test(&l,4,0));
    assert(control_speed_test(&l,1,0));
    l.heartbeat_ms=10;control_tick(&l,10,0,0);
    assert(l.duty[0]==3000 && l.duty[1]==0);
    float left_speed=30;
    for(uint32_t t=20;t<4000;t+=10) {
        if(t>20){float eq=l.duty[0]>1200?(l.duty[0]-1200)*0.18f:0;left_speed+=(eq-left_speed)*0.125f;}
        l.heartbeat_ms=t;control_tick(&l,t,(int16_t)left_speed,0);
        assert(l.mode==CAR_SPEED_TEST && l.duty[1]==0 && l.duty[0]>=0 && l.duty[0]<=3000);
    }
    assert(left_speed>60 && left_speed<100);
    l.heartbeat_ms=4000;control_tick(&l,4000,(int16_t)left_speed,0);
    assert(l.mode==CAR_IDLE && l.duty[0]==0 && l.duty[1]==0);
    control_init(&l,&lc);assert(control_speed_test(&l,1,0));
    l.heartbeat_ms=10;control_tick(&l,10,-6,0);assert(l.fault==FAULT_ENCODER && l.duty[0]==0);
    control_init(&l,&lc);assert(control_speed_test(&l,1,0));
    l.heartbeat_ms=10;control_tick(&l,10,451,0);assert(l.fault==FAULT_ENCODER && l.duty[0]==0);
    control_init(&l,&lc);assert(control_speed_test(&l,1,0));
    l.heartbeat_ms=10;control_tick(&l,10,30,11);assert(l.fault==FAULT_ENCODER && l.duty[0]==0);
    control_init(&l,&lc);assert(control_speed_test(&l,1,0));
    for(uint32_t t=10;t<=200;t+=10){l.heartbeat_ms=t;control_tick(&l,t,0,0);}
    assert(l.fault==FAULT_ENCODER && l.duty[0]==0);
    speed_bench_tests();
    if(argc>1 && strcmp(argv[1],"control")==0) { control_tests(); return 0; }
    vision_tests(); puts("vision tests: PASS"); control_tests(); return 0;
}
