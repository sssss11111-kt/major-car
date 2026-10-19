#include "car_core.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL calibration line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static control_state_t fresh(void) {
    control_state_t s;control_config_t c={0};
    c.servo_center_us=1500;c.heartbeat_timeout_ms=100;
    control_init(&s,&c);return s;
}
static void tick(control_state_t *s,unsigned t,int l,int r) {
    s->heartbeat_ms=t;control_drive_keepalive(s,t);control_tick(s,t,l,r);
}
int main(void) {
    for(unsigned motor=1;motor<=2;motor++)for(unsigned duty=0;duty<=7000;duty+=500) {
        control_state_t s=fresh();CHECK(control_calibration_test(&s,motor,duty,0));
        for(unsigned t=10;t<2000;t+=10) {
            tick(&s,t,0,0);CHECK(s.mode==CAR_CALIBRATION);
            CHECK(s.duty[motor-1]==duty && s.duty[2-motor]==0);
            CHECK(s.servo_us==1500);
        }
        tick(&s,2000,0,0);CHECK(s.mode==CAR_IDLE && s.duty[0]==0 && s.duty[1]==0);
    }
    control_state_t s=fresh();
    CHECK(!control_calibration_test(&s,0,500,0));CHECK(!control_calibration_test(&s,3,500,0));
    CHECK(!control_calibration_test(&s,1,7001,0));
    s.cfg.simulated=1;CHECK(!control_calibration_test(&s,1,500,0));
    s=fresh();CHECK(control_calibration_test(&s,1,1500,0));
    CHECK(!control_calibration_test(&s,2,2000,0));
    s.heartbeat_ms=500;control_tick(&s,500,0,0);
    CHECK(s.mode==CAR_FAULT && s.fault==FAULT_LINK && s.duty[0]==0);
    s=fresh();CHECK(control_calibration_test(&s,1,3000,0));
    control_tick(&s,110,0,0);CHECK(s.fault==FAULT_FOREGROUND && s.duty[0]==0);
    for(unsigned bad=0;bad<3;bad++) {
        s=fresh();CHECK(control_calibration_test(&s,1,3000,0));
        tick(&s,10,bad==0?-6:(bad==1?451:0),bad==2?11:0);
        CHECK(s.mode==CAR_FAULT && s.fault==FAULT_ENCODER && s.duty[0]==0 && s.duty[1]==0);
    }
    s=fresh();CHECK(control_calibration_test(&s,1,3000,0));tick(&s,10,200,0);
    control_stop(&s);tick(&s,20,0,0);CHECK(s.mode==CAR_IDLE && s.duty[0]==0);
    s=fresh();s.heartbeat_ms=0xfffffff0u;CHECK(control_calibration_test(&s,2,3000,0xfffffff0u));
    tick(&s,0x10u,0,200);CHECK(s.mode==CAR_CALIBRATION);
    puts("Calibration: range / isolation / expiry / lease / heartbeat / feedback / stop / wrap PASS");return 0;
}
