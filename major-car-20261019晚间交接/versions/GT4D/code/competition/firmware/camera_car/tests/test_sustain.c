#include "car_core.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL sustain %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static control_state_t fresh(void){control_state_t s;control_config_t c={0};c.servo_center_us=1500;c.heartbeat_timeout_ms=100;control_init(&s,&c);return s;}
static void step(control_state_t *s,uint32_t t,int active,int other){s->heartbeat_ms=t;control_drive_keepalive(s,t);control_tick(s,t,s->test_motor==1?active:other,s->test_motor==2?active:other);}
static void kick(control_state_t *s){for(unsigned t=10;t<=200;t+=10)step(s,t,20,0);CHECK(s->sustain_phase==2);}
int main(void){
    for(unsigned motor=1;motor<=2;motor++){
        control_state_t s=fresh();CHECK(control_sustain_test(&s,motor,1800,0));
        for(unsigned t=10;t<200;t+=10){step(&s,t,20,0);CHECK(s.sustain_phase==1 && s.duty[motor-1]==3000 && s.duty[2-motor]==0);}
        step(&s,200,20,0);CHECK(s.sustain_phase==2 && s.duty[motor-1]==1800);
        CHECK(!control_sustain_test(&s,3-motor,2000,200));
        for(unsigned t=210;t<3200;t+=10)step(&s,t,20,0);
        step(&s,3200,20,0);CHECK(s.mode==CAR_IDLE && s.sustain_result==1 && s.sustain_phase==3);
        CHECK(s.sustain_n==200 && s.sustain_sum==4000 && s.sustain_min==20 && s.sustain_max==20 && s.sustain_zeros==0);
        CHECK(s.duty[0]==0 && s.duty[1]==0 && s.sustain_hold_ms==3000 && s.sustain_total_ms==3200);
        unsigned id=s.sustain_id;control_stop(&s);CHECK(s.sustain_result==0 && s.sustain_phase==0);
        CHECK(control_sustain_test(&s,motor,0,3200) && s.sustain_id!=id && s.sustain_n==0);
    }
    control_state_t s=fresh();CHECK(control_sustain_test(&s,1,0,0));kick(&s);CHECK(s.duty[0]==0);
    for(unsigned t=210;t<=700;t+=10)step(&s,t,0,0);
    CHECK(s.mode==CAR_IDLE && s.sustain_result==2 && s.sustain_phase==3 && s.duty[0]==0);
    s=fresh();CHECK(control_sustain_test(&s,1,2000,0));
    for(unsigned t=10;t<=300;t+=10)step(&s,t,t==10?10:0,0);
    CHECK(s.mode==CAR_FAULT && s.fault==FAULT_ENCODER && s.sustain_result==0);
    for(unsigned bad=0;bad<3;bad++){
        s=fresh();CHECK(control_sustain_test(&s,2,2000,0));step(&s,10,bad==0?-6:bad==1?1501:20,bad==2?11:0);
        CHECK(s.fault==FAULT_ENCODER && s.duty[0]==0 && s.duty[1]==0);
    }
    s=fresh();CHECK(!control_sustain_test(&s,1,3001,0) && !control_sustain_test(&s,0,1000,0));
    s.cfg.simulated=1;CHECK(!control_sustain_test(&s,1,1000,0));
    s=fresh();CHECK(control_sustain_test(&s,1,1000,0));s.heartbeat_ms=500;control_tick(&s,500,0,0);CHECK(s.fault==FAULT_LINK);
    s=fresh();CHECK(control_sustain_test(&s,1,1000,0));control_tick(&s,110,0,0);CHECK(s.fault==FAULT_FOREGROUND);
    s=fresh();CHECK(control_sustain_test(&s,1,1000,0));kick(&s);
    for(unsigned t=210;t<=3200;t+=10)step(&s,t,t==1500?0:20,0);
    CHECK(s.sustain_result==1 && s.sustain_n==200 && s.sustain_sum==3980 && s.sustain_min==0 && s.sustain_zeros==1);
    s=fresh();s.heartbeat_ms=0xfffffff0u;CHECK(control_sustain_test(&s,2,1000,0xfffffff0u));
    for(unsigned dt=10;dt<=200;dt+=10)step(&s,0xfffffff0u+dt,20,0);
    CHECK(s.sustain_phase==2 && s.duty[1]==1000);control_stop(&s);CHECK(s.sustain_result==0 && s.duty[1]==0);
    puts("Sustain: kick gate / drop / full-window stats / lost motion / isolation / guards / IDs / wrap PASS");return 0;
}
