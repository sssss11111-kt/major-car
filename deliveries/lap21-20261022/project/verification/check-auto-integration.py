import os, shutil
"""Host-check the actual production trial_tick with hardware I/O stubs."""
from pathlib import Path
import subprocess
import re

p = Path(__file__).resolve().parent
source = (p.parent/'demo/user/src/main.c').read_text(encoding='utf-8')
trace = source[source.index('typedef struct {'):source.index('static void drive_trace_report')]
supervisor = source[source.index('volatile uint32 trial_ticks=0'):]
stubs = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "auto_start.h"
#include "pid.h"
#include "img_process.h"
typedef uint8_t uint8;
typedef uint32_t uint32;
typedef int32_t int32;
static auto_start_t auto_start_state={0};
static volatile uint8 auto_phase=0,auto_run=0,wireless_ready=1,trial_bench=0,snapshot_requested=0;
static volatile uint8 vision_mode=1,vision_degraded=0,model_good=5;
static volatile uint32 auto_center_since=0;
static const float servo_motor_duty_middle=92.5f;
static int encoder_data_1=30,encoder_data_2=30;

static const char *rx="";
static float servo_command;
static volatile float steering_command=92.5f;
static volatile float steering_speed_scale=1.0f;
static uint8 marker=0;
uint8_t img_get_stripe_active(void) {return marker;}
static int motor1,motor2;
static struct {struct {uint32 VAL1,VAL3;} SM[2];uint32 OUTEN,MASK,MCTRL,FSTS,DTSRCSEL;} pwm_regs;
static struct {uint32 DR;} gpio_regs;
#define PWM2 (&pwm_regs)
#define GPIO2 (&gpio_regs)
#define MOTOR1_DIR 1
#define MOTOR2_DIR 2
#define MOTOR1_FORWARD_DIR_LEVEL 1
#define MOTOR2_FORWARD_DIR_LEVEL 0
#define MOTOR1_PWM 1
#define MOTOR2_PWM 2
#define SERVO_MOTOR_PWM 3
#define SERVO_MOTOR_DUTY(x) (x)
static int wireless_uart_read_buffer(uint8 *ch,int n) {(void)n;if(!*rx)return 0;*ch=(uint8)*rx++;return 1;}
static void pwm_set_duty(int channel,float v) {if(channel==3)servo_command=v;else if(channel==1)motor1=(int)v;else motor2=(int)v;}
static void gpio_set_level(int a,int b) {(void)a;(void)b;}

#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
'''
publication = source[source.index('uint8 degraded ='):source.index('            vision_error =')]
publication = 'static void publish_mode(uint8 valid,uint8 detected_both) {\n' + publication + '\n}\n'
tests = r'''
static void step(void) {frame_tick=trial_ticks;trial_tick();}
int main(int argc,char **argv) {
    CHECK(argc==2);
    if (!strcmp(argv[1],"auto")) {
        wireless_ready=0; /* Offline startup must work without the radio. */
        speed_target=TRIAL_DRIVE_TARGET;
        CHECK(speed_target==45.0f);
        good_frames=5;
        step(); CHECK(auto_phase==AUTO_CENTER && !trial_running);
        CHECK(servo_command==92.5f && motor1==0 && motor2==0);
        for(int i=0;i<99;i++){step();CHECK(!trial_running && motor1==0 && motor2==0);}
        step();CHECK(trial_running && auto_run && motor1>0 && motor2>0);
        for(int i=0;i<100;i++)step();
        CHECK(trial_running && trial_fault==0); /* No K heartbeat received. */
        rx="STOP\n";step();CHECK(!trial_running && trial_fault==1 && motor1==0 && motor2==0);
        for(int i=0;i<200;i++)step();
        CHECK(!trial_running && auto_phase==AUTO_LOCKED && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"lost")) {
        good_frames=5;step();
        for(int i=0;i<98;i++)step();
        vision_mode=0;good_frames=0;step();CHECK(auto_phase==AUTO_WAIT && !trial_running);
        vision_mode=1;good_frames=5;step();
        for(int i=0;i<99;i++){step();CHECK(!trial_running);}
        step();CHECK(trial_running);
        good_frames=0;step();CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"snapshot")) {
        good_frames=5;snapshot_requested=1;
        for(int i=0;i<200;i++)step();CHECK(!trial_running && auto_phase==AUTO_WAIT);
        snapshot_requested=0;step();CHECK(auto_phase==AUTO_CENTER && !trial_running);
    } else if (!strcmp(argv[1],"stale")) {
        good_frames=5;trial_ticks=30;frame_tick=0;trial_tick();
        CHECK(!trial_running && auto_phase==AUTO_WAIT && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"bench")) {
        rx="BENCH\n";step();CHECK(trial_running && trial_bench && !auto_run);
        for(int i=0;i<50;i++)step();
        CHECK(!trial_running && trial_fault==3 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"pwm100")) {
        CHECK(TRIAL_DRIVE_MAX == 10000.0f);
        speed_target=200;good_frames=5;
        for(int i=0;i<101;i++)step();
        CHECK(trial_running && motor1==10000 && motor2==10000);
        speed_target=TRIAL_DRIVE_TARGET;encoder_data_1=150;encoder_data_2=150;
        step();CHECK(trial_running && motor1==0 && motor2==0); /* Overspeed requests no forward PWM. */
        rx="STOP\n";step();CHECK(!trial_running && trial_fault==1 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"single")) {
        good_frames=5;
        for(int i=0;i<101;i++)step();
        vision_mode=2;vision_degraded=1;
        encoder_data_1=24;encoder_data_2=24;
        step();CHECK(motor1==1500 && motor2==1500);
        for(int i=0;i<100;i++)step();
        CHECK(trial_running && trial_fault==0); /* New detector owns validity. */
        good_frames=0;step();
        CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"curve_speed")) {
        speed_target=TRIAL_DRIVE_TARGET;good_frames=5;
        for(int i=0;i<101;i++)step();
        steering_speed_scale=24.0f/45.0f;encoder_data_1=24;encoder_data_2=24;
        step();CHECK(motor1==1500 && motor2==1500);
        steering_speed_scale=1.0f;step();CHECK(motor1==2550 && motor2==2550);
        vision_degraded=1;step();CHECK(motor1==1500 && motor2==1500);
    } else if (!strcmp(argv[1],"stripe_start")) {
        marker=1;publish_mode(1,40);good_frames=5;
        CHECK(vision_mode==2 && vision_degraded);
        for(int i=0;i<200;i++)step();
        CHECK(!trial_running && auto_phase==AUTO_WAIT);
        marker=0;publish_mode(1,40);
        for(int i=0;i<101;i++)step();
        CHECK(trial_running);
    } else if (!strcmp(argv[1],"stripe_drive")) {
        good_frames=5;
        for(int i=0;i<101;i++)step();
        CHECK(trial_running);
        marker=1;publish_mode(1,40);
        encoder_data_1=24;encoder_data_2=24;step();
        CHECK(vision_degraded && motor1==1500 && motor2==1500);
        marker=0;publish_mode(1,40);step();
        CHECK(motor1==2550 && motor2==2550);
        marker=1;publish_mode(1,40);
        rx="STOP\n";step();CHECK(!trial_running && trial_fault==1 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"stripe_stale")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        marker=1;publish_mode(1,40);
        for(int i=0;i<20;i++)trial_tick();
        CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"stripe_encoder")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        marker=1;publish_mode(1,40);encoder_data_1=151;step();
        CHECK(!trial_running && trial_fault==5 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"stripe_stall")) {
        good_frames=5;for(int i=0;i<140;i++)step();
        marker=1;publish_mode(1,40);encoder_data_1=0;
        for(int i=0;i<20;i++)step();
        CHECK(!trial_running && trial_fault==6 && motor1==0 && motor2==0);
    } else return 2;
    printf("PASS: actual trial_tick %s\n",argv[1]);return 0;
}
'''
generated = p/'auto-integration-generated.c'
header = (p.parent/'demo/user/inc/main.h').read_text(encoding='utf-8')
drive_defines = '\n'.join(re.findall(r'^#define TRIAL_DRIVE_.*$', header, flags=re.M))
generated.write_text(stubs+drive_defines+'\n'+trace+supervisor+publication+tests,encoding='utf-8')
zig = (os.environ.get('ZIG') or shutil.which('zig') or r'D:\major-car\firmware\camera_car\.tools\ziglang\zig.exe')
subprocess.run([zig,'cc','-Wall','-Wextra','-Werror','-I',str(p/'vision-stubs'),'-I',str(p.parent/'demo/user/inc'),str(generated),str(p.parent/'demo/user/src/pid.c'),
                str(p.parent/'demo/user/src/auto_start.c'),'-o',str(p/'auto-integration.exe')],check=True)
for case in ['auto','lost','snapshot','stale','bench','pwm100','single','curve_speed',
             'stripe_start','stripe_drive','stripe_stale','stripe_encoder','stripe_stall']:
    subprocess.run([str(p/'auto-integration.exe'),case],check=True)
