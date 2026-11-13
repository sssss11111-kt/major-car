import os, shutil
"""Host-check the actual production trial_tick with hardware I/O stubs."""
from pathlib import Path
import subprocess
import re

p = Path(__file__).resolve().parent
source = (p.parent/'demo/user/src/main.c').read_text(encoding='utf-8')
# Locate the drive trace by its own type, not the first unrelated typedef.
trace_end = source.index('static void drive_trace_report')
trace_start = source.rfind('typedef struct {', 0, source.index('} drive_trace_t;'))
trace = source[trace_start:trace_end]
supervisor = source[source.index('volatile uint32 trial_ticks=0'):]
stubs = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "auto_start.h"
#include "roundabout_control.h"
#include "entry_bridge.h"
#include "pid.h"
#include "steering_control.h"
#include "cross_drive.h"
typedef uint8_t uint8;
typedef uint32_t uint32;
typedef int32_t int32;
static auto_start_t auto_start_state={0};
static volatile uint8 auto_phase=0,auto_run=0,wireless_ready=1,trial_bench=0,snapshot_requested=0,rd29_requested=0,sf_view_mode=0;
static volatile uint8 vision_mode=1,vision_degraded=0,model_good=5;
static volatile uint8 vision_stripe=0,vision_ring=0,vision_entry_kind=0,vision_cross_approach=0;
static volatile entry_bridge_guard_t vision_bridge={0};
static entry_bridge_guard_t mock_bridge={0};
uint8_t img_get_entry_status(void) {return vision_entry_kind;}
const entry_bridge_guard_t *img_get_entry_bridge_guard(void) {return &mock_bridge;}
static volatile rb_state_t roundabout_state={0};
uint8_t img_get_roundabout_candidate(void){return 0;}
uint8_t img_get_near_valid(void){return 1;}
static uint8 marker=0;
uint8_t img_get_stripe_active(void) {return marker;}
static volatile uint32 auto_center_since=0;
static const float servo_motor_duty_middle=92.5f;
static int encoder_data_1=30,encoder_data_2=30;

static const char *rx="";
static float servo_command;
static volatile float steering_command=92.5f;
static volatile float steering_speed_scale=1.0f;
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
static uint32 debug_read_ring_buffer(uint8 *ch,uint32 n) {(void)ch;(void)n;return 0;}
static void pwm_set_duty(int channel,float v) {if(channel==3)servo_command=v;else if(channel==1)motor1=(int)v;else motor2=(int)v;}
static void gpio_set_level(int a,int b) {(void)a;(void)b;}

#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
'''
stubs += source[source.index('/* FIRST_FAULT4_BEGIN'):source.index('/* FIRST_FAULT4_END */')] + '\nstatic uint8 usable_rows=0;\n'
publication = source[source.index('uint8 degraded ='):source.index('            vision_error =')]
publication = ('static cross_drive_output_t cross_path;\n'
               'static cross_drive_t cross;\n'
               'static special_track_output_t scan;\n'
               'static void publish_mode(uint8 valid,uint8 detected_both) {\n' + publication + '\n}\n')
tests = r'''
static void step(void) {frame_tick=trial_ticks;trial_tick();}
int main(int argc,char **argv) {
    CHECK(argc==2);
    if (!strcmp(argv[1],"auto")) {
        wireless_ready=0; /* Offline startup must work without the radio. */
        speed_target=TRIAL_DRIVE_TARGET;
        CHECK(speed_target==100.0f);
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
    } else if (!strcmp(argv[1],"view")) {
        good_frames=5;rx="WIMG2\n";step();
        CHECK(sf_view_mode==2 && !trial_running);
        for(int i=0;i<200;i++)step();
        CHECK(!trial_running && motor1==0 && motor2==0);
        rx="BENCH\n";step();CHECK(!trial_running && !trial_bench);
        rx="IMGOFF\n";step();CHECK(sf_view_mode==0);
        for(int i=0;i<101;i++)step();CHECK(trial_running);
        rx="WIMG2\n";step();CHECK(trial_running && sf_view_mode==0);
    } else if (!strcmp(argv[1],"lost")) {
        good_frames=5;step();
        for(int i=0;i<98;i++)step();
        vision_mode=0;good_frames=0;step();CHECK(auto_phase==AUTO_WAIT && !trial_running);
        vision_mode=1;good_frames=5;step();
        for(int i=0;i<99;i++){step();CHECK(!trial_running);}
        step();CHECK(trial_running);
        good_frames=0;step();CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
        CHECK(fault4_reason==2 && fault4_age==1 && fault4_good==0);
        uint32 saved_frame=fault4_frame;good_frames=100;vision_mode=1;frame_count+=100;
        step();CHECK(fault4_reason==2 && fault4_good==0 && fault4_frame==saved_frame);
    } else if (!strcmp(argv[1],"snapshot")) {
        good_frames=5;snapshot_requested=1;
        for(int i=0;i<200;i++)step();CHECK(!trial_running && auto_phase==AUTO_WAIT);
        snapshot_requested=0,rd29_requested=0;step();CHECK(auto_phase==AUTO_CENTER && !trial_running);
    } else if (!strcmp(argv[1],"stale")) {
        good_frames=5;trial_ticks=30;frame_tick=0;trial_tick();
        CHECK(!trial_running && auto_phase==AUTO_WAIT && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"running_stale")) {
        good_frames=5;for(int i=0;i<101;i++)step();CHECK(trial_running);
        frame_tick=trial_ticks-19;trial_tick();
        CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
        CHECK(fault4_reason==1 && fault4_age==20 && fault4_good==5);
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
        encoder_data_1=80;encoder_data_2=80;
        step();CHECK(motor1==1500 && motor2==1500);
        for(int i=0;i<100;i++)step();
        CHECK(trial_running && trial_fault==0); /* New detector owns validity. */
        good_frames=0;step();
        CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"curve_speed")) {
        speed_target=TRIAL_DRIVE_TARGET;good_frames=5;
        for(int i=0;i<101;i++)step();
        steering_speed_scale=80.0f/100.0f;encoder_data_1=80;encoder_data_2=80;
        step();CHECK(motor1==1500 && motor2==1500);
        steering_speed_scale=1.0f;step();CHECK(motor1==2500 && motor2==2500);
        vision_degraded=1;step();CHECK(motor1==1500 && motor2==1500);
    } else if (!strcmp(argv[1],"high_curve_speed")) {
        speed_target=140;good_frames=5;
        for(int i=0;i<101;i++)step();
        CHECK(trial_running && !vision_ring && !vision_degraded);
        encoder_data_1=80;encoder_data_2=80;
        steering_speed_scale=0.8f;step();
        CHECK(speed_pid_l.last_error>-0.001f && speed_pid_l.last_error<0.001f);
        CHECK(speed_pid_r.last_error>-0.001f && speed_pid_r.last_error<0.001f);
        CHECK(motor1==TRIAL_DRIVE_FEEDFORWARD && motor2==TRIAL_DRIVE_FEEDFORWARD);
        steering_speed_scale=1.0f;step();
        CHECK(speed_pid_l.last_error+80==140 && speed_pid_r.last_error+80==140);
        speed_target=50;steering_speed_scale=0.8f;step();
        CHECK(speed_pid_l.last_error+80==40 && speed_pid_r.last_error+80==40);
    } else if (!strcmp(argv[1],"stripe_start")) {
        marker=1;publish_mode(1,40);good_frames=5;
        CHECK(vision_mode==2 && vision_degraded && vision_stripe);
        for(int i=0;i<200;i++)step();
        CHECK(!trial_running && auto_phase==AUTO_WAIT);
        marker=0;publish_mode(1,40);
        for(int i=0;i<101;i++)step();CHECK(trial_running);
    } else if (!strcmp(argv[1],"stripe_recovery")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        CHECK(trial_running);
        marker=1;publish_mode(1,40);encoder_data_1=24;encoder_data_2=24;
        step();CHECK(motor1==1500 && motor2==1500 && vision_stripe);
        for(int i=0;i<20;i++){step();CHECK(motor1==1500 && motor2==1500);}
        marker=0;publish_mode(1,40);
        float previous=24;
        for(int i=0;i<100;i++){
            step();float target=speed_pid_l.last_error+24;
            CHECK(target>=previous && target-previous<=0.8001f && target<=100.0001f);
            CHECK(speed_pid_l.last_error==speed_pid_r.last_error);previous=target;
        }
        CHECK(previous>=99.99f);
        marker=1;publish_mode(1,40);step();CHECK(motor1==1500 && motor2==1500);
        marker=0;publish_mode(1,40);for(int i=0;i<75;i++)step();
        steering_speed_scale=STEERING_TIGHT_CURVE_SPEED_SCALE;step();
        CHECK(speed_pid_l.last_error+24>=79.999f && speed_pid_l.last_error+24<=80.001f); /* Curve cap wins once stripe recovery rises above80. */
        rx="STOP\n";step();CHECK(!trial_running && trial_fault==1 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"stripe_stale")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        marker=1;publish_mode(1,40);
        for(int i=0;i<20;i++)trial_tick();
        CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"stripe_lost")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        marker=1;publish_mode(1,40);good_frames=0;step();
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
    } else if (!strcmp(argv[1],"stripe_degraded")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        marker=1;publish_mode(1,40);encoder_data_1=24;encoder_data_2=24;step();
        marker=0;publish_mode(1,0);
        for(int i=0;i<100;i++){step();CHECK(speed_pid_l.last_error+24<=80.001f);}
        CHECK(speed_pid_l.last_error+24>=79.999f);
    } else if (!strcmp(argv[1],"no_stripe_target_change")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        speed_target=80;encoder_data_1=24;encoder_data_2=24;step();
        CHECK(speed_pid_l.last_error+24==80);
        speed_target=30;step();CHECK(speed_pid_l.last_error+24==30);
    } else if (!strcmp(argv[1],"ring_speed")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        roundabout_state.phase=RB_ORBIT;publish_mode(1,40);
        encoder_data_1=80;encoder_data_2=80;step();
        CHECK(trial_running && speed_pid_l.last_error==0 && motor1==1500 && motor2==1500);
        marker=1;publish_mode(1,40);step();
        CHECK(speed_pid_l.last_error+80==24); /* stripe cap always wins */
    } else if (!strcmp(argv[1],"cross_speed")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        cross_path.active=1;publish_mode(1,40);
        encoder_data_1=80;encoder_data_2=80;step();
        CHECK(trial_running && vision_mode==2 && speed_pid_l.last_error==0);
        publish_mode(0,40);good_frames=0;step();
        CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"entry_speed")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        encoder_data_1=30;encoder_data_2=30;
        vision_entry_kind=2;step();CHECK(speed_pid_l.last_error+30==50);
        vision_entry_kind=4;step();CHECK(speed_pid_l.last_error+30==50);
        vision_entry_kind=5;step();CHECK(speed_pid_l.last_error+30==35);
        marker=1;publish_mode(1,40);step();CHECK(speed_pid_l.last_error+30==24);
    } else if (!strncmp(argv[1],"bridge_",7)) {
        good_frames=5;for(int i=0;i<101;i++)step();
        roundabout_state.phase=RB_ENTER;
        vision_entry_kind=5;
        vision_bridge.active=vision_bridge.stable=1;
        vision_bridge.visual_tick=trial_ticks;
        vision_bridge.distance_m=roundabout_state.distance_m;
        vision_bridge.yaw_deg=roundabout_state.yaw_deg;
        vision_bridge.curvature=0;
        if (!strcmp(argv[1],"bridge_time")) {
            for(int i=0;i<9;i++){step();CHECK(trial_running);}
            step();CHECK(!trial_running && fault4_reason==4);
        } else if (!strcmp(argv[1],"bridge_distance")) {
            roundabout_state.distance_m+=0.041f;step();
            CHECK(!trial_running && fault4_reason==4);
        } else if (!strcmp(argv[1],"bridge_crossing")) {
            roundabout_state.distance_m+=0.039f;step();
            CHECK(!trial_running && fault4_reason==4);
        } else if (!strcmp(argv[1],"bridge_yaw")) {
            roundabout_state.yaw_deg+=3.1f;step();
            CHECK(!trial_running && fault4_reason==4);
        } else if (!strcmp(argv[1],"bridge_stale")) {
            frame_tick=trial_ticks-19;trial_tick();
            CHECK(!trial_running && (fault4_reason&1));
        } else if (!strcmp(argv[1],"bridge_encoder")) {
            encoder_data_1=151;step();CHECK(!trial_running && trial_fault==5);
        } else if (!strcmp(argv[1],"bridge_stop")) {
            rx="STOP\n";step();CHECK(!trial_running && trial_fault==1);
        } else return 2;
        CHECK(motor1==0 && motor2==0);
        vision_bridge.active=0;good_frames=100;
        for(int i=0;i<110;i++)step();
        CHECK(!trial_running && auto_phase==AUTO_LOCKED && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"ring_curve_speed")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        vision_ring=1;steering_speed_scale=0.8f;
        encoder_data_1=64;encoder_data_2=64;step();
        CHECK(trial_running && motor1==TRIAL_DRIVE_FEEDFORWARD && motor2==TRIAL_DRIVE_FEEDFORWARD);
    } else if (!strcmp(argv[1],"ring_fault")) {
        good_frames=5;for(int i=0;i<101;i++)step();
        roundabout_state.phase=RB_ENTER;roundabout_state.ticks=4499;
        step();CHECK(!trial_running && trial_fault==8 && motor1==0 && motor2==0);
    } else return 2;
    printf("PASS: actual trial_tick %s\n",argv[1]);return 0;
}
'''
out = Path(os.environ.get('HOST_TEST_OUT', str(p)))
out.mkdir(parents=True, exist_ok=True)
generated = out/'auto-integration-generated.c'
executable = out/'auto-integration.exe'
shared_inc = p.parents[3]/'firmware/camera_car/demo/user/inc'
header = (p.parent/'demo/user/inc/main.h').read_text(encoding='utf-8')
drive_defines = '\n'.join(re.findall(r'^#define (?:TRIAL_DRIVE_|CROSS_APPROACH_TARGET).*$', header, flags=re.M))
generated.write_text(stubs+drive_defines+'\n'+trace+supervisor+publication+tests,encoding='utf-8')
if os.environ.get('CC')=='cl':
    subprocess.run(['cl','/nologo','/TC','/W3','/I'+str(p/'vision-stubs'),'/I'+str(p.parent/'demo/user/inc'),'/I'+str(shared_inc),str(generated),str(p.parent/'demo/user/src/pid.c'),str(p.parent/'demo/user/src/auto_start.c'),str(p.parent/'demo/user/src/steering_control.c'),'/Fe'+str(executable)],cwd=p,check=True)
else:
    zig=os.environ.get('ZIG') or shutil.which('zig')
    if not zig: raise SystemExit('Set CC=cl in a Visual Studio developer shell, or set ZIG.')
    subprocess.run([zig,'cc','-Wall','-Wextra','-Werror','-I',str(p/'vision-stubs'),'-I',str(p.parent/'demo/user/inc'),'-I',str(shared_inc),str(generated),str(p.parent/'demo/user/src/pid.c'),str(p.parent/'demo/user/src/auto_start.c'),str(p.parent/'demo/user/src/steering_control.c'),'-o',str(executable)],check=True)
for case in ['view','auto','lost','snapshot','stale','running_stale','bench','pwm100','single','curve_speed','high_curve_speed',
             'stripe_start','stripe_recovery','stripe_stale','stripe_lost','stripe_encoder',
             'stripe_stall','stripe_degraded','no_stripe_target_change','ring_speed','ring_curve_speed','ring_fault','entry_speed','bridge_time','bridge_distance','bridge_yaw',
             'bridge_stale','bridge_encoder','bridge_stop','bridge_crossing','cross_speed']:
    subprocess.run([str(executable),case],check=True)
