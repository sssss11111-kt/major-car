
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "auto_start.h"
#include "roundabout_control.h"
#include "entry_bridge.h"
#include "pid.h"
#include "steering_control.h"
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
/* FIRST_FAULT4_BEGIN: immutable after latch, even if current vision recovers. */
static volatile uint32 fault4_age=0,fault4_frame=0,fault4_good=0;
static volatile uint8 fault4_reason=0,fault4_mode=0,fault4_rows=0,fault4_phase=0;
typedef struct {
    uint8 type, frame_type, cross_phase, scan_status, flags;
} special_diag_t;
static volatile special_diag_t vision_special={0}, fault4_special={0};

static uint8 usable_rows=0;
#define TRIAL_DRIVE_FEEDFORWARD (1500.0f)
#define TRIAL_DRIVE_MAX (10000.0f)
#define TRIAL_DRIVE_TARGET (100.0f)
#define TRIAL_DRIVE_DEGRADED_TARGET (80.0f)
#define CROSS_APPROACH_TARGET (35.0f) /* counts/10 ms during measured entry retry */
#define TRIAL_DRIVE_STRIPE_TARGET (24.0f)
#define TRIAL_DRIVE_INVALID_FRAME_LIMIT 5U
typedef struct {
    uint32 tick, run, fault, stall, p1, p2, val1, val3_l, val3_r;
    uint32 outen, mask, mctrl, fsts, source, dir;
    int32 e1, e2;
} drive_trace_t;
volatile drive_trace_t drive_trace[128];
volatile uint32 drive_trace_count=0;
static uint8 drive_trace_frozen=0;
volatile uint32 trial_ticks=0, frame_tick=0, good_frames=0, frame_count=0;
volatile uint8 trial_running=0, trial_fault=0;
volatile uint8 trial_invalid_frames=0;
volatile int trial_left_pwm=0, trial_right_pwm=0;
static uint32 start_tick=0, lease_tick=0;
static uint8 used=0, stall_ticks=0;
/* Solely owned by the 10 ms ISR. Steering history remains main-owned.
 * Arm only after a stripe, preserving all ordinary LAP21 speed behavior. */
static float stripe_speed_cap=TRIAL_DRIVE_STRIPE_TARGET;
static uint8 stripe_speed_recovering=0;
static uint32 sf_read_command(uint8 *ch) {
    uint32 n=wireless_uart_read_buffer(ch,1);
    return n ? n:debug_read_ring_buffer(ch,1);
}
void trial_tick(void)
{
    static char cmd[16]; static uint8 len=0;
    uint8 ch, n=0;
    ++trial_ticks;
    while (n++ < 32 && sf_read_command(&ch)) {
        if (ch=='\n' || ch=='\r') {
            cmd[len]=0;
            if (!strcmp(cmd,"STOP")) { trial_running=0; if (!trial_fault) trial_fault=1; }
            else if (!trial_running && !snapshot_requested && !strcmp(cmd,"WIMG1")) sf_view_mode=1;
            else if (!trial_running && !snapshot_requested && !strcmp(cmd,"WIMG2")) sf_view_mode=2;
            else if (!trial_running && !snapshot_requested && !strcmp(cmd,"UIMG1")) sf_view_mode=3;
            else if (!trial_running && !snapshot_requested && !strcmp(cmd,"UIMG2")) sf_view_mode=4;
            else if (!strcmp(cmd,"IMGOFF")) sf_view_mode=0;
            else if (!strcmp(cmd,"TRACE") && !trial_running && !snapshot_requested) {
                rd29_requested=1;snapshot_requested=1;
            }
            else if (!strcmp(cmd,"SNAP") && !trial_running && !snapshot_requested) snapshot_requested=1;
            else if (!strcmp(cmd,"K") && trial_running) lease_tick=trial_ticks;
            else if (wireless_ready && !snapshot_requested && !sf_view_mode && !used && !trial_fault &&
                     !strcmp(cmd,"BENCH")) {
                trial_bench = 1;
                used=1; trial_running=1; start_tick=lease_tick=trial_ticks;
                stall_ticks=0;
                My_Pid_Init();
            }
            len=0;
        } else if (len<15) cmd[len++]=(char)ch;
        else { len=0; trial_running=0; trial_fault=7; }
    }
    uint8 track_ready = vision_mode == 1 && good_frames >= 5 && model_good >= 5 &&
                        trial_ticks - frame_tick < 15;
    auto_phase = auto_start_step(&auto_start_state, trial_ticks, track_ready,
                                snapshot_requested || sf_view_mode,
                                trial_fault || (used && !trial_running) || trial_bench);
    auto_center_since = auto_start_state.center_since;
    if (auto_phase == AUTO_CENTER) {
        /* Hold the calibrated command midpoint throughout the full delay.
         * Main-loop steering is enabled only after trial_running becomes true. */
        pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_motor_duty_middle));
        steering_command = servo_motor_duty_middle;
    }
    if (auto_phase == AUTO_STARTED && !used) {
        auto_run = 1; trial_bench = 0;
        used = 1; trial_running = 1; start_tick = lease_tick = trial_ticks;
        stall_ticks = 0;
        My_Pid_Init();
    }
    /* Check bridge distance/yaw against this tick's validated encoder update. */
    rb_tick(&roundabout_state,encoder_data_1,encoder_data_2,trial_running && !trial_bench);
    if (trial_running) {
        if (trial_ticks-start_tick>=(trial_bench ? 200U : 12000U)) { trial_running=0; trial_fault=2; }
        else if (!auto_run && trial_ticks-lease_tick>=50) { trial_running=0; trial_fault=3; }
        else if (!trial_bench && (trial_ticks-frame_tick>=20 || trial_invalid_frames>=TRIAL_DRIVE_INVALID_FRAME_LIMIT ||
                 (vision_bridge.active && !entry_bridge_allowed(&vision_bridge,roundabout_state.distance_m,
                                                               roundabout_state.yaw_deg,trial_ticks)))) {
            fault4_age=trial_ticks-frame_tick;fault4_frame=frame_count;fault4_good=good_frames;
            fault4_reason=(uint8)((fault4_age>=20 ? 1:0) | (trial_invalid_frames>=TRIAL_DRIVE_INVALID_FRAME_LIMIT ? 2:0) |
                (vision_bridge.active && !entry_bridge_allowed(&vision_bridge,roundabout_state.distance_m,
                                                               roundabout_state.yaw_deg,trial_ticks) ? 4:0));
            fault4_mode=vision_mode;fault4_rows=usable_rows;fault4_phase=roundabout_state.phase;
            fault4_special=vision_special;
            trial_running=0;trial_fault=4;
        }
        else if (encoder_data_1 < -5 || encoder_data_2 < -5 ||
                 encoder_data_1 > 150 || encoder_data_2 > 150) { trial_running=0; trial_fault=5; }
        else if (trial_ticks-start_tick>30) {
            if (encoder_data_1<2 || encoder_data_2<2) ++stall_ticks; else stall_ticks=0;
            if (stall_ticks>=20) { trial_running=0; trial_fault=6; }
        }
    }
    if(trial_running && !trial_bench && roundabout_state.fault) {
        trial_running=0;trial_fault=8;
    }
    gpio_set_level(MOTOR1_DIR,MOTOR1_FORWARD_DIR_LEVEL);
    gpio_set_level(MOTOR2_DIR,MOTOR2_FORWARD_DIR_LEVEL);
    if (trial_running) {
        float effective_target = speed_target;
        if (!trial_bench) {
            if (vision_ring && effective_target > RB_TARGET_COUNTS)
                effective_target=RB_TARGET_COUNTS;
            effective_target *= steering_speed_scale;
            if (vision_degraded && effective_target > TRIAL_DRIVE_DEGRADED_TARGET)
                effective_target = TRIAL_DRIVE_DEGRADED_TARGET;
            if ((vision_entry_kind==2 || vision_entry_kind==4) && effective_target>ENTRY_SINGLE_EDGE_TARGET)
                effective_target=ENTRY_SINGLE_EDGE_TARGET;
            if (vision_cross_approach && effective_target > CROSS_APPROACH_TARGET)
                effective_target = CROSS_APPROACH_TARGET;
            if (vision_entry_kind==5 && effective_target>ENTRY_BRIDGE_TARGET)
                effective_target=ENTRY_BRIDGE_TARGET;
            if (vision_stripe) {
                stripe_speed_cap=TRIAL_DRIVE_STRIPE_TARGET;
                stripe_speed_recovering=1;
            } else if (stripe_speed_recovering) {
                stripe_speed_cap += speed_target*STEERING_SPEED_RECOVERY_PER_S*0.01f;
                if (stripe_speed_cap >= speed_target) stripe_speed_recovering=0;
            }
            if (stripe_speed_recovering && effective_target > stripe_speed_cap)
                effective_target=stripe_speed_cap;
        } else stripe_speed_recovering=0;
        speed_real=encoder_data_1;
        speed_pwm=PidLocCtrl(&speed_pid_l,effective_target-speed_real,1.f);
        trial_left_pwm=(int)constrain_float(TRIAL_DRIVE_FEEDFORWARD+speed_pwm,0,TRIAL_DRIVE_MAX);
        speed_real=encoder_data_2;
        speed_pwm=PidLocCtrl(&speed_pid_r,effective_target-speed_real,1.f);
        trial_right_pwm=(int)constrain_float(TRIAL_DRIVE_FEEDFORWARD+speed_pwm,0,TRIAL_DRIVE_MAX);
    } else { trial_left_pwm=trial_right_pwm=0; speed_pwm=0; stripe_speed_recovering=0; }
    pwm_set_duty(MOTOR1_PWM,trial_left_pwm);
    pwm_set_duty(MOTOR2_PWM,trial_right_pwm);
    if (used && !drive_trace_frozen) {
        if (drive_trace_count<128) {
            volatile drive_trace_t *r=&drive_trace[drive_trace_count++];
            r->tick=trial_ticks; r->run=trial_running; r->fault=trial_fault;
            r->stall=stall_ticks; r->p1=trial_left_pwm; r->p2=trial_right_pwm;
            r->val1=PWM2->SM[0].VAL1; r->val3_l=PWM2->SM[0].VAL3; r->val3_r=PWM2->SM[1].VAL3;
            r->outen=PWM2->OUTEN; r->mask=PWM2->MASK; r->mctrl=PWM2->MCTRL;
            r->fsts=PWM2->FSTS; r->source=PWM2->DTSRCSEL; r->dir=GPIO2->DR;
            r->e1=encoder_data_1; r->e2=encoder_data_2;
        }
        if (!trial_running) drive_trace_frozen=1;
    }
}
static struct {uint8 active,approach_only;} cross_path;
static struct {uint8 type,frame_type,full_scan_attempted,full_scan_valid,diagnostic_flags;} scan;
static struct {uint8 phase;} cross;
static void publish_mode(uint8 valid,uint8 detected_both) {
uint8 degraded = valid && (cross_path.active || detected_both < 3 || img_get_stripe_active() ||
                                      img_get_roundabout_candidate() || rb_busy(&roundabout_state) ||
                                      rb_suspect(&roundabout_state));
            vision_ring = (uint8)(rb_busy(&roundabout_state) || rb_suspect(&roundabout_state));
            vision_stripe = img_get_stripe_active();
            vision_entry_kind=img_get_entry_status();
            vision_cross_approach=cross_path.approach_only && valid;
            vision_bridge=*img_get_entry_bridge_guard();
            vision_mode = valid ? (degraded ? 2 : 1) : 0;
            vision_degraded = degraded;
            vision_special=(special_diag_t){
                .type=(uint8)scan.type, .frame_type=(uint8)scan.frame_type, .cross_phase=(uint8)cross.phase,
                .scan_status=scan.full_scan_valid ? 2 : scan.full_scan_attempted,
                .flags=scan.diagnostic_flags
            };

}
static void publish_frame(uint8 valid) {
            if (valid) { if (good_frames<100) ++good_frames; }
            else good_frames=0;
            if (valid) trial_invalid_frames=0;
            else if (trial_invalid_frames < TRIAL_DRIVE_INVALID_FRAME_LIMIT)
                ++trial_invalid_frames;

frame_tick=trial_ticks; ++frame_count;
}

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
        scan.type=1;scan.frame_type=3;scan.full_scan_attempted=1;scan.full_scan_valid=1;
        scan.diagnostic_flags=8;cross.phase=3;publish_mode(0,40);
        good_frames=0;trial_invalid_frames=5;step();CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
        CHECK(fault4_reason==2 && fault4_age==1 && fault4_good==0);
        CHECK(fault4_special.type==1 && fault4_special.frame_type==3 && fault4_special.cross_phase==3);
        CHECK(fault4_special.scan_status==2 && fault4_special.flags==8);
        uint32 saved_frame=fault4_frame;good_frames=100;frame_count+=100;
        scan.type=scan.frame_type=scan.full_scan_attempted=scan.full_scan_valid=scan.diagnostic_flags=0;
        cross.phase=0;publish_mode(1,40);
        step();CHECK(fault4_reason==2 && fault4_good==0 && fault4_frame==saved_frame);
        CHECK(fault4_special.type==1 && fault4_special.frame_type==3 && fault4_special.cross_phase==3);
        CHECK(fault4_special.scan_status==2 && fault4_special.flags==8);
    } else if (!strcmp(argv[1],"loss5")) {
        good_frames=5;for(int i=0;i<101;i++)step();CHECK(trial_running);
        for(int i=1;i<=4;i++) {
            publish_frame(0);step();CHECK(trial_running && trial_invalid_frames==i);
            step();step();CHECK(trial_running && trial_invalid_frames==i);
        }
        publish_frame(1);step();CHECK(trial_running && trial_invalid_frames==0);
        for(int i=1;i<=5;i++) {
            publish_frame(0);step();
            CHECK(trial_invalid_frames==i);
            if(i<5)CHECK(trial_running && trial_fault==0);
        }
        CHECK(!trial_running && trial_fault==4 && fault4_reason==2 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"snapshot")) {
        good_frames=5;snapshot_requested=1;
        for(int i=0;i<200;i++)step();CHECK(!trial_running && auto_phase==AUTO_WAIT);
        snapshot_requested=0,rd29_requested=0;step();CHECK(auto_phase==AUTO_CENTER && !trial_running);
    } else if (!strcmp(argv[1],"stale")) {
        good_frames=5;trial_ticks=30;frame_tick=0;trial_tick();
        CHECK(!trial_running && auto_phase==AUTO_WAIT && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"running_stale")) {
        good_frames=5;for(int i=0;i<101;i++)step();CHECK(trial_running);
        scan.type=3;scan.frame_type=3;scan.full_scan_attempted=1;publish_mode(1,40);
        frame_tick=trial_ticks-19;trial_tick();
        CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
        CHECK(fault4_reason==1 && fault4_age==20 && fault4_good==5);
        CHECK(fault4_special.type==3 && fault4_special.scan_status==1 && fault4_special.cross_phase==0);
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
        good_frames=0;trial_invalid_frames=5;step();
        CHECK(!trial_running && trial_fault==4 && motor1==0 && motor2==0);
    } else if (!strcmp(argv[1],"curve_speed")) {
        speed_target=TRIAL_DRIVE_TARGET;good_frames=5;
        for(int i=0;i<101;i++)step();
        steering_speed_scale=80.0f/100.0f;encoder_data_1=80;encoder_data_2=80;
        step();CHECK(motor1==1500 && motor2==1500);
        steering_speed_scale=1.0f;step();CHECK(motor1==2500 && motor2==2500);
        vision_degraded=1;step();CHECK(motor1==1500 && motor2==1500);
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
        marker=1;publish_mode(1,40);good_frames=0;trial_invalid_frames=5;step();
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
        cross_path.approach_only=1;publish_mode(1,40);step();
        CHECK(speed_pid_l.last_error+80==CROSS_APPROACH_TARGET);
        publish_mode(0,40);good_frames=0;trial_invalid_frames=5;step();
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
