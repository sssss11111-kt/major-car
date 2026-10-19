/* SeekFree RT1064 adapters for the C-car application. GPL-3.0-or-later. */
#include "zf_common_headfile.h"
#include "car_config.h"
#include "car_core.h"
#include "car_app.h"
#include "car_servo.h"
#include <stdio.h>
#include <string.h>
#if MT9V03X_W != CAR_W || MT9V03X_H != CAR_H
#error "Camera dimensions must match the application buffers."
#endif
static control_state_t control;
static vision_state_t vision;
static vision_config_t vision_cfg;
static vision_result_t result;
static uint8_t gray[CAR_W*CAR_H], binary[CAR_W*CAR_H];
static uint8_t image_tx[CAR_W*CAR_H];
static volatile uint32_t milliseconds;
static volatile int16_t measured_left,measured_right;
static volatile uint8_t rx[64],rx_head,rx_tail;
static volatile uint32_t rx_received_ms[64]; /* STEP-06: lease age starts at RX, not dequeue. */
static volatile uint32_t stop_epoch;
typedef struct {
    char text[512];
    unsigned head,tail;
    uint32_t image_position,sent,dropped_lines;
} serial_tx_t;
static serial_tx_t serial_tx[2]; /* UART1 and UART8 advance independently. */
static uint32_t image_length,frame_count;
/* STEP-04: metadata is published by the actual completed-buffer callback. */
extern void (*mt9v03x_frame_observer)(void);
static volatile uint32_t capture_sequence,capture_ms;
static uint32_t consumed_sequence,dropped_frames,processing_ms,last_capture_age;
static volatile uint32_t wireless_received,wireless_errors;
static volatile uint8_t baud_handshake_active,baud_ack_count,baud_ack[3];
static uint8_t baud_handshake_result; /* 0 not attempted, 1 verified ACK, 2 failed */
static uint8_t image_requested,camera_ready;
static volatile uint8_t outputs_ready;
#if CAR_COMM_DIAGNOSTIC
static uint8_t c11_probe_level;
#endif
static uint32_t last_screen,last_telemetry;
static uint16_t calibration_selected;
static uint16_t sustain_selected;
static uint8_t low_selected=20;
#if CAR_SIMULATED
static uint32_t last_generated;
#else
static uint16_t exposure=MT9V03X_EXP_TIME_DEF;
#endif
static volatile uint8_t servo_test_active;
static volatile float servo_test_us;
static volatile uint32_t servo_test_until;
static const uint8_t image_header[4]={0x00,0xff,0x01,0x01};

static uint32_t lock(void) { return interrupt_global_disable(); }
static void unlock(uint32_t key) { interrupt_global_enable(key); }
static uint8_t images_idle(void) {
    return serial_tx[0].image_position>=image_length && serial_tx[1].image_position>=image_length;
}
static uint8_t texts_idle(void) {
    return serial_tx[0].tail==serial_tx[0].head && serial_tx[1].tail==serial_tx[1].head;
}
static void text_queue(const char *text) {
    size_t length=strlen(text);
    for(unsigned link=0;link<2;link++) {
        serial_tx_t *tx=&serial_tx[link];
        unsigned free_bytes=(tx->tail+sizeof tx->text-tx->head-1)%sizeof tx->text;
        /* Drop complete messages, never partial status lines; no cross-port stall. */
        if(length>free_bytes) { tx->dropped_lines++; continue; }
        for(size_t i=0;i<length;i++) {
            tx->text[tx->head]=text[i]; tx->head=(tx->head+1)%sizeof tx->text;
        }
    }
}
static void uart_flush_some(void) {
    /* No busy-wait, no dependency on readiness of the other port. */
    for(unsigned link=0;link<2;link++) {
        serial_tx_t *tx=&serial_tx[link];
        for(unsigned i=0;i<16;i++) {
#if CAR_WIRELESS_USE_RTS
            if(link==1 && gpio_get_level(CAR_WIRELESS_RTS)) break;
#endif
            uint32_t flags=link?LPUART_GetStatusFlags(LPUART8):LPUART_GetStatusFlags(LPUART1);
            if(!(flags&kLPUART_TxDataRegEmptyFlag)) break;
            uint8_t byte;
            if(tx->image_position<image_length) {
                uint32_t pos=tx->image_position++;
                byte=pos<4?image_header[pos]:image_tx[pos-4];
            } else if(tx->tail!=tx->head) {
                byte=(uint8_t)tx->text[tx->tail]; tx->tail=(tx->tail+1)%sizeof tx->text;
            } else break;
            if(link) LPUART_WriteByte(LPUART8,byte);
            else LPUART_WriteByte(LPUART1,byte);
            tx->sent++;
        }
    }
}
static void apply_outputs(void) {
#if CAR_COMM_DIAGNOSTIC
    pwm_set_duty(CAR_LEFT_PWM,0);
    pwm_set_duty(CAR_RIGHT_PWM,0);
#else
    pwm_set_duty(CAR_LEFT_PWM,(uint32_t)control.duty[0]);
    pwm_set_duty(CAR_RIGHT_PWM,(uint32_t)control.duty[1]);
#endif
    car_servo_set_us(control.servo_us);
}
void car_app_camera_frame_isr(void) {
    capture_ms=milliseconds;capture_sequence++;
}
void car_app_tick_10ms(void) {
    milliseconds+=10;
    measured_left=(int16_t)(CAR_LEFT_ENCODER_SIGN*encoder_get_count(CAR_LEFT_ENCODER));
    encoder_clear_count(CAR_LEFT_ENCODER);
    measured_right=(int16_t)(CAR_RIGHT_ENCODER_SIGN*encoder_get_count(CAR_RIGHT_ENCODER));
    encoder_clear_count(CAR_RIGHT_ENCODER);
    control_tick(&control,milliseconds,measured_left,measured_right);
    apply_outputs();
    if(servo_test_active) {
        if(control.mode!=CAR_IDLE || (int32_t)(milliseconds-servo_test_until)>=0)
            servo_test_active=0;
        else car_servo_set_us(servo_test_us);
    }
}
void car_app_uart_rx_isr(uint8_t byte) {
    /* Debug UART is enabled before PWM initialization; do not touch unready I/O. */
    if(!outputs_ready) return;
    /* Stop is executed in the RX interrupt, independent of graphics/foreground. */
    if(byte=='s' || byte=='S') {
        servo_test_active=0; control_stop(&control); apply_outputs();
        stop_epoch++;
        rx_tail=rx_head; /* cancel the entire pre-stop backlog, even when full */
        return;
    }
    uint8_t next=(uint8_t)((rx_head+1u)%64u);
    if(next!=rx_tail) { rx[rx_head]=byte;rx_received_ms[rx_head]=milliseconds;rx_head=next; }
}
void car_app_uart8_irq(void) {
    const uint32_t errors=kLPUART_RxOverrunFlag|kLPUART_FramingErrorFlag|
                          kLPUART_NoiseErrorFlag|kLPUART_ParityErrorFlag;
    /* Bound each IRQ; pending level-triggered RX will re-enter if necessary. */
    for(unsigned i=0;i<32;i++) {
        uint32_t flags=LPUART_GetStatusFlags(LPUART8);
        if(flags&errors) {
            wireless_errors++;
            LPUART_ClearStatusFlags(LPUART8,flags&errors);
        }
        if(!(flags&kLPUART_RxDataRegFullFlag)) break;
        uint8_t byte=LPUART_ReadByte(LPUART8);
        wireless_received++;
        /* Never interpret a framing/noise-corrupted byte as a motion command. */
        if(!(flags&errors)) {
            if(baud_handshake_active) {
                if(baud_ack_count<3) baud_ack[baud_ack_count++]=byte;
            } else car_app_uart_rx_isr(byte);
        }
    }
}
static void wireless_negotiate_baud(void) {
#if CAR_WIRELESS_AUTO_BAUD
    /* Manufacturer V2.x RTS auto-baud sequence, only during stopped startup.
       Unlike the vendor example's && check, require ALL three ACK bytes. */
    uint8_t initial=gpio_get_level(CAR_WIRELESS_RTS);
    baud_ack_count=0; baud_handshake_active=1;
    gpio_init(CAR_WIRELESS_RTS,GPO,initial,GPO_PUSH_PULL);
    system_delay_ms(5);
    gpio_set_level(CAR_WIRELESS_RTS,(uint8_t)!initial);
    system_delay_ms(100);
    gpio_set_level(CAR_WIRELESS_RTS,initial);
    const uint8_t sequence[3]={0x00,0x01,0x03};
    uint8_t sent=0;
    for(unsigned elapsed=0;elapsed<100 && sent<3;elapsed++) {
        if(LPUART_GetStatusFlags(LPUART8)&kLPUART_TxDataRegEmptyFlag) {
            LPUART_WriteByte(LPUART8,sequence[sent++]);
            serial_tx[1].sent++;
        } else system_delay_ms(1);
    }
    for(unsigned elapsed=0;elapsed<100 && baud_ack_count<3;elapsed++) system_delay_ms(1);
    baud_handshake_result=(uint8_t)(sent==3 && baud_ack_count==3 &&
        baud_ack[0]==0xa5 && baud_ack[1]==0xff && baud_ack[2]==0xff?1:2);
    baud_handshake_active=0;
    gpio_init(CAR_WIRELESS_RTS,GPI,GPIO_HIGH,GPI_PULL_UP);
    system_delay_ms(10);
#endif
}
static void wireless_diagnostic(void) {
    char line[220];
    uint32_t key=lock();
    uint32_t received=wireless_received,errors=wireless_errors;
    unlock(key);
    snprintf(line,sizeof line,
        "UART8-DIAG v=20261017B cfg_baud=%lu rx=%lu tx=%lu err=%lu rts=%u drop=%lu diag=%u ab=%u pattern=UUUU0123456789\r\n",
        (unsigned long)CAR_WIRELESS_BAUD,(unsigned long)received,
        (unsigned long)serial_tx[1].sent,(unsigned long)errors,
        (unsigned)gpio_get_level(CAR_WIRELESS_RTS),
        (unsigned long)serial_tx[1].dropped_lines,(unsigned)CAR_COMM_DIAGNOSTIC,
        (unsigned)baud_handshake_result);
    text_queue(line);
    snprintf(line,sizeof line,
        "MOTOR-DIR v=20261017I left=C11 cfg=%u sample=%u right=C9 cfg=%u sample=%u sample_unverified=1\r\n",
        (unsigned)CAR_LEFT_FORWARD,(unsigned)gpio_get_level(CAR_LEFT_DIR),
        (unsigned)CAR_RIGHT_FORWARD,(unsigned)gpio_get_level(CAR_RIGHT_DIR));
    text_queue(line);
#if defined(__arm__) || defined(__ARM_ARCH)
    /* GPIO_B0_07 addresses are specified by the SDK IOMUXC definitions.
       Read only: inspect regular GPIO2 versus fast GPIO7 routing and state. */
    snprintf(line,sizeof line,
        "C7-REG v=20261017D mux=%08lx ctl=%08lx route=%08lx dr2=%08lx dir2=%08lx psr2=%08lx dr7=%08lx dir7=%08lx psr7=%08lx\r\n",
        (unsigned long)*(volatile uint32_t *)0x401F8158u,
        (unsigned long)*(volatile uint32_t *)0x401F8348u,
        (unsigned long)IOMUXC_GPR->GPR27,
        (unsigned long)GPIO2->DR,(unsigned long)GPIO2->GDIR,(unsigned long)GPIO2->PSR,
        (unsigned long)GPIO7->DR,(unsigned long)GPIO7->GDIR,(unsigned long)GPIO7->PSR);
    text_queue(line);
#endif
#if CAR_COMM_DIAGNOSTIC
    snprintf(line,sizeof line,"C11-STATIC v=20261017E cfg=%u pad=%u motors=disabled\r\n",
        (unsigned)c11_probe_level,(unsigned)gpio_get_level(C11));
    text_queue(line);
#endif
}
static void servo_register_diagnostic(void) {
#if defined(__arm__) || defined(__ARM_ARCH)
    char line[240];
    uint32_t key=lock();
    uint32_t mux=*(volatile uint32_t *)0x401F81B4u;
    uint16_t ctrl=PWM4->SM[2].CTRL,mctrl=PWM4->MCTRL,out=PWM4->OUTEN;
    uint16_t fault=PWM4->FSTS,map=PWM4->SM[2].DISMAP[0],cnt=PWM4->SM[2].CNT;
    uint16_t init=PWM4->SM[2].INIT,v1=PWM4->SM[2].VAL1,v2=PWM4->SM[2].VAL2,v3=PWM4->SM[2].VAL3;
    unlock(key);
    snprintf(line,sizeof line,
        "SERVO-REG v=20261017J ipg=%lu mux=%08lx ctrl=%04x mctrl=%04x out=%04x fault=%04x map=%04x cnt=%04x init=%04x v1=%04x v2=%04x v3=%04x\r\n",
        (unsigned long)CLOCK_GetFreq(kCLOCK_IpgClk),(unsigned long)mux,
        ctrl,mctrl,out,fault,map,cnt,init,v1,v2,v3);
    text_queue(line);
#else
    text_queue("SERVO-REG: host I/O double; hardware registers unavailable\r\n");
#endif
}
/* STEP-05: coherent control/camera snapshot, at 10 Hz while following. */
static void telemetry(void) {
    char line[512];control_state_t snapshot;
    uint32_t key=lock();snapshot=control;
    uint32_t now=milliseconds,sequence=capture_sequence,age=now-capture_ms;
    int left=measured_left,right=measured_right;unsigned pulse=(unsigned)car_servo_get_us();
    unsigned phase=snapshot.mode==CAR_RUNNING?
        ((int32_t)(now-snapshot.motion_ms)<0?0u:(snapshot.bench_started==3?2u:1u)):3u;
    unsigned ready=camera_ready && control_tracking_ready(&snapshot,now);
    uint32_t accepted_age=now-snapshot.frame_ms;
    uint32_t trial_ms=snapshot.mode==CAR_RUNNING?now-snapshot.trial_started_ms:snapshot.trial_elapsed_ms;
    unlock(key);
    snprintf(line,sizeof line,
        "ms=%lu mode=%u fault=%u frames=%lu valid=%u pair=%u/%u thr=%u err10=%d bench_target=%u bench_started=%u enc=%d,%d pwm=%u,%u us=%u speed=%u target10=%u phase=%u quality=%u reject=%u seq=%lu cap_age=%lu proc_ms=%lu used_age=%lu drop_frames=%lu ready=%u good_frames=%u accepted_age=%lu trial_id=%lu trial_done=%u trial_ms=%lu\r\n",
        (unsigned long)now,(unsigned)snapshot.mode,(unsigned)snapshot.fault,
        (unsigned long)frame_count,result.valid,result.paired_rows,result.scanned_rows,
        result.threshold,(int)(result.error*10),(unsigned)snapshot.bench_target,
        (unsigned)snapshot.bench_started,left,right,(unsigned)snapshot.duty[0],(unsigned)snapshot.duty[1],
        pulse,(unsigned)snapshot.cfg.speed_target,(unsigned)(snapshot.bench_target*10),phase,
        result.quality,result.rejected_rows,(unsigned long)sequence,(unsigned long)age,
        (unsigned long)processing_ms,(unsigned long)last_capture_age,(unsigned long)dropped_frames,
        ready,snapshot.good_frames,(unsigned long)accepted_age,(unsigned long)snapshot.trial_id,
        snapshot.trial_done,(unsigned long)trial_ms);
    text_queue(line);
}
/* STEP-05: identify the trial configuration before any motion command. */
static void tracking_configuration(void) {
    char line[200];control_config_t cfg;
    uint32_t key=lock();cfg=control.cfg;unlock(key);
    snprintf(line,sizeof line,"TRACK-CFG v=20261019T2 target=%u cap=%u settle=%lu duration=%lu lease=%lu simulated=%u left_bias=%u right_bias=%u\r\n",
        (unsigned)cfg.speed_target,(unsigned)cfg.pwm_max,(unsigned long)cfg.tracking_settle_ms,
        (unsigned long)cfg.tracking_duration_ms,(unsigned long)cfg.tracking_lease_ms,(unsigned)cfg.simulated,
        (unsigned)cfg.tracking_pwm_bias[0],(unsigned)cfg.tracking_pwm_bias[1]);
    text_queue(line);
}
static void sustain_query(uint32_t received) {
    control_state_t snap;uint16_t selected;char line[280];
    uint32_t key=lock();control_drive_keepalive(&control,received);snap=control;selected=sustain_selected;
    uint32_t now=milliseconds;int left=measured_left,right=measured_right;unlock(key);
    snprintf(line,sizeof line,"HOLD-STATE ms=%lu mode=%u fault=%u id=%lu phase=%u result=%u total_ms=%lu hold_ms=%lu motor=%u duty=%u enc=%d,%d pwm=%u,%u n=%u sum=%ld min=%d max=%d zeros=%u kick_ticks=%u\r\n",
        (unsigned long)now,(unsigned)snap.mode,(unsigned)snap.fault,(unsigned long)snap.sustain_id,
        (unsigned)snap.sustain_phase,(unsigned)snap.sustain_result,(unsigned long)snap.sustain_total_ms,
        (unsigned long)snap.sustain_hold_ms,(unsigned)snap.test_motor,(unsigned)snap.sustain_duty,
        left,right,(unsigned)snap.duty[0],(unsigned)snap.duty[1],(unsigned)snap.sustain_n,
        (long)snap.sustain_sum,(int)snap.sustain_min,(int)snap.sustain_max,(unsigned)snap.sustain_zeros,(unsigned)snap.sustain_kick_ticks);text_queue(line);
    if(snap.mode!=CAR_SUSTAIN) {
    snprintf(line,sizeof line,"HOLD-CFG v=20261019H1 selected=%u step=100 cap=%u kick=%u kick_min=%u kick_max=%u hold=%u lease=%u guard=%u\r\n",
        (unsigned)selected,(unsigned)CAR_SUSTAIN_KICK_DUTY,(unsigned)CAR_SUSTAIN_KICK_DUTY,
        (unsigned)CAR_SUSTAIN_KICK_MIN_MS,(unsigned)CAR_SUSTAIN_KICK_MAX_MS,(unsigned)CAR_SUSTAIN_HOLD_MS,
        (unsigned)CAR_SUSTAIN_LEASE_MS,(unsigned)CAR_CALIBRATION_ENCODER_LIMIT);text_queue(line);
    }
}
static void low_query(uint32_t received) {
    control_state_t s;char line[480];uint8_t selected;
    uint32_t key=lock();control_drive_keepalive(&control,received);s=control;selected=low_selected;
    uint32_t now=milliseconds;int left=measured_left,right=measured_right;unlock(key);
    snprintf(line,sizeof line,"LOW-STATE ms=%lu mode=%u fault=%u id=%lu done=%u elapsed=%lu motor=%u target=%u confirmed=%u regulating=%u enc=%d,%d pwm=%u,%u n=%u,%u sum=%ld,%ld e2=%lu,%lu ae=%lu,%lu min=%d,%d max=%d,%d\r\n",
        (unsigned long)now,(unsigned)s.mode,(unsigned)s.fault,(unsigned long)s.low_id,s.low_done,
        (unsigned long)s.low_elapsed,s.test_motor,s.low_target,s.low_confirmed,s.low_regulating,left,right,
        (unsigned)s.duty[0],(unsigned)s.duty[1],s.low_n[0],s.low_n[1],(long)s.low_sum[0],(long)s.low_sum[1],
        (unsigned long)s.low_error2[0],(unsigned long)s.low_error2[1],(unsigned long)s.low_abs_error[0],(unsigned long)s.low_abs_error[1],
        s.low_min[0],s.low_min[1],s.low_max[0],s.low_max[1]);text_queue(line);
    if(s.mode!=CAR_LOW_SPEED){
        snprintf(line,sizeof line,"LOW-CFG v=20261019L2 selected=%u min=10 max=40 step=5 duration=6000 stats=3000 lease=500 kick=3000 kick_min=30 kick_max=200 cap=1500 guard=450 transfer=1500 kp10=%u ki10=%u ff0=%u,%u ff10=%u,%u\r\n",
            selected,(unsigned)(s.cfg.low_kp*10),(unsigned)(s.cfg.low_ki*10),(unsigned)s.cfg.low_ff_offset[0],(unsigned)s.cfg.low_ff_offset[1],
            (unsigned)(s.cfg.low_ff_slope[0]*10),(unsigned)(s.cfg.low_ff_slope[1]*10));text_queue(line);
    }
}
static void camera_init_once(void) {
    uint32_t key=lock(); control_stop(&control); apply_outputs(); unlock(key);
    servo_test_active=0; frame_count=0; memset(&result,0,sizeof result);
    key=lock();capture_sequence=consumed_sequence=0;capture_ms=milliseconds;
    mt9v03x_finish_flag=0;unlock(key);
    mt9v03x_frame_observer=car_app_camera_frame_isr;
    vision_reset(&vision);
#if CAR_COMM_DIAGNOSTIC
    camera_ready=0;
    text_queue("COMM DIAG: camera skipped; motion commands disabled\r\n");
#elif CAR_SIMULATED
    camera_ready=1; text_queue("SIMULATED: motors disabled\r\n");
#else
    camera_ready=(uint8_t)(mt9v03x_init()==0);
    if(camera_ready) text_queue("CAMERA OK 188x120\r\n");
    else {
        key=lock(); control_fault(&control,FAULT_CAMERA); unlock(key);
        text_queue("CAMERA FAILED: check adapter, cable, power; r retries\r\n");
    }
#endif
}
void car_app_init(void) {
    control_config_t cfg={0};
    cfg.speed_target=CAR_SPEED_TARGET; cfg.speed_kp=CAR_SPEED_KP;
    cfg.speed_ki=CAR_SPEED_KI; cfg.pwm_max=CAR_PWM_LIMIT;
    /* T2 STEP-01: only real-image tracking receives the per-wheel bias. */
    cfg.tracking_pwm_bias[0]=CAR_TRACK_LEFT_PWM_BIAS;
    cfg.tracking_pwm_bias[1]=CAR_TRACK_RIGHT_PWM_BIAS;
    /* L1 candidate from H1 airborne curves; physically tune before tracking adoption. */
    cfg.low_kp=4;cfg.low_ki=20;
    cfg.low_ff_offset[0]=470;cfg.low_ff_offset[1]=660;
    cfg.low_ff_slope[0]=6.5f;cfg.low_ff_slope[1]=7;
    cfg.steer_kp=CAR_STEER_KP; cfg.steer_kd=CAR_STEER_KD; cfg.steer_sign=CAR_STEER_SIGN;
    cfg.servo_center_us=CAR_SERVO_CENTER_US;
    cfg.servo_min_us=CAR_SERVO_MIN_US; cfg.servo_max_us=CAR_SERVO_MAX_US;
    cfg.frame_timeout_ms=CAR_FRAME_TIMEOUT_MS;
    cfg.heartbeat_timeout_ms=CAR_FOREGROUND_TIMEOUT_MS;
    /* STEP-02 */
    cfg.tracking_settle_ms=CAR_TRACK_SETTLE_MS;cfg.tracking_duration_ms=CAR_TRACK_DURATION_MS;
    cfg.tracking_lease_ms=CAR_TRACK_LEASE_MS;
    cfg.simulated=(CAR_SIMULATED || CAR_COMM_DIAGNOSTIC);
    control_init(&control,&cfg); vision_reset(&vision); vision_config_default(&vision_cfg);
    gpio_init(CAR_LEFT_DIR,GPO,CAR_LEFT_FORWARD,GPO_PUSH_PULL);
    gpio_init(CAR_RIGHT_DIR,GPO,CAR_RIGHT_FORWARD,GPO_PUSH_PULL);
    pwm_init(CAR_LEFT_PWM,CAR_MOTOR_HZ,0); pwm_init(CAR_RIGHT_PWM,CAR_MOTOR_HZ,0);
#if CAR_COMM_DIAGNOSTIC
    c11_probe_level=0; gpio_init(C11,GPO,GPIO_LOW,GPO_PUSH_PULL);
#endif
    car_servo_init();
    outputs_ready=1;
    /* Match the uart_init signature: MCU TX first, MCU RX second. */
    uart_init(CAR_WIRELESS_UART,CAR_WIRELESS_BAUD,CAR_WIRELESS_TX,CAR_WIRELESS_RX);
    gpio_init(CAR_WIRELESS_RTS,GPI,GPIO_HIGH,GPI_PULL_UP);
    interrupt_set_priority(LPUART1_IRQn,2);
    interrupt_set_priority(LPUART8_IRQn,2);
    uart_rx_interrupt(CAR_WIRELESS_UART,1);
#if CAR_ENCODER_QUADRATURE
    encoder_quad_init(CAR_LEFT_ENCODER,CAR_LEFT_A,CAR_LEFT_B);
    encoder_quad_init(CAR_RIGHT_ENCODER,CAR_RIGHT_A,CAR_RIGHT_B);
#else
    encoder_dir_init(CAR_LEFT_ENCODER,CAR_LEFT_A,CAR_LEFT_B);
    encoder_dir_init(CAR_RIGHT_ENCODER,CAR_RIGHT_A,CAR_RIGHT_B);
#endif
    pit_ms_init(PIT_CH0,10);
    interrupt_set_priority(PIT_IRQn,1);
    interrupt_set_priority(LPUART1_IRQn,2);
    interrupt_set_priority(CSI_IRQn,3);
    wireless_negotiate_baud();
    text_queue("C-CAR RT1064 SeekFree V3.5.5 TRACK20261019T2: h help; s STOP\r\n");
    wireless_diagnostic();
#if CAR_DISPLAY_ENABLED
    tft180_set_dir(TFT180_PORTAIT); tft180_init();
    tft180_show_string(0,0,"C-CAR INIT");
#endif
    camera_init_once();
}
static void command(uint8_t byte,uint32_t epoch,uint32_t received) {
    uint32_t key=lock();
    /* Also cancel a command already dequeued when an RX stop preempted us. */
    if(epoch!=stop_epoch) { unlock(key); return; }
#if CAR_COMM_DIAGNOSTIC
    if(byte!='p' && byte!='q' && byte!='h' && byte!='s' && byte!='S' && byte!='u' && byte!='v' && byte!='z') {
        unlock(key); return;
    }
#endif
    switch(byte) {
    case 's': case 'S': servo_test_active=0; control_stop(&control); apply_outputs(); break;
    case 'F': if(control.mode==CAR_IDLE && low_selected<40)low_selected+=5;break;
    case 'G': if(control.mode==CAR_IDLE && low_selected>10)low_selected-=5;break;
    case 'J': case 'N': case 'B':
        if(control.mode==CAR_IDLE && !control.cfg.simulated){
            gpio_init(CAR_LEFT_DIR,GPO,CAR_LEFT_FORWARD,GPO_PUSH_PULL);
            gpio_init(CAR_RIGHT_DIR,GPO,CAR_RIGHT_FORWARD,GPO_PUSH_PULL);
        }
        if(control_low_test(&control,byte=='J'?1:byte=='N'?2:3,low_selected,milliseconds)){
            char line[160];servo_test_active=0;apply_outputs();
            snprintf(line,sizeof line,"LOW-START v=20261019L2 id=%lu motor=%u target=%u duration=6000 lease=500\r\n",
                (unsigned long)control.low_id,control.test_motor,control.low_target);text_queue(line);
        }else text_queue("LOW-START REFUSED\r\n");
        break;
    case 'I': if(control.mode==CAR_IDLE && sustain_selected<CAR_SUSTAIN_KICK_DUTY)sustain_selected+=100;break;
    case 'K': if(control.mode==CAR_IDLE && sustain_selected>=100)sustain_selected-=100;break;
    case 'U': case 'V':
        if(control.mode==CAR_IDLE && !control.cfg.simulated) {
            gpio_init(CAR_LEFT_DIR,GPO,CAR_LEFT_FORWARD,GPO_PUSH_PULL);
            gpio_init(CAR_RIGHT_DIR,GPO,CAR_RIGHT_FORWARD,GPO_PUSH_PULL);
        }
        if(control_sustain_test(&control,byte=='U'?1:2,sustain_selected,milliseconds)) {
            char line[180];servo_test_active=0;apply_outputs();
            snprintf(line,sizeof line,"HOLD-START v=20261019H1 motor=%u duty=%u kick=%u kick_min=%u kick_max=%u hold=%u lease=%u guard=%u\r\n",
                byte=='U'?1u:2u,(unsigned)sustain_selected,(unsigned)CAR_SUSTAIN_KICK_DUTY,
                (unsigned)CAR_SUSTAIN_KICK_MIN_MS,(unsigned)CAR_SUSTAIN_KICK_MAX_MS,(unsigned)CAR_SUSTAIN_HOLD_MS,
                (unsigned)CAR_SUSTAIN_LEASE_MS,(unsigned)CAR_CALIBRATION_ENCODER_LIMIT);text_queue(line);
        }else text_queue("HOLD-START REFUSED\r\n");
        break;
    case 'P': if(control.mode==CAR_IDLE && calibration_selected<CAR_CALIBRATION_MAX_DUTY)calibration_selected+=500;break;
    case 'M': if(control.mode==CAR_IDLE && calibration_selected>=500)calibration_selected-=500;break;
    case 'L': case 'R':
        if(control.mode==CAR_IDLE && !control.cfg.simulated) {
            gpio_init(CAR_LEFT_DIR,GPO,CAR_LEFT_FORWARD,GPO_PUSH_PULL);
            gpio_init(CAR_RIGHT_DIR,GPO,CAR_RIGHT_FORWARD,GPO_PUSH_PULL);
        }
        if(control_calibration_test(&control,byte=='L'?1:2,calibration_selected,milliseconds)) {
            char line[160];servo_test_active=0;apply_outputs();
            snprintf(line,sizeof line,"CAL-START v=20261019C3 motor=%u duty=%u duration=2000 lease=500 cap=%u guard=%u\r\n",
                     byte=='L'?1u:2u,(unsigned)calibration_selected,(unsigned)CAR_CALIBRATION_MAX_DUTY,
                     (unsigned)CAR_CALIBRATION_ENCODER_LIMIT);text_queue(line);
        } else text_queue("CAL-START REFUSED\r\n");
        break;
#if CAR_COMM_DIAGNOSTIC
    case 'u': case 'v':
        c11_probe_level=(uint8_t)(byte=='u');
        gpio_set_level(C11,c11_probe_level);
        text_queue(c11_probe_level?"C11 HIGH; motors disabled\r\n":"C11 LOW; motors disabled\r\n");
        break;
#endif
    case '0': case '7': case '8': case '9':
        if(control.mode==CAR_IDLE && !control.cfg.simulated) {
            gpio_init(CAR_LEFT_DIR,GPO,CAR_LEFT_FORWARD,GPO_PUSH_PULL);
            gpio_init(CAR_RIGHT_DIR,GPO,CAR_RIGHT_FORWARD,GPO_PUSH_PULL);
        }
        if(byte=='0'?control_drive_long_test(&control,milliseconds):control_drive_test(&control,byte=='7'?0:(byte=='8'?-1:1),milliseconds)) {
            servo_test_active=0;apply_outputs();
            text_queue(byte=='0'?"DRIVE-TEST v=20261018N settle=200ms drive=10000ms target=40 cap=30% vision=ignored\r\n":
                "DRIVE-TEST v=20261018N settle=200ms drive=1000ms target=40 cap=30% vision=ignored\r\n");
        } else text_queue("DRIVE-TEST REFUSED\r\n");
        break;
    case 'w':
        if(camera_ready && control_steer_test(&control,milliseconds)) {
            servo_test_active=0;
            text_queue("STATIC-STEER v=20261017K 30s motors=0; s STOP\r\n");
        } else text_queue("STATIC-STEER REFUSED: idle + 3 fresh valid frames required\r\n");
        break;
    case 'a':
        if(camera_ready && control_start(&control,milliseconds)) {
            /* STEP-02: cancel manual servo override and publish visual steering at zero PWM. */
            servo_test_active=0;
            gpio_init(CAR_LEFT_DIR,GPO,CAR_LEFT_FORWARD,GPO_PUSH_PULL);
            gpio_init(CAR_RIGHT_DIR,GPO,CAR_RIGHT_FORWARD,GPO_PUSH_PULL);
            apply_outputs();text_queue("START TRACK-TRIAL v=20261019T2; p renews lease\r\n");
        }
        else text_queue("START REFUSED: idle + 3 valid fresh real frames required\r\n");
        break;
    case '4': case '5': case '6':
        if(control.mode==CAR_IDLE && !control.cfg.simulated) {
            gpio_init(CAR_LEFT_DIR,GPO,CAR_LEFT_FORWARD,GPO_PUSH_PULL);
            gpio_init(CAR_RIGHT_DIR,GPO,CAR_RIGHT_FORWARD,GPO_PUSH_PULL);
        }
        if(control_speed_test(&control,(uint8_t)(byte=='4'?1:(byte=='5'?2:3)),milliseconds))
            text_queue(byte=='4'?"LEFT SPEED TEST v=20261017I 4s target=80 cap=30%\r\n":
                       byte=='5'?"RIGHT SPEED TEST v=20261017I 4s target=80 cap=30%\r\n":
                                 "BOTH SPEED TEST v=20261017I 4s target=80 cap=30%\r\n");
        else text_queue("SPEED TEST REFUSED\r\n");
        break;
    case '1': case '2':
        if(control.mode==CAR_IDLE && !control.cfg.simulated) {
            /* PWM is zero in idle; restore GPIO mux and direction before motion. */
            gpio_init(CAR_LEFT_DIR,GPO,CAR_LEFT_FORWARD,GPO_PUSH_PULL);
            gpio_init(CAR_RIGHT_DIR,GPO,CAR_RIGHT_FORWARD,GPO_PUSH_PULL);
        }
        if(control_test(&control,(uint8_t)(byte-'0'),milliseconds))
            text_queue(byte=='1'?"LEFT MOTOR TEST 2s at 30%\r\n":"RIGHT MOTOR TEST 2s at 30%\r\n");
        else text_queue("TEST REFUSED\r\n");
        break;
    case '+': if(control.cfg.speed_target<60) control.cfg.speed_target+=5; break;
    case '-': if(control.cfg.speed_target>=5) control.cfg.speed_target-=5; break;
    case 'c': case 'l': case 'j':
        if(control.mode==CAR_IDLE) {
            float us=byte=='c'?CAR_SERVO_CENTER_US:(byte=='l'?CAR_SERVO_MIN_US:CAR_SERVO_MAX_US);
            servo_test_us=us; servo_test_until=milliseconds+1000;
            servo_test_active=1; car_servo_set_us(us);
        }
        break;
    default: break;
    }
    unlock(key);
    switch(byte) {
    case 'H': sustain_query(received);break;
    case 'Q': low_query(received);break;
    case 'D': {
        char line[120];
        snprintf(line,sizeof line,"CAL-CFG v=20261019C3 selected=%u step=500 cap=%u duration=2000 lease=500 guard=%u\r\n",(unsigned)calibration_selected,(unsigned)CAR_CALIBRATION_MAX_DUTY,(unsigned)CAR_CALIBRATION_ENCODER_LIMIT);
        text_queue(line);break;
    }
    case 'h':
#if CAR_COMM_DIAGNOSTIC
        text_queue("COMM DIAG: p status; q diagnostics; u C11 HIGH; v C11 LOW; s STOP; motion disabled\r\n");
#else
        text_queue("0 straight 10s (p lease); 7/8/9 ground straight/left/right 1s; d drive config; w static steering 30s motors=0; a tracking trial 3s (p lease); s stop/ack; 1/2 motor test; 4/5/6 left/right/both speed bench 4s; c center,l/j limits; +/- speed\r\no Otsu,t fixed,[/] threshold; g gray,b binary; p status;r camera retry;e/E exposure\r\nq UART8 diagnostics; UART1+UART8 115200\r\n");
#endif
        break;
    case 'p': key=lock();control_drive_keepalive(&control,received);unlock(key);telemetry();break;
    case 'q': wireless_diagnostic(); break;
    case 'z': servo_register_diagnostic(); break;
    case 'd': text_queue("DRIVE-CFG v=20261018N straight=7 left=8 right=9 duration=1000 settle=200 target=40 cap=3000 straight10=0 lease=500\r\n");tracking_configuration();break;
    case 'o': vision_cfg.otsu=1; break;
    case 't': vision_cfg.otsu=0; break;
    case '[': if(vision_cfg.fixed_threshold>=5) vision_cfg.fixed_threshold-=5; break;
    case ']': if(vision_cfg.fixed_threshold<=250) vision_cfg.fixed_threshold+=5; break;
    case 'g': case 'b':
        if(control.mode!=CAR_RUNNING && control.mode!=CAR_TEST && control.mode!=CAR_SPEED_TEST && control.mode!=CAR_DRIVE_TEST && control.mode!=CAR_STEER_TEST && control.mode!=CAR_CALIBRATION && control.mode!=CAR_SUSTAIN && control.mode!=CAR_LOW_SPEED && images_idle())
            image_requested=(uint8_t)(byte=='g'?1:2);
        break;
    case 'r': if(control.mode!=CAR_RUNNING && control.mode!=CAR_TEST && control.mode!=CAR_SPEED_TEST && control.mode!=CAR_DRIVE_TEST && control.mode!=CAR_STEER_TEST && control.mode!=CAR_CALIBRATION && control.mode!=CAR_SUSTAIN && control.mode!=CAR_LOW_SPEED) camera_init_once(); break;
    case 'e': case 'E':
#if !CAR_SIMULATED
        if(control.mode==CAR_IDLE && camera_ready) {
            if(byte=='e' && exposure<=1984) exposure+=64;
            if(byte=='E' && exposure>=128) exposure-=64;
            if(mt9v03x_set_exposure_time(exposure)) text_queue("EXPOSURE FAILED\r\n");
        }
#endif
        break;
    default: break;
    }
}
#if CAR_SIMULATED
static void generate_frame(void) {
    int direction=(int)((frame_count/100)%3)-1;
    for(int y=0;y<CAR_H;y++) {
        int center=CAR_W/2+direction*(CAR_H-y)/4;
        int half=15+y/2;
        for(int x=0;x<CAR_W;x++) gray[y*CAR_W+x]=(uint8_t)(x>=center-half && x<=center+half?210:30);
    }
}
#endif
static void display_frame(void) {
#if CAR_DISPLAY_ENABLED
    uint8_t mode,fault;
    uint32_t key=lock(); mode=(uint8_t)control.mode; fault=(uint8_t)control.fault; unlock(key);
#if CAR_COMM_DIAGNOSTIC
    char diag_line[17];
    tft180_show_string(0,0,"UART8 DIAG 1017B");
    snprintf(diag_line,sizeof diag_line,"RX %8lu",(unsigned long)wireless_received);
    tft180_show_string(0,16,diag_line);
    snprintf(diag_line,sizeof diag_line,"TX %8lu",(unsigned long)serial_tx[1].sent);
    tft180_show_string(0,32,diag_line);
    snprintf(diag_line,sizeof diag_line,"ERR%8lu",(unsigned long)wireless_errors);
    tft180_show_string(0,48,diag_line);
    snprintf(diag_line,sizeof diag_line,"RTS%u AB%u",(unsigned)gpio_get_level(CAR_WIRELESS_RTS),(unsigned)baud_handshake_result);
    tft180_show_string(0,64,diag_line);
#else
    tft180_show_gray_image(0,0,binary,CAR_W,CAR_H,128,82,0);
    for(int y=vision_cfg.last_row;y>=vision_cfg.first_row;y-=vision_cfg.row_step) {
        if(result.left[y]>=0) tft180_draw_point((uint16_t)(result.left[y]*128/CAR_W),(uint16_t)(y*82/CAR_H),RGB565_BLUE);
        if(result.right[y]>=0) tft180_draw_point((uint16_t)(result.right[y]*128/CAR_W),(uint16_t)(y*82/CAR_H),RGB565_BLUE);
        if(result.center[y]>=0) tft180_draw_point((uint16_t)(result.center[y]*128/CAR_W),(uint16_t)(y*82/CAR_H),RGB565_RED);
    }
#endif
    char line[17];
    snprintf(line,sizeof line,"M%u F%u V%u      ",mode,fault,result.valid); tft180_show_string(0,84,line);
    snprintf(line,sizeof line,"T%3u E%4d      ",result.threshold,(int)result.error); tft180_show_string(0,100,line);
    snprintf(line,sizeof line,"L%5d R%5d",measured_left,measured_right); tft180_show_string(0,116,line);
    snprintf(line,sizeof line,"FR%6lu       ",(unsigned long)(frame_count%1000000)); tft180_show_string(0,132,line);
#endif
}
void car_app_foreground(void) {
    uint32_t key=lock(); control.heartbeat_ms=milliseconds; unlock(key);
    /* Limit command work so a flood cannot starve capture and heartbeat. */
    for(unsigned i=0;i<8;i++) {
        key=lock();
        if(rx_tail==rx_head) { unlock(key); break; }
        uint32_t epoch=stop_epoch;
        uint32_t received=rx_received_ms[rx_tail];
        uint8_t byte=rx[rx_tail]; rx_tail=(uint8_t)((rx_tail+1u)%64u);
        unlock(key);
        command(byte,epoch,received);
    }
    uint8_t new_frame=0;
    uint32_t captured=milliseconds,frame_epoch=stop_epoch;
#if CAR_SIMULATED
    if((uint32_t)(milliseconds-last_generated)>=20) {
        last_generated=milliseconds; generate_frame(); new_frame=1;
    }
#else
    if(camera_ready && mt9v03x_finish_flag) {
        /* Vendor callback swaps two CSI buffers. Capture pointer and clear flag
           atomically, then copy before the following frame reuses the buffer. */
        uint8_t *source;uint32_t sequence;
        key=lock();source=mt9v03x_image[0];sequence=capture_sequence;captured=capture_ms;
        frame_epoch=stop_epoch;mt9v03x_finish_flag=0;unlock(key);
        if(sequence!=consumed_sequence) {
            dropped_frames+=(uint32_t)(sequence-consumed_sequence)-1u;
            consumed_sequence=sequence;
            memcpy(gray,source,sizeof gray);
            key=lock();
            if(sequence==capture_sequence)new_frame=1;
            else dropped_frames++; /* Buffer ownership advanced during copy: reject it. */
            unlock(key);
        }
    }
#endif
    if(new_frame) {
        uint32_t began=milliseconds;
        vision_process(&vision,&vision_cfg,gray,binary,&result);frame_count++;
        key=lock();processing_ms=milliseconds-began;last_capture_age=milliseconds-captured;
        if(frame_epoch==stop_epoch)control_frame_at(&control,&result,captured,milliseconds);
        unlock(key);
    }
    if(image_requested && frame_count && texts_idle() && images_idle()) {
        memcpy(image_tx,image_requested==1?gray:binary,sizeof image_tx);
        serial_tx[0].image_position=0; serial_tx[1].image_position=0;
        image_length=sizeof image_tx+4; image_requested=0;
    }
    if((uint32_t)(milliseconds-last_screen)>=CAR_SCREEN_PERIOD_MS) {
        last_screen=milliseconds; display_frame();
    }
    if((uint32_t)(milliseconds-last_telemetry)>=((control.mode==CAR_SPEED_TEST || control.mode==CAR_DRIVE_TEST)?50u:(control.mode==CAR_RUNNING?100u:CAR_TELEMETRY_PERIOD_MS))) {
        last_telemetry=milliseconds; telemetry();
#if CAR_COMM_DIAGNOSTIC
        wireless_diagnostic();
#endif
    }
    uart_flush_some();
}
