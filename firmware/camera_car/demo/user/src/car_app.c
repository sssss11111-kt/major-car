/* SeekFree RT1064 adapters for the C-car application. GPL-3.0-or-later. */
#include "zf_common_headfile.h"
#include "car_config.h"
#include "car_core.h"
#include "car_app.h"
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
static volatile uint32_t stop_epoch;
static char text_tx[1024];
static unsigned text_head,text_tail;
static uint32_t image_position,image_length,frame_count;
static uint8_t image_requested,camera_ready;
static uint32_t last_screen,last_telemetry;
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
static void text_queue(const char *text) {
    while(*text) {
        unsigned next=(text_head+1)%sizeof text_tx;
        if(next==text_tail) break; /* drop debug output rather than block control */
        text_tx[text_head]=*text++; text_head=next;
    }
}
static void uart_flush_some(void) {
    /* At most 16 bytes per foreground iteration, never wait on the UART. */
    for(unsigned i=0;i<16;i++) {
        if(!(LPUART_GetStatusFlags(LPUART1)&kLPUART_TxDataRegEmptyFlag)) break;
        uint8_t byte;
        if(image_position<image_length) {
            byte=image_position<4?image_header[image_position]:image_tx[image_position-4];
            image_position++;
        } else if(text_tail!=text_head) {
            byte=(uint8_t)text_tx[text_tail]; text_tail=(text_tail+1)%sizeof text_tx;
        } else break;
        LPUART_WriteByte(LPUART1,byte);
    }
}
static void servo_write(float us) {
    if(us<CAR_SERVO_MIN_US) us=CAR_SERVO_MIN_US;
    if(us>CAR_SERVO_MAX_US) us=CAR_SERVO_MAX_US;
    pwm_set_duty(CAR_SERVO_PWM,(uint32_t)(us*CAR_SERVO_HZ*PWM_DUTY_MAX/1000000.0f));
}
static void apply_outputs(void) {
    pwm_set_duty(CAR_LEFT_PWM,(uint32_t)control.duty[0]);
    pwm_set_duty(CAR_RIGHT_PWM,(uint32_t)control.duty[1]);
    servo_write(control.servo_us);
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
        else servo_write(servo_test_us);
    }
}
void car_app_uart_rx_isr(uint8_t byte) {
    /* Stop is executed in the RX interrupt, independent of graphics/foreground. */
    if(byte=='s' || byte=='S') {
        servo_test_active=0; control_stop(&control); apply_outputs();
        stop_epoch++;
        rx_tail=rx_head; /* cancel the entire pre-stop backlog, even when full */
        return;
    }
    uint8_t next=(uint8_t)((rx_head+1u)%64u);
    if(next!=rx_tail) { rx[rx_head]=byte; rx_head=next; }
}
static void telemetry(void) {
    char line[220];
    control_state_t snapshot;
    uint32_t key=lock(); snapshot=control; unlock(key);
    snprintf(line,sizeof line,
        "ms=%lu mode=%u fault=%u frames=%lu valid=%u pair=%u/%u thr=%u err10=%d enc=%d,%d pwm=%u,%u us=%u speed=%u\r\n",
        (unsigned long)milliseconds,(unsigned)snapshot.mode,(unsigned)snapshot.fault,
        (unsigned long)frame_count,result.valid,result.paired_rows,result.scanned_rows,
        result.threshold,(int)(result.error*10),measured_left,measured_right,
        (unsigned)snapshot.duty[0],(unsigned)snapshot.duty[1],
        (unsigned)snapshot.servo_us,(unsigned)snapshot.cfg.speed_target);
    text_queue(line);
}
static void camera_init_once(void) {
    uint32_t key=lock(); control_stop(&control); apply_outputs(); unlock(key);
    servo_test_active=0; frame_count=0; memset(&result,0,sizeof result);
    vision_reset(&vision);
#if CAR_SIMULATED
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
    cfg.steer_kp=CAR_STEER_KP; cfg.steer_kd=CAR_STEER_KD; cfg.steer_sign=CAR_STEER_SIGN;
    cfg.servo_center_us=CAR_SERVO_CENTER_US;
    cfg.servo_min_us=CAR_SERVO_MIN_US; cfg.servo_max_us=CAR_SERVO_MAX_US;
    cfg.frame_timeout_ms=CAR_FRAME_TIMEOUT_MS;
    cfg.heartbeat_timeout_ms=CAR_FOREGROUND_TIMEOUT_MS; cfg.simulated=CAR_SIMULATED;
    control_init(&control,&cfg); vision_reset(&vision); vision_config_default(&vision_cfg);
    gpio_init(CAR_LEFT_DIR,GPO,CAR_LEFT_FORWARD,GPO_PUSH_PULL);
    gpio_init(CAR_RIGHT_DIR,GPO,CAR_RIGHT_FORWARD,GPO_PUSH_PULL);
    pwm_init(CAR_LEFT_PWM,CAR_MOTOR_HZ,0); pwm_init(CAR_RIGHT_PWM,CAR_MOTOR_HZ,0);
    pwm_init(CAR_SERVO_PWM,CAR_SERVO_HZ,0); servo_write(CAR_SERVO_CENTER_US);
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
    text_queue("C-CAR RT1064 SeekFree V3.5.5: h help; s STOP\r\n");
#if CAR_DISPLAY_ENABLED
    tft180_set_dir(TFT180_PORTAIT); tft180_init();
    tft180_show_string(0,0,"C-CAR INIT");
#endif
    camera_init_once();
}
static void command(uint8_t byte,uint32_t epoch) {
    uint32_t key=lock();
    /* Also cancel a command already dequeued when an RX stop preempted us. */
    if(epoch!=stop_epoch) { unlock(key); return; }
    switch(byte) {
    case 's': case 'S': servo_test_active=0; control_stop(&control); apply_outputs(); break;
    case 'a':
        if(camera_ready && control_start(&control,milliseconds)) text_queue("START\r\n");
        else text_queue("START REFUSED: idle + 3 valid fresh real frames required\r\n");
        break;
    case '1': case '2':
        if(control_test(&control,(uint8_t)(byte-'0'),milliseconds)) text_queue("MOTOR TEST 1s at 5%\r\n");
        else text_queue("TEST REFUSED\r\n");
        break;
    case '+': if(control.cfg.speed_target<60) control.cfg.speed_target+=5; break;
    case '-': if(control.cfg.speed_target>=5) control.cfg.speed_target-=5; break;
    case 'c': case 'l': case 'j':
        if(control.mode==CAR_IDLE) {
            float us=byte=='c'?CAR_SERVO_CENTER_US:(byte=='l'?CAR_SERVO_MIN_US:CAR_SERVO_MAX_US);
            servo_test_us=us; servo_test_until=milliseconds+1000;
            servo_test_active=1; servo_write(us);
        }
        break;
    default: break;
    }
    unlock(key);
    switch(byte) {
    case 'h': text_queue("a start; s stop/ack; 1/2 motor test; c center,l/j limits; +/- speed\r\no Otsu,t fixed,[/] threshold; g gray,b binary; p status;r camera retry;e/E exposure\r\n"); break;
    case 'p': telemetry(); break;
    case 'o': vision_cfg.otsu=1; break;
    case 't': vision_cfg.otsu=0; break;
    case '[': if(vision_cfg.fixed_threshold>=5) vision_cfg.fixed_threshold-=5; break;
    case ']': if(vision_cfg.fixed_threshold<=250) vision_cfg.fixed_threshold+=5; break;
    case 'g': case 'b':
        if(control.mode!=CAR_RUNNING && control.mode!=CAR_TEST && image_position>=image_length)
            image_requested=(uint8_t)(byte=='g'?1:2);
        break;
    case 'r': if(control.mode!=CAR_RUNNING && control.mode!=CAR_TEST) camera_init_once(); break;
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
    tft180_show_gray_image(0,0,binary,CAR_W,CAR_H,128,82,0);
    for(int y=vision_cfg.last_row;y>=vision_cfg.first_row;y-=vision_cfg.row_step) {
        if(result.left[y]>=0) tft180_draw_point((uint16_t)(result.left[y]*128/CAR_W),(uint16_t)(y*82/CAR_H),RGB565_BLUE);
        if(result.right[y]>=0) tft180_draw_point((uint16_t)(result.right[y]*128/CAR_W),(uint16_t)(y*82/CAR_H),RGB565_BLUE);
        if(result.center[y]>=0) tft180_draw_point((uint16_t)(result.center[y]*128/CAR_W),(uint16_t)(y*82/CAR_H),RGB565_RED);
    }
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
        uint8_t byte=rx[rx_tail]; rx_tail=(uint8_t)((rx_tail+1u)%64u);
        unlock(key);
        command(byte,epoch);
    }
    uint8_t new_frame=0;
#if CAR_SIMULATED
    if((uint32_t)(milliseconds-last_generated)>=20) {
        last_generated=milliseconds; generate_frame(); new_frame=1;
    }
#else
    if(camera_ready && mt9v03x_finish_flag) {
        /* Vendor callback swaps two CSI buffers. Capture pointer and clear flag
           atomically, then copy before the following frame reuses the buffer. */
        uint8_t *source;
        key=lock(); source=mt9v03x_image[0]; mt9v03x_finish_flag=0; unlock(key);
        memcpy(gray,source,sizeof gray);
        new_frame=1;
    }
#endif
    if(new_frame) {
        vision_process(&vision,&vision_cfg,gray,binary,&result); frame_count++;
        key=lock(); control_frame(&control,&result,milliseconds); unlock(key);
    }
    if(image_requested && frame_count && text_tail==text_head && image_position>=image_length) {
        memcpy(image_tx,image_requested==1?gray:binary,sizeof image_tx);
        image_position=0; image_length=sizeof image_tx+4; image_requested=0;
    }
    if((uint32_t)(milliseconds-last_screen)>=CAR_SCREEN_PERIOD_MS) {
        last_screen=milliseconds; display_frame();
    }
    if((uint32_t)(milliseconds-last_telemetry)>=CAR_TELEMETRY_PERIOD_MS && image_position>=image_length) {
        last_telemetry=milliseconds; telemetry();
    }
    uart_flush_some();
}
