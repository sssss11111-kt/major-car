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
static volatile uint32_t stop_epoch;
typedef struct {
    char text[512];
    unsigned head,tail;
    uint32_t image_position,sent,dropped_lines;
} serial_tx_t;
static serial_tx_t serial_tx[2]; /* UART1 and UART8 advance independently. */
static uint32_t image_length,frame_count;
static volatile uint32_t wireless_received,wireless_errors;
static volatile uint8_t baud_handshake_active,baud_ack_count,baud_ack[3];
static uint8_t baud_handshake_result; /* 0 not attempted, 1 verified ACK, 2 failed */
static uint8_t image_requested,camera_ready;
static volatile uint8_t outputs_ready;
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
    if(next!=rx_tail) { rx[rx_head]=byte; rx_head=next; }
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
        (unsigned)car_servo_get_us(),(unsigned)snapshot.cfg.speed_target);
    text_queue(line);
}
static void camera_init_once(void) {
    uint32_t key=lock(); control_stop(&control); apply_outputs(); unlock(key);
    servo_test_active=0; frame_count=0; memset(&result,0,sizeof result);
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
    cfg.steer_kp=CAR_STEER_KP; cfg.steer_kd=CAR_STEER_KD; cfg.steer_sign=CAR_STEER_SIGN;
    cfg.servo_center_us=CAR_SERVO_CENTER_US;
    cfg.servo_min_us=CAR_SERVO_MIN_US; cfg.servo_max_us=CAR_SERVO_MAX_US;
    cfg.frame_timeout_ms=CAR_FRAME_TIMEOUT_MS;
    cfg.heartbeat_timeout_ms=CAR_FOREGROUND_TIMEOUT_MS;
    cfg.simulated=(CAR_SIMULATED || CAR_COMM_DIAGNOSTIC);
    control_init(&control,&cfg); vision_reset(&vision); vision_config_default(&vision_cfg);
    gpio_init(CAR_LEFT_DIR,GPO,CAR_LEFT_FORWARD,GPO_PUSH_PULL);
    gpio_init(CAR_RIGHT_DIR,GPO,CAR_RIGHT_FORWARD,GPO_PUSH_PULL);
    pwm_init(CAR_LEFT_PWM,CAR_MOTOR_HZ,0); pwm_init(CAR_RIGHT_PWM,CAR_MOTOR_HZ,0);
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
    text_queue("C-CAR RT1064 SeekFree V3.5.5: h help; s STOP\r\n");
    wireless_diagnostic();
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
#if CAR_COMM_DIAGNOSTIC
    if(byte!='p' && byte!='q' && byte!='h' && byte!='s' && byte!='S') {
        unlock(key); return;
    }
#endif
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
            servo_test_active=1; car_servo_set_us(us);
        }
        break;
    default: break;
    }
    unlock(key);
    switch(byte) {
    case 'h':
#if CAR_COMM_DIAGNOSTIC
        text_queue("COMM DIAG: p status; q UART8 diagnostics; s STOP; motion disabled\r\n");
#else
        text_queue("a start; s stop/ack; 1/2 motor test; c center,l/j limits; +/- speed\r\no Otsu,t fixed,[/] threshold; g gray,b binary; p status;r camera retry;e/E exposure\r\nq UART8 diagnostics; UART1+UART8 115200\r\n");
#endif
        break;
    case 'p': telemetry(); break;
    case 'q': wireless_diagnostic(); break;
    case 'o': vision_cfg.otsu=1; break;
    case 't': vision_cfg.otsu=0; break;
    case '[': if(vision_cfg.fixed_threshold>=5) vision_cfg.fixed_threshold-=5; break;
    case ']': if(vision_cfg.fixed_threshold<=250) vision_cfg.fixed_threshold+=5; break;
    case 'g': case 'b':
        if(control.mode!=CAR_RUNNING && control.mode!=CAR_TEST && images_idle())
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
    if(image_requested && frame_count && texts_idle() && images_idle()) {
        memcpy(image_tx,image_requested==1?gray:binary,sizeof image_tx);
        serial_tx[0].image_position=0; serial_tx[1].image_position=0;
        image_length=sizeof image_tx+4; image_requested=0;
    }
    if((uint32_t)(milliseconds-last_screen)>=CAR_SCREEN_PERIOD_MS) {
        last_screen=milliseconds; display_frame();
    }
    if((uint32_t)(milliseconds-last_telemetry)>=CAR_TELEMETRY_PERIOD_MS) {
        last_telemetry=milliseconds; telemetry();
#if CAR_COMM_DIAGNOSTIC
        wireless_diagnostic();
#endif
    }
    uart_flush_some();
}
