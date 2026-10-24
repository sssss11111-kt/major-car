#include "single_edge.h"
#include "auto_start.h"
#include "main.h" // 所有引脚信息更改在main.h里改宏

// *************************** 例程硬件连接说明 ***************************
/*

            连接好电机、主板、驱动板、摄像头、编码器
            (请确保全部正确连接后在使用此例程，所有外设均会使用)
*/
// *************************** 例程测试说明 ***************************
/*
            本例程包含功能：
            1.初始化摄像头并采集图像进行大津法二值化
            2.初始化屏幕并显示关键信息
            3.根据偏差计算最基本的舵机打角
            4.运用最基本pid参数和pid算法使电机旋转 (pid参数在pid.c中)


            注意！注意！注意！注意！
            注意！注意！注意！注意！
            注意！注意！注意！注意！

            1.请仔细阅读main.h的所有注释，确保所有宏都和自己的硬件连接对应
            2.若屏幕显示二值化后的图像有缺陷，可能不能正常走直线，
            可以自行更改二值化算法或找光环境良好且均匀的环境测试例程（例程不提供更多的二值化算法）

*/

// **************************** 代码区域 ****************************

// 舵机动作角度中值
const float servo_motor_duty_middle = (SERVO_MOTOR_R_MAX + SERVO_MOTOR_L_MAX) / 2.f; 

static volatile uint8 wireless_ready = 0;
static volatile uint8 trial_bench = 0;
static volatile uint8 snapshot_requested = 0;
static volatile uint8 vision_mode=0, vision_degraded=0, model_good=0, control_center=94;
static volatile uint32 vision_since=0;
static auto_start_t auto_start_state = {0}; /* Written only by the 10 ms ISR. */
static volatile uint8 auto_phase = AUTO_WAIT, auto_run = 0;
static volatile uint32 auto_center_since = 0;

/* Diagnostic output must never wait for radio flow control while driving.
 * SNAP retains the driver's full transfer routine while start is inhibited. */
static uint32 telemetry_send(const char *text)
{
    uint32 remaining = (uint32)strlen(text);
    if (!wireless_ready) return remaining;
    while (remaining) {
        uint32 count = remaining > 30U ? 30U : remaining;
        if (gpio_get_level(WIRELESS_UART_RTS_PIN)) return remaining;
        uart_write_buffer(WIRELESS_UART_INDEX, (const uint8 *)text, count);
        text += count;
        remaining -= count;
    }
    return 0;
}

/* Capture the first 1.28 seconds at control rate, including the stop tick.
 * VAL3 is a register readback, not a measurement at the connector. */
typedef struct {
    uint32 tick, run, fault, stall, p1, p2, val1, val3_l, val3_r;
    uint32 outen, mask, mctrl, fsts, source, dir;
    int32 e1, e2;
} drive_trace_t;
volatile drive_trace_t drive_trace[128];
volatile uint32 drive_trace_count=0;
static uint8 drive_trace_frozen=0;
static void drive_trace_report(void)
{
    static uint32 sent=0;
    if (!drive_trace_frozen || sent>=drive_trace_count) return;
    volatile drive_trace_t *r=&drive_trace[sent++];
    char b[256];
    sprintf(b,"DRV t=%lu run=%lu fault=%lu stall=%lu p=%lu,%lu val=%lu,%lu,%lu out=%lx mask=%lx ctrl=%lx fs=%lx src=%lx dir=%lx e=%ld,%ld\r\n",
        r->tick,r->run,r->fault,r->stall,r->p1,r->p2,r->val1,r->val3_l,r->val3_r,
        r->outen,r->mask,r->mctrl,r->fsts,r->source,r->dir,r->e1,r->e2);
    telemetry_send(b);
}


void Init()
{

    // 初始化flash, 储存参数. 一个扇区有8页, 一页可以储存4096字节, 一个参数占4个字节, 因此一页最多只能存64个参数
    flash_init();

    // tft180_set_dir(TFT180_CROSSWISE);                                           // 需要先横屏 不然显示不下
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();

    // mt9v03x摄像头初始化
    while (1)
    {
        if (mt9v03x_init()) {
            tft180_show_string(0, 16, "mt9v03x reinit.");
            telemetry_send("C11LAP11 CAMERA_RETRY\r\n");
        }
        else
            break;
        system_delay_ms(1000);
    }
    tft180_show_string(0, 16, "init success.");

    // 编码器初始化
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式

    // 电机初始化
    pwm_init(MOTOR1_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR1_DIR, GPO, MOTOR1_FORWARD_DIR_LEVEL, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR2_DIR, GPO, MOTOR2_FORWARD_DIR_LEVEL, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高

    // 舵机
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);

    // PID初始化
    My_Pid_Init();

    // 定时器0初始化，10ms可调
    pit_ms_init(PIT_CH0, 10);
}

uint8_t image[MT9V03X_H][MT9V03X_W];
static uint8_t raw_image[MT9V03X_H][MT9V03X_W];
short hist_gram[256];
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];


/* SNAP is accepted only while stopped. Main owns these buffers and does not
 * consume another camera frame until the complete snapshot has been sent.
 * CSI DMA uses separate buffers; PIT/RX remain enabled and RUN/BENCH are gated. */
#define SNAP_SIZE (MT9V03X_H * MT9V03X_W + 3 * MT9V03X_H)
static uint8 snapshot_byte(uint32 i)
{
    if (i < sizeof(raw_image)) return ((uint8 *)raw_image)[i];
    i -= sizeof(raw_image);
    if (i < MT9V03X_H) return left_line[i];
    i -= MT9V03X_H;
    if (i < MT9V03X_H) return mid_line[i];
    return right_line[i-MT9V03X_H];
}
static void snapshot_send(uint8 threshold)
{
    uint32 crc=0xffffffffU, capture_tick=frame_tick, capture_frame=frame_count;
    char line[128];
    static const char hex[]="0123456789abcdef";
    for (uint32 i=0; i<SNAP_SIZE; ++i) {
        crc ^= snapshot_byte(i);
        for (uint8 b=0;b<8;++b) crc=(crc>>1)^((crc&1U)?0xedb88320U:0U);
    }
    crc ^= 0xffffffffU;
    sprintf(line,"SNAP9 w=%u h=%u n=%u threshold=%u frame=%lu tick=%lu crc=%08lx\r\n",
        MT9V03X_W,MT9V03X_H,SNAP_SIZE,threshold,(unsigned long)capture_frame,
        (unsigned long)capture_tick,(unsigned long)crc);
    if (wireless_uart_send_string(line)) goto failed;
    for (uint32 off=0;off<SNAP_SIZE;off+=32) {
        uint32 count=SNAP_SIZE-off; if(count>32) count=32;
        int n=sprintf(line,"PX9 off=%lu data=",(unsigned long)off);
        for(uint32 k=0;k<count;++k) {
            uint8 v=snapshot_byte(off+k);line[n++]=hex[v>>4];line[n++]=hex[v&15];
        }
        line[n++]='\r';line[n++]='\n';line[n]=0;
        if(wireless_uart_send_string(line)) goto failed;
    }
    sprintf(line,"END9 frame=%lu crc=%08lx\r\n",(unsigned long)capture_frame,(unsigned long)crc);
    wireless_uart_send_string(line);
    snapshot_requested=0;
    return;
failed:
    wireless_uart_send_string("\r\nSNAP9_ABORT\r\n");
    snapshot_requested=0;
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    debug_init();                  // 调试端口初始化
    system_delay_ms(300);          // 等待主板其他外设上电完成

    wireless_ready = (wireless_uart_init() == 0); /* Supplied wireless driver: RTS and IRQ RX FIFO. */
    telemetry_send("C11LAP11 AUTO BOOT\r\n");
    Init();
    telemetry_send("C11LAP11 AUTO READY\r\n");
    tft180_clear();
    tft180_show_string(0, 0, wireless_ready ? "W:OK GRAY/BINARY" : "W:FAIL GRAY/BIN");
    interrupt_global_enable(0);

    uint32 report_tick = 0, display_tick = 0;
    while (1)
    {
        if (trial_ticks-report_tick >= 10) {
            char report[200]; report_tick=trial_ticks;
            sprintf(report, "C11LAP11 t=%lu run=%u fault=%u frame=%lu good=%lu age=%lu e1=%d e2=%d p1=%d p2=%d\r\n",
                (unsigned long)trial_ticks,trial_running,trial_fault,(unsigned long)frame_count,
                (unsigned long)good_frames,(unsigned long)(trial_ticks-frame_tick),
                encoder_data_1,encoder_data_2,trial_left_pwm,trial_right_pwm);
            telemetry_send(report);
            char vision_report[100];
            sprintf(vision_report,"VIS10 mode=%u center=%u degraded=%u model=%u single_age=%lu\r\n",
                vision_mode,control_center,vision_degraded,model_good,
                (unsigned long)(vision_degraded ? trial_ticks-vision_since : 0));
            telemetry_send(vision_report);
            char auto_report[80];
            uint8 phase = auto_phase;
            uint32 elapsed = trial_ticks - auto_center_since;
            uint32 remaining = (phase == AUTO_CENTER && elapsed < AUTO_CENTER_TICKS)
                               ? AUTO_CENTER_TICKS - elapsed : 0;
            sprintf(auto_report,"AUTO11 phase=%u wait_ms=%lu autonomous=%u\r\n",
                phase, (unsigned long)(remaining * 10U), auto_run);
            telemetry_send(auto_report);
            drive_trace_report();
        }
        //  mt9v03x摄像头
        if (mt9v03x_finish_flag)
        {
            // 另寻空间将图像保存下来，以免产生因读写冲突带来的未知后果
            /* Keep CSI's completed-buffer pointer stable during the short copy.
             * Clear only the frame being consumed, never a later completion. */
            uint32 camera_mask = interrupt_global_disable();
            memcpy(raw_image, (const void *)mt9v03x_image, sizeof(raw_image));
            mt9v03x_finish_flag = 0;
            interrupt_global_enable(camera_mask);
            memcpy(image, raw_image, sizeof(image));
            // 获取直方图
            get_hist_gram((uint8_t *)image, MT9V03X_H, MT9V03X_W, hist_gram);
            unsigned char threshold = get_threshold_otsu(hist_gram);
            // 二值化处理
            binaryzation_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold);
            // 边界线寻找
            auxiliary_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, 127, left_line, mid_line, right_line);

            se_result_t vision=single_edge_update((uint8_t *)image,left_line,mid_line,right_line,
                trial_ticks,trial_running && !trial_bench);
            /* Publish mode, lease and freshness coherently to the PIT ISR. */
            uint32 vision_mask=interrupt_global_disable();
            vision_mode=vision.mode; vision_degraded=vision.degraded;
            vision_since=vision.since; control_center=vision.center;
            if (vision.mode) { if (good_frames<100) ++good_frames; }
            else good_frames=0;
            if (!trial_running) {
                if(vision.mode==1 && vision.model_matches) {if(model_good<100) ++model_good;}
                else model_good=0;
            }
            frame_tick=trial_ticks; ++frame_count;
            interrupt_global_enable(vision_mask);
            if (snapshot_requested && !trial_running) snapshot_send(threshold);
            for (uint8_t _i = 0; _i < MT9V03X_H; ++_i)
            {
                // 将边界线也显示出来
                image[_i][left_line[_i]] = 0;
                image[_i][mid_line[_i]] = 0;
                image[_i][right_line[_i]] = 0;
            }
            // 显示图像
            /* LCD rendering is rate-limited below, after steering is updated. */

            // 取图像下1/4处做误差判断
            float mid_err = (MT9V03X_W/2) - control_center;
            // 舵机根据误差乘以系数打角
            float tmp_duty = servo_motor_duty_middle - mid_err * 0.5 * SERVO_DIR;
            // 限幅
            tmp_duty = MAX(tmp_duty,MIN(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
            tmp_duty = MIN(tmp_duty,MAX(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
            if (trial_running && !trial_bench && vision_mode)
                pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(tmp_duty));

            /* 5 Hz LCD: no full-screen clear, no text over the pictures.
             * Both pictures use the same captured frame at 80 x 51 pixels. */
            if (trial_ticks-display_tick >= 20) {
                char line[24];
                static const char * const phases[] = {"AUTO WAIT       ", "CENTER 1s       ", "AUTO RUN        ", "STOP RESET      "};
                unsigned int sum=0, white=0;
                uint8 low=255, high=0;
                display_tick=trial_ticks;
                tft180_show_string(0,0,phases[auto_phase]);
                for (uint32 k=0; k<sizeof(raw_image); ++k) {
                    uint8 v=((uint8 *)raw_image)[k];
                    if(v<low) low=v;
                    if(v>high) high=v;
                    sum+=v;
                    if(v>threshold) ++white;
                }
                tft180_show_gray_image(0,16,(uint8 *)raw_image,MT9V03X_W,MT9V03X_H,80,51,0);
                tft180_show_gray_image(80,16,(uint8 *)image,MT9V03X_W,MT9V03X_H,80,51,0);
                sprintf(line,"T%3u M%u C%3u A%u",threshold,vision_mode,control_center,model_good>=5);
                tft180_show_string(0,72,line);
                sprintf(line,"F%6lu V%3lu",(unsigned long)frame_count,(unsigned long)good_frames);
                tft180_show_string(0,88,line);
                sprintf(line,"R%u X%u E%4d/%4d",trial_running,trial_fault,encoder_data_1,encoder_data_2);
                tft180_show_string(0,104,line);
                char camera_report[128];
                sprintf(camera_report,"CAM2 frame=%lu threshold=%u min=%u max=%u mean=%u white=%u\r\n",
                    (unsigned long)frame_count,threshold,low,high,sum/(unsigned int)sizeof(raw_image),
                    white*100/(unsigned int)sizeof(raw_image));
                telemetry_send(camera_report);
            }
        }
    }
}

/* Automatic start: reliable track -> centered steering for 1 s -> run.
 * STOP latches until reset. BENCH retains its 2 s limit and K lease.
 * Autonomous running does not require a computer or a wireless heartbeat. */
volatile uint32 trial_ticks=0, frame_tick=0, good_frames=0, frame_count=0;
volatile uint8 trial_running=0, trial_fault=0;
volatile int trial_left_pwm=0, trial_right_pwm=0;
static uint32 start_tick=0, lease_tick=0;
static uint8 used=0, stall_ticks=0;
void trial_tick(void)
{
    static char cmd[16]; static uint8 len=0;
    uint8 ch, n=0;
    ++trial_ticks;
    while (n++ < 32 && wireless_uart_read_buffer(&ch, 1)) {
        if (ch=='\n' || ch=='\r') {
            cmd[len]=0;
            if (!strcmp(cmd,"STOP")) { trial_running=0; if (!trial_fault) trial_fault=1; }
            else if (!strcmp(cmd,"SNAP") && !trial_running && !snapshot_requested) snapshot_requested=1;
            else if (!strcmp(cmd,"K") && trial_running) lease_tick=trial_ticks;
            else if (wireless_ready && !snapshot_requested && !used && !trial_fault &&
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
                                snapshot_requested,
                                trial_fault || (used && !trial_running) || trial_bench);
    auto_center_since = auto_start_state.center_since;
    if (auto_phase == AUTO_CENTER) {
        /* Hold the calibrated command midpoint throughout the full delay.
         * Main-loop steering is enabled only after trial_running becomes true. */
        pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_motor_duty_middle));
    }
    if (auto_phase == AUTO_STARTED && !used) {
        auto_run = 1; trial_bench = 0;
        used = 1; trial_running = 1; start_tick = lease_tick = trial_ticks;
        stall_ticks = 0;
        My_Pid_Init();
    }
    if (trial_running) {
        if (trial_ticks-start_tick>=(trial_bench ? 200U : 12000U)) { trial_running=0; trial_fault=2; }
        else if (!auto_run && trial_ticks-lease_tick>=50) { trial_running=0; trial_fault=3; }
        else if (!trial_bench && (trial_ticks-frame_tick>=20 || !good_frames ||
                    (vision_degraded && trial_ticks-vision_since>=SE_TIMEOUT_TICKS))) { trial_running=0; trial_fault=4; }
        else if (encoder_data_1 < -5 || encoder_data_2 < -5 ||
                 encoder_data_1 > 150 || encoder_data_2 > 150) { trial_running=0; trial_fault=5; }
        else if (trial_ticks-start_tick>30) {
            if (encoder_data_1<2 || encoder_data_2<2) ++stall_ticks; else stall_ticks=0;
            if (stall_ticks>=20) { trial_running=0; trial_fault=6; }
        }
    }
    gpio_set_level(MOTOR1_DIR,MOTOR1_FORWARD_DIR_LEVEL);
    gpio_set_level(MOTOR2_DIR,MOTOR2_FORWARD_DIR_LEVEL);
    if (trial_running) {
        float effective_target=(!trial_bench && vision_degraded) ? 20.0f : speed_target;
        speed_real=encoder_data_1;
        speed_pwm=PidLocCtrl(&speed_pid_l,effective_target-speed_real,1.f);
        trial_left_pwm=(int)constrain_float(TRIAL_DRIVE_FEEDFORWARD+speed_pwm,0,TRIAL_DRIVE_MAX);
        speed_real=encoder_data_2;
        speed_pwm=PidLocCtrl(&speed_pid_r,effective_target-speed_real,1.f);
        trial_right_pwm=(int)constrain_float(TRIAL_DRIVE_FEEDFORWARD+speed_pwm,0,TRIAL_DRIVE_MAX);
    } else { trial_left_pwm=trial_right_pwm=0; speed_pwm=0; }
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
