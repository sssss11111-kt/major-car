#include "zf_common_headfile.h"
#include "img_process.h"

// 编码器引脚信息
// 请确保两轮编码器前进时回传值都为正数，若回传为负请isr.c中encoder_get_count前修改正负号
// 左轮编码器
#define ENCODER_1 (QTIMER1_ENCODER1)
#define ENCODER_1_A (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B (QTIMER1_ENCODER1_CH2_C1)
// 右轮编码器
#define ENCODER_2 (QTIMER1_ENCODER2)
#define ENCODER_2_A (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B (QTIMER1_ENCODER2_CH2_C24)

// 电机参数设置
// !!! 请确保dir和pwm脚不要弄反了，否则电机会疯转 !!!
// 左轮电机
#define MOTOR1_PWM (PWM2_MODULE0_CHA_C6)
#define MOTOR1_DIR (C11)
// 左电机前进需要的DIR脚的电平 (GPIO_HIGH or GPIO_LOW) 请自行测试
#define MOTOR1_FORWARD_DIR_LEVEL (GPIO_HIGH)
// 右轮电机
#define MOTOR2_PWM (PWM2_MODULE1_CHA_C8)
#define MOTOR2_DIR (C9)
// 右电机前进需要的DIR脚的电平 (GPIO_HIGH or GPIO_LOW) 请自行测试
#define MOTOR2_FORWARD_DIR_LEVEL (GPIO_LOW)


// 舵机参数设置
#define SERVO_MOTOR_PWM (PWM4_MODULE2_CHA_C30) // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ (50)                  // 定义主板上舵机频率  请务必注意范围 50-300
#define SERVO_MOTOR_L_MAX (124)  // 定义主板上舵机活动范围(左打方向的极限值) 角度 自行标定
#define SERVO_MOTOR_R_MAX (61) // 定义主板上舵机活动范围(右打方向的极限值) 角度 自行标定
#define SERVO_MOTOR_MIDDLE (92.5f) // Independently calibrated center
#define SERVO_DIR (SERVO_MOTOR_L_MAX>SERVO_MOTOR_R_MAX?-1.f:1.f) // 根据左右duty的大小自动决定舵机方向

// 以下宏在初步测试例程时不可更改，后面若需更改舵机算法可以自行更改
// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
//
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
//
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
//
// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))

#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#error "SERVO_MOTOR_FREQ ERROE!"
#endif

extern volatile uint32 trial_ticks, frame_tick, good_frames, frame_count;
extern volatile uint8 trial_running, trial_fault;
extern volatile int trial_left_pwm, trial_right_pwm;
extern volatile uint32 camera_ready_cycle;
void trial_tick(void);

/* Full hardware duty range; actual output remains speed-feedback controlled. */
#define TRIAL_DRIVE_FEEDFORWARD (1500.0f)
#define TRIAL_DRIVE_MAX (10000.0f)

#define TRIAL_DRIVE_TARGET (45.0f)
#define TRIAL_DRIVE_DEGRADED_TARGET (24.0f)
