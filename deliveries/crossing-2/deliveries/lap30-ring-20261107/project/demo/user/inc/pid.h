#ifndef _PID_H
#define _PID_H

#include "zf_common_headfile.h"

typedef struct
{
    float                kp;                    //P
    float                ki;                    //I
    float                kd;                    //D
    float                imax;                  //积分限幅

    float                out_p;                 //KP输出
    float                out_i;                 //KI输出
    float                out_d;                 //KD输出
    float                out;                   //pid输出
    float                outmax;                //输出限幅

    float                integrator;            //< 积分值
    float                last_error;            //< 上次误差
    float                last_derivative;       //< 上次误差与上上次误差之差
    unsigned long        last_t;                //< 上次时间
}pid_param_t;

extern pid_param_t speed_pid_l;  // 电机PID
extern pid_param_t speed_pid_r;  // 电机PID
extern float speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX;
extern float speed_target;                      // 目标速度
extern float speed_real;                        // 实际速度
extern float speed_pwm;                         // 施加在电机上的PWM占空比


void My_Pid_Init(void);
void Pid_Param_Init(pid_param_t * pid, float kp, float ki, float kd, float imax, float outmax);

float constrain_float(float amt, float low, float high);
short constrain_short(short amt, short low, short high);


float PidLocCtrl(pid_param_t * pid, float error, float t);
float PidIncCtrl(pid_param_t * pid, float error , float t);


#endif
