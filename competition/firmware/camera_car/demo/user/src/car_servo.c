/* BDS300 PWM adapter. GPL-3.0-or-later. */
#include "zf_common_headfile.h"
#include "car_config.h"
#include "car_servo.h"
#include <math.h>
#if CAR_SERVO_HZ <= 0 || CAR_SERVO_HZ > 300
#error "Servo PWM frequency must be between 1 and 300 Hz."
#endif
static volatile float commanded_us=CAR_SERVO_CENTER_US;
static uint32_t pulse_to_duty(float us) {
    return (uint32_t)(us*(float)CAR_SERVO_HZ*(float)PWM_DUTY_MAX/1000000.0f);
}
void car_servo_init(void) {
    /* Request neutral duty in the init call; timer setup sequencing is SDK-owned. */
    pwm_init(CAR_SERVO_PWM,CAR_SERVO_HZ,pulse_to_duty(CAR_SERVO_CENTER_US));
    commanded_us=CAR_SERVO_CENTER_US;
}
void car_servo_set_us(float pulse_us) {
    if(!isfinite(pulse_us)) pulse_us=CAR_SERVO_CENTER_US;
    if(pulse_us<CAR_SERVO_MIN_US) pulse_us=CAR_SERVO_MIN_US;
    if(pulse_us>CAR_SERVO_MAX_US) pulse_us=CAR_SERVO_MAX_US;
    pwm_set_duty(CAR_SERVO_PWM,pulse_to_duty(pulse_us));
    commanded_us=pulse_us;
}
void car_servo_center(void) { car_servo_set_us(CAR_SERVO_CENTER_US); }
float car_servo_get_us(void) { return commanded_us; }
