#include "zf_common_headfile.h"
#include "car_config.h"
#include "car_servo.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL servo line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static int init_pin,init_hz,last_pin;
static uint32_t init_duty,last_duty;
void pwm_init(int pin,int hz,int duty) { init_pin=pin;init_hz=hz;init_duty=(uint32_t)duty; }
void pwm_set_duty(int pin,uint32_t duty) { last_pin=pin;last_duty=duty; }
int main(void) {
    car_servo_init();
    CHECK(init_pin==CAR_SERVO_PWM && init_hz==50 && init_duty==750);
    CHECK(car_servo_get_us()==1500);
    car_servo_set_us(1600); CHECK(last_pin==CAR_SERVO_PWM && last_duty==800);
    CHECK(car_servo_get_us()==1600);
    car_servo_set_us(0); CHECK(last_duty==675 && car_servo_get_us()==1350);
    car_servo_set_us(3000); CHECK(last_duty==825 && car_servo_get_us()==1650);
    car_servo_center(); CHECK(last_duty==750 && car_servo_get_us()==1500);
    car_servo_set_us(NAN); CHECK(last_duty==750 && car_servo_get_us()==1500);
    car_servo_set_us(INFINITY); CHECK(last_duty==750);
    car_servo_set_us(-INFINITY); CHECK(last_duty==750);
    car_servo_init(); CHECK(init_duty==750 && car_servo_get_us()==1500);
    puts("servo initialization/limits/nonfinite/reinitialize tests: PASS");
    return 0;
}
