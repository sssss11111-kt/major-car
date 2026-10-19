#ifndef CAR_SERVO_H
#define CAR_SERVO_H
/* Call once before any timer/IRQ starts commanding the servo. Units: microseconds. */
void car_servo_init(void);
void car_servo_set_us(float pulse_us);
void car_servo_center(void);
float car_servo_get_us(void);
#endif
