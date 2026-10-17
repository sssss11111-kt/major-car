#ifndef CAR_APP_H
#define CAR_APP_H
#include <stdint.h>
void car_app_init(void);
void car_app_foreground(void);
void car_app_tick_10ms(void);
void car_app_uart_rx_isr(uint8_t byte);
void car_app_uart8_irq(void);
#endif
