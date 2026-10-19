/* Narrow host double for MCU I/O only; car_app/core remain real production C. */
#ifndef TEST_SDK_H
#define TEST_SDK_H
#include <stdint.h>
typedef uint8_t uint8;
typedef uint32_t uint32;
#define MT9V03X_W 188
#define MT9V03X_H 120
#define MT9V03X_EXP_TIME_DEF 512
#define PWM_DUTY_MAX 10000
#define GPIO_LOW 0
#define C11 60
#define C15 61
#define RGB565_GREEN 0x07e0
enum { QTIMER1_ENCODER1=1,QTIMER1_ENCODER2,
 QTIMER1_ENCODER1_CH1_C0,QTIMER1_ENCODER1_CH2_C1,
 QTIMER1_ENCODER2_CH1_C2,QTIMER1_ENCODER2_CH2_C24,
 PWM2_MODULE0_CHA_C6,PWM2_MODULE1_CHA_C8,PWM4_MODULE2_CHA_C30,
 C7,C9,GPIO_HIGH,GPO,GPO_PUSH_PULL,PIT_CH0,PIT_IRQn,LPUART1_IRQn,CSI_IRQn,
 TFT180_PORTAIT,RGB565_BLUE,RGB565_RED,
 UART_8,UART8_TX_D16,UART8_RX_D17,LPUART8_IRQn,D26,GPI,GPI_PULL_UP };
#define LPUART1 ((void *)1)
#define LPUART8 ((void *)8)
#define kLPUART_TxDataRegEmptyFlag 1
#define kLPUART_RxDataRegFullFlag 2
#define kLPUART_RxOverrunFlag 4
#define kLPUART_FramingErrorFlag 8
#define kLPUART_NoiseErrorFlag 16
#define kLPUART_ParityErrorFlag 32
extern volatile uint8_t mt9v03x_finish_flag;
extern uint8_t (*mt9v03x_image)[188];
uint32_t interrupt_global_disable(void);
void interrupt_global_enable(uint32_t key);
void interrupt_set_priority(int irq,int priority);
uint32_t LPUART_GetStatusFlags(void *uart);
void LPUART_WriteByte(void *uart,uint8_t b);
uint8_t LPUART_ReadByte(void *uart);
int LPUART_ClearStatusFlags(void *uart,uint32_t flags);
void uart_init(int channel,uint32_t baud,int tx_pin,int rx_pin);
void uart_rx_interrupt(int channel,int enable);
uint8_t gpio_get_level(int pin);
void gpio_set_level(int pin,uint8_t level);
void system_delay_ms(uint32_t ms);
void pwm_set_duty(int pin,uint32_t duty);
void pwm_init(int pin,int hz,int duty);
void gpio_init(int pin,int dir,int level,int mode);
int16_t encoder_get_count(int channel);
void encoder_clear_count(int channel);
void encoder_quad_init(int ch,int a,int b);
void encoder_dir_init(int ch,int a,int b);
void pit_ms_init(int channel,int ms);
uint8_t mt9v03x_init(void);
uint8_t mt9v03x_set_exposure_time(uint16_t value);
void tft180_set_dir(int direction);
void tft180_init(void);
void tft180_show_string(int x,int y,const char *text);
void tft180_show_gray_image(int x,int y,const uint8_t *data,int w,int h,int dw,int dh,int th);
void tft180_draw_point(int x,int y,int color);
#endif
