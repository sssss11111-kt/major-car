#include "zf_common_headfile.h"
#include "car_app.h"
#include "car_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL adapter line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static uint8_t camera[120][188],tx[80000];
uint8_t (*mt9v03x_image)[188]=camera;
volatile uint8_t mt9v03x_finish_flag;
static uint32_t duties[64];
static unsigned tx_len;
static unsigned stop_on_lock;
uint32_t interrupt_global_disable(void) {
    /* Inject RX just before entering a critical section, as on real hardware. */
    if(stop_on_lock && --stop_on_lock==0) car_app_uart_rx_isr('s');
    return 0;
}
void interrupt_global_enable(uint32_t k) {(void)k;}
void interrupt_set_priority(int i,int p) {(void)i;(void)p;}
uint32_t LPUART_GetStatusFlags(void *u) {(void)u;return 1;}
void LPUART_WriteByte(void *u,uint8_t b) {(void)u;if(tx_len<sizeof tx) tx[tx_len++]=b;}
void pwm_set_duty(int p,uint32_t d) {duties[p]=d;}
void pwm_init(int p,int h,int d) {(void)h;duties[p]=(uint32_t)d;}
void gpio_init(int p,int d,int l,int m) {(void)p;(void)d;(void)l;(void)m;}
int16_t encoder_get_count(int c) {(void)c;return 0;}
void encoder_clear_count(int c) {(void)c;}
void encoder_quad_init(int c,int a,int b) {(void)c;(void)a;(void)b;}
void encoder_dir_init(int c,int a,int b) {(void)c;(void)a;(void)b;}
void pit_ms_init(int c,int m) {(void)c;(void)m;}
uint8_t mt9v03x_init(void) {return 0;}
uint8_t mt9v03x_set_exposure_time(uint16_t v) {(void)v;return 0;}
void tft180_set_dir(int d) {(void)d;}
void tft180_init(void) {}
void tft180_show_string(int x,int y,const char *s) {(void)x;(void)y;(void)s;}
void tft180_show_gray_image(int x,int y,const uint8_t *d,int w,int h,int dw,int dh,int th) {
    (void)x;(void)y;(void)d;(void)w;(void)h;(void)dw;(void)dh;(void)th;
}
void tft180_draw_point(int x,int y,int c) {(void)x;(void)y;(void)c;}
static void frame(void) {
    car_app_tick_10ms(); mt9v03x_finish_flag=1; car_app_foreground();
}
int main(void) {
    for(int y=0;y<120;y++) for(int x=0;x<188;x++) camera[y][x]=(uint8_t)(x>=50 && x<=138?210:30);
    car_app_init(); CHECK(duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    car_app_uart_rx_isr('a'); car_app_foreground(); car_app_tick_10ms();
    CHECK(duties[CAR_LEFT_PWM]==0);
    for(int i=0;i<4;i++) frame();
    car_app_uart_rx_isr('a'); car_app_foreground(); car_app_tick_10ms();
    CHECK(duties[CAR_LEFT_PWM]>0 && duties[CAR_RIGHT_PWM]>0);
    car_app_uart_rx_isr('s'); /* No foreground call: ISR must already stop. */
    CHECK(duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    car_app_foreground();
    for(int i=0;i<4;i++) frame();
    car_app_uart_rx_isr('a'); car_app_foreground(); car_app_tick_10ms();
    CHECK(duties[CAR_LEFT_PWM]>0);
    for(int i=0;i<11;i++) car_app_tick_10ms();
    CHECK(duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    car_app_uart_rx_isr('s'); car_app_foreground();
    car_app_uart_rx_isr('l'); car_app_foreground(); car_app_tick_10ms();
    CHECK(duties[CAR_SERVO_PWM]==(uint32_t)(CAR_SERVO_MIN_US*50*10000/1000000));
    car_app_uart_rx_isr('s'); CHECK(duties[CAR_SERVO_PWM]==750);
    car_app_foreground(); frame();
    car_app_uart_rx_isr('g');
    for(int i=0;i<2000;i++) car_app_foreground();
    unsigned start=0;
    while(start+4<tx_len && memcmp(tx+start,"\x00\xff\x01\x01",4)!=0) start++;
    CHECK(start+4+sizeof camera<=tx_len);
    CHECK(memcmp(tx+start+4,camera,sizeof camera)==0);
    /* Pre-stop commands must not revive motion, even if RX ring was full. */
    for(int i=0;i<63;i++) car_app_uart_rx_isr('1');
    car_app_uart_rx_isr('s');
    car_app_foreground(); car_app_tick_10ms();
    CHECK(duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    for(int i=0;i<10;i++) { car_app_foreground(); car_app_tick_10ms(); }
    CHECK(duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    /* heartbeat lock, dequeue lock, then command lock: stop after dequeue. */
    car_app_uart_rx_isr('1'); stop_on_lock=3;
    car_app_foreground(); car_app_tick_10ms();
    CHECK(stop_on_lock==0 && duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    puts("hardware adapter tests: PASS (host I/O double, not physical hardware)");
    return 0;
}
