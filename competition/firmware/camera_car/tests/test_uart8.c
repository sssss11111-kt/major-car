/* Exercise actual UART8 dispatch, independent TX and diagnostic safety. */
#include "zf_common_headfile.h"
#include "car_app.h"
#include "car_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"UART8 FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static uint8_t camera[120][188],tx[2][150000],input[256];
uint8_t (*mt9v03x_image)[188]=camera;
volatile uint8_t mt9v03x_finish_flag;
static unsigned length[2],read_pos,read_end,errors,initializations,enabled;
static int ready[2]={1,1},rts,priority1,priority8,camera_inits;
static uint32_t duties[64];
uint32_t interrupt_global_disable(void) {return 0;}
void interrupt_global_enable(uint32_t key) {(void)key;}
void interrupt_set_priority(int irq,int p) {
    if(irq==LPUART1_IRQn) priority1=p;
    if(irq==LPUART8_IRQn) priority8=p;
}
uint32_t LPUART_GetStatusFlags(void *u) {
    unsigned n=u==LPUART8?1:0;
    return (ready[n]?kLPUART_TxDataRegEmptyFlag:0)|
        (n && read_pos<read_end?kLPUART_RxDataRegFullFlag:0)|(n?errors:0);
}
void LPUART_WriteByte(void *u,uint8_t b) {
    unsigned n=u==LPUART8?1:0;
    CHECK(length[n]+1<sizeof tx[n]);tx[n][length[n]++]=b;tx[n][length[n]]=0;
#if CAR_WIRELESS_AUTO_BAUD
    if(n && length[n]==3) {
        CHECK(tx[n][0]==0 && tx[n][1]==1 && tx[n][2]==3);
        input[0]=0xa5;
#ifdef TEST_BAD_BAUD_ACK
        input[1]=0;input[2]=0;
#else
        input[1]=0xff;input[2]=0xff;
#endif
        read_pos=0;read_end=3;
        car_app_uart8_irq();
    }
#endif
}
uint8_t LPUART_ReadByte(void *u) {CHECK(u==LPUART8 && read_pos<read_end);return input[read_pos++];}
int LPUART_ClearStatusFlags(void *u,uint32_t f) {CHECK(u==LPUART8);errors&=~f;return 0;}
void uart_init(int ch,uint32_t baud,int txp,int rxp) {
    CHECK(ch==UART_8 && baud==115200 && txp==UART8_TX_D16 && rxp==UART8_RX_D17);initializations++;
}
void uart_rx_interrupt(int ch,int on) {CHECK(ch==UART_8 && on && priority1==2 && priority8==2);enabled++;}
static uint8_t c11_level;
uint8_t gpio_get_level(int p) {
    if(p==C11) return c11_level;
    if(p==CAR_LEFT_DIR) return CAR_LEFT_FORWARD;
    if(p==CAR_RIGHT_DIR) return CAR_RIGHT_FORWARD;
    CHECK(p==D26);return (uint8_t)rts;
}
void gpio_set_level(int p,uint8_t level) {
    if(p==C11) {c11_level=!!level;return;}
    CHECK(p==D26);rts=level;
}
void system_delay_ms(uint32_t ms) {(void)ms;}
void pwm_set_duty(int pin,uint32_t duty) {duties[pin]=duty;}
void pwm_init(int pin,int hz,int duty) {(void)hz;duties[pin]=(uint32_t)duty;}
void gpio_init(int p,int d,int l,int m) {(void)p;(void)d;(void)l;(void)m;}
int16_t encoder_get_count(int ch) {(void)ch;return 0;}
void encoder_clear_count(int ch) {(void)ch;}
void encoder_quad_init(int ch,int a,int b) {(void)ch;(void)a;(void)b;}
void encoder_dir_init(int ch,int a,int b) {(void)ch;(void)a;(void)b;}
void pit_ms_init(int ch,int ms) {(void)ch;(void)ms;}
uint8_t mt9v03x_init(void) {camera_inits++;return 0;}
uint8_t mt9v03x_set_exposure_time(uint16_t e) {(void)e;return 0;}
void tft180_set_dir(int d) {(void)d;}
void tft180_init(void) {}
void tft180_show_string(int x,int y,const char *s) {(void)x;(void)y;(void)s;}
void tft180_show_gray_image(int x,int y,const uint8_t *p,int w,int h,int dw,int dh,int t) {
    (void)x;(void)y;(void)p;(void)w;(void)h;(void)dw;(void)dh;(void)t;
}
void tft180_draw_point(int x,int y,int c) {(void)x;(void)y;(void)c;}
static void flush(void) {for(int i=0;i<300;i++) car_app_foreground();}
static void receive(const char *s) {
    CHECK(strlen(s)<sizeof input);memcpy(input,s,strlen(s));read_pos=0;read_end=(unsigned)strlen(s);
    do {car_app_uart8_irq();} while(read_pos<read_end);
}
static void frame(void) {car_app_tick_10ms();mt9v03x_finish_flag=1;car_app_foreground();}
int main(void) {
    receive("1s"); /* Before outputs are ready, no PWM activity. */
    CHECK(duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    for(int y=0;y<120;y++) for(int x=0;x<188;x++) camera[y][x]=(uint8_t)(x>=50 && x<=138?210:30);
    car_app_init();CHECK(initializations==1 && enabled==1);flush();
    unsigned offset=CAR_WIRELESS_AUTO_BAUD?3:0;
    CHECK(strstr((char*)tx[1]+offset,"UART8-DIAG v=20261017B")!=NULL);
#if CAR_WIRELESS_AUTO_BAUD
#ifdef TEST_BAD_BAUD_ACK
    CHECK(strstr((char*)tx[1]+offset,"ab=2")!=NULL);
#else
    CHECK(strstr((char*)tx[1]+offset,"ab=1")!=NULL);
#endif
#endif
    ready[0]=0;unsigned old1=length[0],old8=length[1];
    receive("q");flush();
    CHECK(length[0]==old1 && length[1]>old8);
    CHECK(strstr((char*)tx[1]+old8,"pattern=UUUU0123456789")!=NULL);
    /* A blocked UART1 queue must not starve future wireless telemetry. */
    for(int k=0;k<400;k++){car_app_tick_10ms();flush();}
    CHECK(strstr((char*)tx[1]+offset,"ms=4000")!=NULL);
    ready[0]=1;flush();
    rts=1;old1=length[0];old8=length[1];receive("q");flush();
    CHECK(length[0]>old1 && length[1]==old8);
    rts=0;flush();CHECK(length[1]>old8);
    errors=kLPUART_FramingErrorFlag;receive("1");flush();car_app_tick_10ms();
    CHECK(errors==0 && duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    /* A valid command through the UART8 IRQ reaches real application dispatch. */
    receive("s");for(int i=0;i<4;i++) frame();receive("a");flush();car_app_tick_10ms();
#if CAR_COMM_DIAGNOSTIC
    CHECK(camera_inits==0 && duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    receive("u");flush();car_app_tick_10ms();
    CHECK(c11_level==1 && duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    receive("v");flush();car_app_tick_10ms();
    CHECK(c11_level==0 && duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);
    receive("12aljcgbreE");flush();car_app_tick_10ms();
    CHECK(duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0 && duties[CAR_SERVO_PWM]==750);
#else
    CHECK(camera_inits==1 && duties[CAR_LEFT_PWM]>0 && duties[CAR_RIGHT_PWM]>0);
#endif
    receive("s"); /* Immediate stop from UART8 without foreground. */
    CHECK(duties[CAR_LEFT_PWM]==0 && duties[CAR_RIGHT_PWM]==0);flush();
    receive("q");flush();CHECK(strstr((char*)tx[1]+offset,"err=1")!=NULL);
    for(unsigned i=offset;i<length[1];i++) CHECK(tx[1][i]!=0);
    printf("UART8 IRQ/RX/error-discard, independent TX/backpressure, RTS and diagnostic safety: PASS (diag=%u)\n",(unsigned)CAR_COMM_DIAGNOSTIC);
    return 0;
}
