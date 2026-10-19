#define main legacy_adapter_main
#include "test_app.c"
#undef main
static void steps(int n){for(int i=0;i<n;i++)frame();}
static void flush(void){for(int i=0;i<150;i++)car_app_foreground();tx[tx_len]=0;}
static void clear_tx(void){flush();tx_len=0;}
static void road(int good){for(int y=0;y<120;y++)for(int x=0;x<188;x++)camera[y][x]=(uint8_t)(good&&x>=50&&x<=138?210:30);}
static void press(void){gpio_levels[C15]=0;steps(5);}
static void release(void){gpio_levels[C15]=1;steps(5);}
static void click(void){press();release();}
static void stopped(void){CHECK(!duties[CAR_LEFT_PWM]&&!duties[CAR_RIGHT_PWM]);}
static void launch(void){feedback_left=feedback_right=0;road(1);steps(10);click();steps(202);feedback_left=feedback_right=20;steps(25);CHECK(duties[CAR_LEFT_PWM]&&duties[CAR_RIGHT_PWM]);}
int main(void){
 road(1);car_app_init();flush();
 /* Boot-held key cannot trigger a test, nor can holding indefinitely. */
 gpio_levels[C15]=0;steps(250);stopped();release();steps(230);stopped();
 /* Short bounce cannot produce a request. */
 gpio_levels[C15]=0;steps(1);gpio_levels[C15]=1;steps(5);steps(220);stopped();
 /* Countdown gives no power, and RX s cancels it permanently. */
 click();steps(100);stopped();car_app_uart_rx_isr('s');steps(130);stopped();
 /* A second press cancels countdown; its release must not re-arm. */
 click();steps(30);click();steps(230);stopped();
 /* Invalid vision at countdown expiry refuses without automatic retry. */
 clear_tx();click();road(0);steps(230);stopped();flush();CHECK(strstr((char*)tx,"BUTTON REFUSED"));road(1);steps(100);stopped();
 /* Encoder movement at expiry also refuses. */
 clear_tx();click();feedback_left=1;steps(230);stopped();flush();CHECK(strstr((char*)tx,"BUTTON REFUSED"));feedback_left=0;
 /* Local start finishes the full bounded trial without a single p packet. */
 clear_tx();launch();steps(1005);stopped();car_app_uart_rx_isr('p');flush();if(!strstr((char*)tx,"trial_done=1 trial_ms=10200"))fprintf(stderr,"TAIL %s\n",(char*)tx+(tx_len>1800?tx_len-1800:0));CHECK(strstr((char*)tx,"trial_done=1 trial_ms=10200"));CHECK(strstr((char*)tx,"BUTTON START"));
 /* Held run-stop button stops once, and cannot restart on release. */
 clear_tx();launch();press();stopped();steps(230);stopped();feedback_left=feedback_right=0;release();steps(230);stopped();
 /* ISR remote stop remains immediate for local mode. */
 launch();car_app_uart_rx_isr('s');stopped();feedback_left=feedback_right=0;steps(20);
 /* Local mode retains lost-line and foreground watchdog protection. */
 launch();road(0);steps(5);stopped();car_app_uart_rx_isr('s');feedback_left=feedback_right=0;road(1);steps(20);
 launch();for(int i=0;i<11;i++)car_app_tick_10ms();stopped();car_app_uart_rx_isr('s');feedback_left=feedback_right=0;steps(20);
 /* Wireless a still requires its original 500ms lease. */
 clear_tx();car_app_uart_rx_isr('a');car_app_foreground();feedback_left=feedback_right=20;steps(60);stopped();car_app_uart_rx_isr('p');flush();CHECK(strstr((char*)tx,"mode=3 fault=6"));
 puts("GT5 C15 actual adapter: debounce/boot-held/countdown/cancel/refuse/local 10s/no-radio/stop/line/foreground/remote lease PASS");return 0;
}
