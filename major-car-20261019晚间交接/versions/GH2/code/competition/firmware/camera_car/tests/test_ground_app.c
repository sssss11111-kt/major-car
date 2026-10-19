#define main legacy_adapter_main
#include "test_app.c"
#undef main
int main(int argc,char **argv){
 (void)argc;(void)argv;car_app_init();
 for(int i=0;i<6;i++)frame();
 car_app_uart_rx_isr('W');for(int i=0;i<100;i++)car_app_foreground();tx[tx_len]=0;
 CHECK(strstr((char*)tx,"GROUND-CFG v=20261019GH2 selected=2000"));
 car_app_uart_rx_isr('a');car_app_foreground();car_app_tick_10ms();CHECK(duties[CAR_LEFT_PWM]==0&&duties[CAR_RIGHT_PWM]==0);
 feedback_left=feedback_right=0;car_app_uart_rx_isr('Y');car_app_foreground();
 for(int i=0;i<19;i++)frame();CHECK(duties[CAR_LEFT_PWM]==0&&duties[CAR_RIGHT_PWM]==0);
 feedback_left=feedback_right=20;
 for(int i=0;i<172;i++){if(i%10==0)car_app_uart_rx_isr('W');frame();CHECK(duties[CAR_SERVO_PWM]==750);}
 CHECK(duties[CAR_LEFT_PWM]==0&&duties[CAR_RIGHT_PWM]==0);
 car_app_uart_rx_isr('W');for(int i=0;i<100;i++)car_app_foreground();tx[tx_len]=0;
 CHECK(strstr((char*)tx,"phase=3 done=1 elapsed=1900"));
 CHECK(strstr((char*)tx,"n=50,50 sum=1000,1000 min=20,20 max=20,20"));
 if(argc>1){FILE *f=fopen(argv[1],"wb");CHECK(f);fwrite(tx,1,tx_len,f);fclose(f);}
 feedback_left=feedback_right=0;car_app_uart_rx_isr('s');car_app_foreground();
 car_app_uart_rx_isr('Y');car_app_foreground();for(int i=0;i<25;i++){if(i%10==0)car_app_uart_rx_isr('W');frame();}
 car_app_uart_rx_isr('s');CHECK(duties[CAR_LEFT_PWM]==0&&duties[CAR_RIGHT_PWM]==0);car_app_foreground();
 puts("Ground adapter: GH2 identity / a refused / centered / normal expiry / ISR stop PASS");return 0;
}
