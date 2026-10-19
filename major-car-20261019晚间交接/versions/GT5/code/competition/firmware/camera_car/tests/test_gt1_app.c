#define main legacy_adapter_main
#include "test_app.c"
#undef main
int main(int argc,char **argv){
 for(int y=0;y<120;y++)for(int x=0;x<188;x++)camera[y][x]=(uint8_t)(x>=50&&x<=138?210:30);
 car_app_init();for(int i=0;i<100;i++)car_app_foreground();
 car_app_uart_rx_isr('d');for(int i=0;i<100;i++)car_app_foreground();tx[tx_len]=0;
 CHECK(strstr((char*)tx,"TRACK-CFG v=20261019GT5 target=40 cap=3000 settle=200 duration=10000 lease=500 simulated=0 profile=GL2 kp10=40 ki10=200 ff0=896,896 ff10=155,151 closed_cap=3000 kick_min=200 kick_max=300 transfer=200 entry=ff guard=120"));
 for(int i=0;i<4;i++)frame();car_app_uart_rx_isr('a');car_app_foreground();feedback_left=feedback_right=20;
 for(int i=0;i<1025;i++){if(i%10==0)car_app_uart_rx_isr('p');frame();}
 CHECK(duties[CAR_LEFT_PWM]==0&&duties[CAR_RIGHT_PWM]==0&&duties[CAR_SERVO_PWM]==750);
 for(int attempt=0;attempt<3;attempt++){car_app_uart_rx_isr('p');for(int i=0;i<100;i++)car_app_foreground();}tx[tx_len]=0;
 if(argc>1){FILE*f=fopen(argv[1],"wb");CHECK(f);fwrite(tx,1,tx_len,f);fclose(f);}
 CHECK(strstr((char*)tx,"trial_done=1 trial_ms=10200 pi_mask=3"));
 if(argc>1){FILE*f=fopen(argv[1],"wb");CHECK(f);fwrite(tx,1,tx_len,f);fclose(f);}

 car_app_uart_rx_isr('s');car_app_foreground();feedback_left=feedback_right=0;for(int i=0;i<100;i++)car_app_foreground();unsigned reacquire_offset=tx_len;
 for(int y=0;y<120;y++)for(int x=0;x<188;x++)camera[y][x]=(uint8_t)(x>=10&&x<=178?210:30);
 for(int i=0;i<50;i++)frame();
 for(int j=0;j<3;j++){car_app_uart_rx_isr('p');for(int i=0;i<100;i++)car_app_foreground();}tx[tx_len]=0;
 CHECK(strstr((char*)tx+reacquire_offset,"valid=1 pair=13/13"));
 CHECK(strstr((char*)tx+reacquire_offset,"ready=1 good_frames="));
 CHECK(duties[CAR_LEFT_PWM]==0&&duties[CAR_RIGHT_PWM]==0);

 for(int i=0;i<5;i++)frame();car_app_uart_rx_isr('a');car_app_foreground();feedback_left=121;feedback_right=20;frame();
 car_app_uart_rx_isr('s');car_app_foreground();feedback_left=feedback_right=0;
 for(int i=0;i<100;i++)car_app_foreground();unsigned trace_offset=tx_len;
 car_app_uart_rx_isr('Z');for(int i=0;i<100;i++)car_app_foreground();tx[tx_len]=0;
 CHECK(strstr((char*)tx+trace_offset,"DIAG-STOP "));
 CHECK(strstr((char*)tx+trace_offset,"prior_mode=3 prior_fault=5"));
 CHECK(duties[CAR_LEFT_PWM]==0&&duties[CAR_RIGHT_PWM]==0);
 puts("GT4D actual adapter: runtime profile / GL2 takeover / bounded visual trial / zero PASS");return 0;
}
