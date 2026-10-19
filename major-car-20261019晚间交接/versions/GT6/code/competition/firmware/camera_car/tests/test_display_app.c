#define main legacy_adapter_main
#define tft180_show_gray_image legacy_gray
#define tft180_draw_point legacy_point
#define tft180_show_string legacy_string
#include "test_app.c"
#undef main
#undef tft180_show_gray_image
#undef tft180_draw_point
#undef tft180_show_string
static unsigned images,blues,greens,reds,pair_labels;
void tft180_show_gray_image(int x,int y,const uint8_t*d,int w,int h,int dw,int dh,int th){
 CHECK(x==0&&y==0&&w==188&&h==120&&dw==128&&dh==82&&th==0);
 CHECK(d[60*188+90]==137);CHECK(d[60*188+10]==30);images++;
}
void tft180_draw_point(int x,int y,int c){CHECK(x>=0&&x<128&&y>=0&&y<82);if(c==RGB565_BLUE)blues++;if(c==RGB565_GREEN)greens++;if(c==RGB565_RED)reds++;}
void tft180_show_string(int x,int y,const char*s){CHECK(x>=0&&y>=0&&y+16<=160);CHECK(strlen(s)<=16);if(strstr(s,"PAIR"))pair_labels++;}
int main(void){
 for(int y=0;y<120;y++)for(int x=0;x<188;x++)camera[y][x]=(uint8_t)(x>=50&&x<=138?137:30);
 car_app_init();for(int i=0;i<30;i++)frame();
 CHECK(images>0);CHECK(blues>0&&greens>0&&reds>0);CHECK(pair_labels>0);
 CHECK(!duties[CAR_LEFT_PWM]&&!duties[CAR_RIGHT_PWM]);
 puts("GT6 display: original grayscale / distinct edge colors / pair counts / bounds / idle zero PASS");return 0;
}
