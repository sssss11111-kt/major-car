
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
typedef uint8_t uint8;typedef uint32_t uint32;
static uint8 sf_view_mode,trial_running,raw_image[120][188],wire[23000];
static uint32 length,wired,radio;
static uint32 debug_send_buffer(const uint8 *p,uint32 n){memcpy(wire+length,p,n);length+=n;wired++;return 0;}
static uint32 wireless_uart_send_buffer(const uint8 *p,uint32 n){memcpy(wire+length,p,n);length+=n;radio++;return 0;}
static void sf_camera_send(void)
{
    static uint8 gray[94*60];
    uint8 mode=sf_view_mode, h[12]={0};
    uint8 width=mode>=3 ? 188:94, height=mode>=3 ? 120:60;
    const uint8 *pixels=mode>=3 ? (const uint8 *)raw_image:gray;
    uint32 payload=(uint32)width*height;
    if(!mode || trial_running)return;
    for(uint32 y=0;y<60;++y)for(uint32 x=0;x<94;++x)
        gray[y*94+x]=raw_image[y*2][x*2];
    h[0]=0xaa;
    if(mode==2 || mode==4) {
        /* V2: head, sum excluding sum byte, cmd=CAMERA, GRAY, LE size,
         * data_type=UINT8, three reserved zeros (official config size 12). */
        h[2]=2;h[3]=2;h[4]=width;h[6]=height;h[8]=1;
        for(uint8 i=0;i<12;++i)if(i!=1)h[1]=(uint8)(h[1]+h[i]);
    }else {
        /* V1: head, CAMERA function, MT9V03X<<5, header size, LE size. */
        h[1]=2;h[2]=0x40;h[3]=8;h[4]=width;h[6]=height;
    }
    uint32 n=(mode==2 || mode==4)?12:8;
    if(mode>=3) {
        debug_send_buffer(h,n);debug_send_buffer(pixels,payload);
    }else {
        if(wireless_uart_send_buffer(h,n))return;
        wireless_uart_send_buffer(pixels,payload);
    }
}


int main(void) {
 for(int y=0;y<120;y++)for(int x=0;x<188;x++)raw_image[y][x]=(uint8)((y*188+x)%251);
 for(uint8 mode=1;mode<=4;mode++) {
  sf_view_mode=mode;length=wired=radio=0;sf_camera_send();
  int v2=(mode==2 || mode==4),head=v2?12:8,w=mode>=3?188:94,h=mode>=3?120:60;
  assert(length==(uint32)(head+w*h));assert(wire[0]==0xaa && wire[4]==w && wire[6]==h);
  if(v2){uint8 sum=0;for(int i=0;i<12;i++)if(i!=1)sum=(uint8)(sum+wire[i]);assert(sum==wire[1] && wire[3]==2 && wire[8]==1);}
  else assert(wire[1]==2 && wire[2]==0x40 && wire[3]==8);
  for(int y=0;y<h;y++)for(int x=0;x<w;x++)assert(wire[head+y*w+x]==raw_image[mode>=3?y:y*2][mode>=3?x:x*2]);
  assert(mode>=3 ? (wired==2 && radio==0):(radio==2 && wired==0));
 }
 sf_view_mode=4;trial_running=1;length=0;sf_camera_send();assert(length==0);
 trial_running=0;sf_view_mode=0;sf_camera_send();assert(length==0);
 puts("PASS: actual V1/V2 headers, full 188x120 wired pixel identity, 94x60 radio sampling, correct transport, no image send while running/off");
 return 0;
}
