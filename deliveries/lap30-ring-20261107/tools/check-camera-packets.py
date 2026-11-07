"""Host-check actual image transmitter: native wired pixels and protocol bytes."""
from pathlib import Path
import subprocess
b=Path(__file__).resolve().parents[1];v=b/'project/verification'
s=(b/'project/demo/user/src/main.c').read_text(encoding='utf-8-sig');fn=s[s.index('static void sf_camera_send(void)'):s.index('int main(void)')]
stub=r'''
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
typedef uint8_t uint8;typedef uint32_t uint32;
static uint8 sf_view_mode,trial_running,raw_image[120][188],wire[23000];
static uint32 length,wired,radio;
static uint32 debug_send_buffer(const uint8 *p,uint32 n){memcpy(wire+length,p,n);length+=n;wired++;return 0;}
static uint32 wireless_uart_send_buffer(const uint8 *p,uint32 n){memcpy(wire+length,p,n);length+=n;radio++;return 0;}
'''
test=r'''
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
'''
p=v/'test-camera-packets.c';p.write_text(stub+fn+test,encoding='utf-8');exe=v/'test-camera-packets.exe'
subprocess.run(['cl','/nologo','/TC','/std:c17','/W4','/WX','/utf-8',str(p),'/Fe'+str(exe)],cwd=v,check=True)
subprocess.run([str(exe)],check=True)
