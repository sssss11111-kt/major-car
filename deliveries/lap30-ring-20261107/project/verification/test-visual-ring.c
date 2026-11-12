/* Actual vision owner; synthetic images are topology tests, not vehicle tests. */
#include "img_process.h"
#include "roundabout_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint8_t raw[120*188],left[120],mid[120],right[120];
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
static void scene(int single,int side) {
 memset(raw,20,sizeof raw);
 for(int y=18;y<=105;++y) {
  int shift=side*(105-y)/3;
  int l=54+shift,r=133+shift;
  if(single){l=side<0?0:45;r=side<0?142:187;}
  if(l<0)l=0;if(r>187)r=187;
  for(int x=l;x<=r;++x)raw[y*188+x]=220;
 }
}
static void frame(void){auxiliary_process(raw,120,188,127,left,mid,right);}
int main(int argc,char **argv) {
 CHECK(argc==2);
 if(!strcmp(argv[1],"single")) {
  for(int side=-1;side<=1;side+=2) {
   img_process_reset();scene(1,side);frame();
   CHECK(img_get_valid() && img_get_near_valid());
   CHECK(img_get_single_rows()>=20);
  }
 }else if(!strcmp(argv[1],"orbit")) {
  for(int side=-1;side<=1;side+=2) {
   img_process_reset();scene(0,side);for(int i=0;i<6;++i)frame();
   int ordinary=img_get_error(),preview=img_get_preview_error();
   CHECK(img_get_valid());
   img_process_reset();img_set_roundabout_guidance(-side);img_set_roundabout_entry(RB_ORBIT);
   for(int i=0;i<6;++i)frame();
   CHECK(img_get_valid() && img_get_near_valid());
   const entry_path_t *path=img_get_entry_path();
   CHECK(path->count>=12 && img_get_preview_valid());
   for(int i=0;i<path->count;++i)CHECK(raw[path->points[i].y*188+path->points[i].x]>100);
   printf("side=%d normal=%d/%d orbit=%d/%d\n",side,ordinary,preview,img_get_error(),img_get_preview_error());
   CHECK(img_get_error()*side>0 && img_get_preview_error()*side>0);
   CHECK(abs(img_get_error())<=12 && abs(img_get_preview_error())<=24);
   memset(raw,220,sizeof raw);frame();CHECK(!img_get_valid());
  }
 }else if(!strcmp(argv[1],"preview")) {
  img_process_reset();scene(0,0);
  for(int y=18;y<=76;++y) {
   memset(raw+y*188,20,188);
   for(int x=94;x<174;++x)raw[y*188+x]=220;
  }
  for(int i=0;i<4;++i)frame();
  CHECK(img_get_valid() && img_get_near_valid());
  CHECK(!img_get_preview_valid()); /* Far unrelated run must not bend the near controller. */
 }else if(!strcmp(argv[1],"tip")) {
  for(int side=-1;side<=1;side+=2) {
   img_process_reset();img_set_roundabout_guidance(-side);
   img_set_roundabout_entry(RB_ORBIT);img_set_exit_search(1);
   memset(raw,20,sizeof raw);
   for(int y=18;y<=105;++y) {
    int d=y>65?y-65:65-y;
    int bank=100-2*d;if(bank<20)bank=20;
    for(int x=bank+1;x<188;++x)raw[y*188+(side>0?x:187-x)]=220;
   }
   for(int i=0;i<4;++i)frame();
   CHECK(img_get_valid() && img_get_near_valid() && img_get_exit_ready());
   /* Monotonic bend has no two-arm corner. */
   for(int y=18;y<=105;++y) {
    memset(raw+y*188,20,188);
    for(int x=20+(105-y)/2;x<188;++x)raw[y*188+(side>0?x:187-x)]=220;
   }
   frame();CHECK(!img_get_exit_ready());
   memset(raw,220,sizeof raw);frame();CHECK(!img_get_exit_ready() && !img_get_valid());
  }
 }else if(!strcmp(argv[1],"angle")) {
  rb_state_t state={0};state.phase=RB_ENTER;
  for(int i=0;i<200;++i)rb_tick(&state,80,40,1);
  CHECK(state.phase==RB_ENTER); /* Motion alone is never a visual handoff. */
 }else return 2;
 printf("PASS visual-ring %s\n",argv[1]);return 0;
}
