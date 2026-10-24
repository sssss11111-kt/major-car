#include <stdio.h>
#include <math.h>
#include "steering_control.h"
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
static float ab(float x){return x<0?-x:x;}
static steering_result_t step(steering_state_t *s,int n,int p,int pv,uint32_t t)
{return steering_update(s,n,p,(uint8_t)pv,1,1,t,92.5f,-1,61,124);}
int main(void){
 CHECK(STEERING_MODEL_CALIBRATED==0);
 CHECK(ab(STEERING_TIGHT_CURVE_SPEED_SCALE*67.5f-36)<0.001f);
 steering_result_t r=steering_control(-4,-3,1,92.5f,-1,61,124);
 CHECK(!r.curve && ab(r.command-94.5f)<0.001f && r.speed_scale==1);
 float previous=92.5f;
 for(int e=0;e<=94;e++){
  r=steering_control(-e,-e,1,92.5f,-1,61,124);
  steering_result_t mirror=steering_control(e,e,1,92.5f,-1,61,124);
  CHECK(r.command>=previous && r.command<=124 && mirror.command>=61);
  CHECK(ab(r.command+mirror.command-185)<0.001f);
  CHECK(ab(r.speed_scale-mirror.speed_scale)<0.001f);
  CHECK(r.speed_scale>=36.0f/67.5f-0.0001f && r.speed_scale<=1);
  previous=r.command;
 }
 CHECK(r.command==124 && ab(r.speed_scale-36.0f/67.5f)<0.001f);
 steering_result_t mild=steering_control(-8,-12,1,92.5f,-1,61,124);
 steering_result_t sharp=steering_control(-30,-40,1,92.5f,-1,61,124);
 CHECK(mild.command>92.5f && mild.command<sharp.command && mild.gain<sharp.gain);
 float a=steering_control(5,5,1,92.5f,-1,61,124).command;
 float b=steering_control(6,6,1,92.5f,-1,61,124).command;
 CHECK(ab(a-b)<0.6f);
 r=steering_control(-6,80,1,92.5f,-1,61,124);CHECK(r.command>=92.5f);
 steering_state_t s,mirror;steering_reset(&s,92.5f,0);steering_reset(&mirror,92.5f,0);
 for(uint32_t t=2;t<=40;t+=2){r=step(&s,0,0,1,t);CHECK(r.command==92.5f);}
 r=step(&s,94,94,1,42);CHECK(r.command==92.5f); /* Isolated frame rejected. */
 r=step(&s,0,0,1,44);CHECK(r.command==92.5f);
 float before=r.command;
 for(uint32_t t=46;t<=90;t+=2){
  r=step(&s,-94,-94,1,t);
  CHECK(ab(r.command-before)<=STEERING_SLEW_TIGHT*0.02f+0.001f);
  CHECK(r.command>=before && r.command<=124);before=r.command;
 }
 CHECK(ab(r.command-124)<0.001f);
 CHECK(ab(r.speed_scale*67.5f-36)<0.001f);
 before=r.command;
 for(uint32_t t=92;t<=152;t+=2){
  r=step(&s,94,94,1,t);
  CHECK(ab(r.command-before)<=4.401f && r.command>=61 && r.command<=124);before=r.command;
 }
 CHECK(ab(r.command-61)<0.001f); /* Persistent opposite bend still reaches full authority. */
 float last_scale=r.speed_scale;
 for(uint32_t t=154;t<=214;t+=2){
  r=step(&s,0,0,1,t);
  CHECK(r.speed_scale-last_scale<=STEERING_SPEED_RECOVERY_PER_S*0.02f+0.001f);
  last_scale=r.speed_scale;
 }
 CHECK(ab(r.command-92.5f)<0.2f);
 steering_reset(&s,92.5f,0);steering_reset(&mirror,92.5f,0);
 for(uint32_t t=2;t<202;t+=2){
  int n=t<60?8:(t<100?35:(t<150?-30:0));
  r=step(&s,n,n+4,1,t);steering_result_t m=step(&mirror,-n,-n-4,1,t);
  CHECK(ab(r.command+m.command-185)<0.002f && ab(r.speed_scale-m.speed_scale)<0.001f);
 }
 r=steering_update(&s,40,40,1,0,1,204,92.5f,-1,61,124);
 CHECK(r.command==92.5f && !s.initialized && !s.preview_count);
 r=step(&s,0,-94,0,206);CHECK(ab(r.command-92.5f)<0.001f);
 steering_result_t same=step(&s,94,94,1,206);CHECK(same.command==r.command);
 steering_reset(&s,92.5f,0xfffffffeU);
 r=step(&s,-94,-94,1,0);CHECK(r.command<=94.701f);
 puts("PASS: continuous bend response, symmetric bounds, 50% targets, isolated spikes, slew bound, sharp bend/reversal authority, speed recovery, preview loss, invalid reset, duplicate tick, tick wrap");
 return 0;
}
