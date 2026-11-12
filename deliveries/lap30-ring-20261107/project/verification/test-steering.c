#include <stdio.h>
#include <math.h>
#include "steering_control.h"
#include "main.h"
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
static float ab(float x){return x<0?-x:x;}
static steering_result_t step(steering_state_t *s,int n,int p,int pv,uint32_t t)
{return steering_update(s,n,p,(uint8_t)pv,1,1,t,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);}
int main(void){
 CHECK(STEERING_MODEL_CALIBRATED==0);
 CHECK(SERVO_MOTOR_L_MAX==119 && SERVO_MOTOR_R_MAX==66 && SERVO_MOTOR_MIDDLE==92.5f);
 CHECK(ab(STEERING_TIGHT_CURVE_SPEED_SCALE*100.0f-80)<0.001f);
 steering_result_t r=steering_control(-4,-3,1,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
 CHECK(!r.curve && ab(r.command-94.5f)<0.001f && r.speed_scale==1);
 float previous=SERVO_MOTOR_MIDDLE;
 for(int e=0;e<=94;e++){
  r=steering_control(-e,-e,1,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
  steering_result_t mirror=steering_control(e,e,1,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
  CHECK(r.command>=previous && r.command<=SERVO_MOTOR_L_MAX && mirror.command>=SERVO_MOTOR_R_MAX);
  CHECK(ab(r.command+mirror.command-185)<0.001f);
  CHECK(ab(r.speed_scale-mirror.speed_scale)<0.001f);
  CHECK(r.speed_scale>=80.0f/100.0f-0.0001f && r.speed_scale<=1);
  previous=r.command;
 }
 CHECK(r.command==SERVO_MOTOR_L_MAX-STEERING_SOFT_LIMIT_MARGIN && ab(r.speed_scale-80.0f/100.0f)<0.001f);
 steering_result_t mild=steering_control(-8,-12,1,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
 steering_result_t sharp=steering_control(-30,-40,1,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
 CHECK(mild.command>SERVO_MOTOR_MIDDLE && mild.command<sharp.command && mild.gain==sharp.gain && sharp.gain==STEERING_LINEAR_GAIN);
 float a=steering_control(5,5,1,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX).command;
 float b=steering_control(6,6,1,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX).command;
 CHECK(ab(a-b)<0.6f);
 r=steering_control(-6,80,1,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);CHECK(r.command>=SERVO_MOTOR_MIDDLE);
 steering_state_t s,mirror;steering_reset(&s,SERVO_MOTOR_MIDDLE,0);steering_reset(&mirror,SERVO_MOTOR_MIDDLE,0);
 for(uint32_t t=2;t<=40;t+=2){r=step(&s,0,0,1,t);CHECK(r.command==SERVO_MOTOR_MIDDLE);}
 r=step(&s,94,94,1,42);CHECK(r.command==SERVO_MOTOR_MIDDLE); /* Isolated frame rejected. */
 r=step(&s,0,0,1,44);CHECK(r.command==SERVO_MOTOR_MIDDLE);
 float before=r.command;
 for(uint32_t t=46;t<=90;t+=2){
  r=step(&s,-94,-94,1,t);
  CHECK(ab(r.command-before)<=STEERING_SLEW_OUTWARD*0.02f+0.001f);
  CHECK(r.command>=before && r.command<=SERVO_MOTOR_L_MAX);before=r.command;
 }
 CHECK(ab(r.command-(SERVO_MOTOR_L_MAX-STEERING_SOFT_LIMIT_MARGIN))<0.001f);
 CHECK(ab(r.speed_scale*100.0f-80)<0.001f);
 before=r.command;
 for(uint32_t t=92;t<=152;t+=2){
  r=step(&s,94,94,1,t);
  CHECK(ab(r.command-before)<=STEERING_SLEW_RETURN*0.02f+0.001f && r.command>=SERVO_MOTOR_R_MAX && r.command<=SERVO_MOTOR_L_MAX);before=r.command;
 }
 CHECK(ab(r.command-(SERVO_MOTOR_R_MAX+STEERING_SOFT_LIMIT_MARGIN))<0.001f); /* Persistent opposite bend reaches the usable soft endpoint. */
 float last_scale=r.speed_scale;
 for(uint32_t t=154;t<=214;t+=2){
  r=step(&s,0,0,1,t);
  CHECK(r.speed_scale-last_scale<=STEERING_SPEED_RECOVERY_PER_S*0.02f+0.001f);
  last_scale=r.speed_scale;
 }
 CHECK(ab(r.command-SERVO_MOTOR_MIDDLE)<0.2f);
 steering_reset(&s,SERVO_MOTOR_MIDDLE,0);steering_reset(&mirror,SERVO_MOTOR_MIDDLE,0);
 for(uint32_t t=2;t<202;t+=2){
  int n=t<60?8:(t<100?35:(t<150?-30:0));
  r=step(&s,n,n+4,1,t);steering_result_t m=step(&mirror,-n,-n-4,1,t);
  CHECK(ab(r.command+m.command-185)<0.002f && ab(r.speed_scale-m.speed_scale)<0.001f);
 }
 r=steering_update(&s,40,40,1,0,1,204,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
 CHECK(r.command==SERVO_MOTOR_MIDDLE && !s.initialized && !s.preview_count);
 r=step(&s,0,-94,0,206);CHECK(ab(r.command-SERVO_MOTOR_MIDDLE)<0.001f);
 steering_result_t same=step(&s,94,94,1,206);CHECK(same.command==r.command);
 steering_reset(&s,SERVO_MOTOR_MIDDLE,0xfffffffeU);
 r=step(&s,-94,-94,1,0);CHECK(r.command<=94.701f);
 /* Increasing straight speed must not increase the full-bend target. */
 const float targets[]={100.0f,120.0f,140.0f};
 for(unsigned i=0;i<sizeof targets/sizeof targets[0];i++){
  CHECK(ab(steering_speed_target(targets[i],STEERING_TIGHT_CURVE_SPEED_SCALE)-80.0f)<0.001f);
  CHECK(steering_speed_target(targets[i],1.0f)==targets[i]);
  float previous_target=targets[i];
  for(int j=0;j<=200;j++){
   float scale=1.0f-j*0.001f;
   float target=steering_speed_target(targets[i],scale);
   CHECK(target<=previous_target+0.001f && target<=targets[i]*scale+0.001f);
   CHECK(previous_target-target<1.0f); /* No step at the straight/bend boundary. */
   previous_target=target;
  }
 }
 CHECK(ab(steering_speed_target(140.0f,0.9f)-110.0f)<0.001f);
 CHECK(ab(steering_speed_target(50.0f,0.8f)-40.0f)<0.001f);
 CHECK(steering_speed_target(50.0f,1.0f)==50.0f);
 /* Preserve the existing slower ring target and preview-only braking. */
 CHECK(ab(steering_speed_target(80.0f,0.8f)-64.0f)<0.001f);
 r=steering_control(0,40,1,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
 CHECK(r.command==SERVO_MOTOR_MIDDLE && ab(steering_speed_target(140,r.speed_scale)-80)<0.001f);
 /* Irregular but fresh camera cadence must use real elapsed time. */
 const uint32_t gaps[]={5,8,10,20};
 for(unsigned g=0;g<sizeof gaps/sizeof gaps[0];g++){
  uint32_t gap=gaps[g];
  steering_reset(&s,SERVO_MOTOR_MIDDLE,0);steering_reset(&mirror,SERVO_MOTOR_MIDDLE,0);
  step(&s,0,0,1,2);step(&s,0,0,1,4);
  step(&mirror,0,0,1,2);step(&mirror,0,0,1,4);
  r=step(&s,94,94,1,4+gap);CHECK(r.command==SERVO_MOTOR_MIDDLE);
  step(&mirror,-94,-94,1,4+gap);
  r=step(&s,94,94,1,4+2*gap);
  steering_result_t m=step(&mirror,-94,-94,1,4+2*gap);
  CHECK(ab(r.command-(SERVO_MOTOR_MIDDLE-STEERING_SLEW_OUTWARD*gap*0.01f))<0.001f);
  CHECK(ab(r.command+m.command-2*SERVO_MOTOR_MIDDLE)<0.001f);
 }
 /* A gap beyond the image deadline cannot become a queued large movement. */
 r=step(&s,94,94,1,s.last_tick+21);
 CHECK(r.command==SERVO_MOTOR_MIDDLE && !s.initialized);
 r=step(&s,94,94,1,s.last_tick+1);
 CHECK(ab(r.command-(SERVO_MOTOR_MIDDLE-STEERING_SLEW_OUTWARD*0.01f))<0.001f);
 r=steering_update(&s,94,94,1,1,0,s.last_tick+1,SERVO_MOTOR_MIDDLE,-1,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
 CHECK(r.command==SERVO_MOTOR_MIDDLE && !s.initialized);
 r=step(&s,94,94,1,s.last_tick+1000);
 CHECK(ab(r.command-(SERVO_MOTOR_MIDDLE-STEERING_SLEW_OUTWARD*0.01f))<0.001f);
 /* The unsigned elapsed calculation must work across timer rollover. */
 steering_reset(&s,SERVO_MOTOR_MIDDLE,0xffffffe0U);
 step(&s,0,0,1,0xffffffe2U);step(&s,0,0,1,0xffffffe4U);
 step(&s,94,94,1,0xfffffff8U);
 r=step(&s,94,94,1,0U);
 CHECK(ab(r.command-(SERVO_MOTOR_MIDDLE-STEERING_SLEW_OUTWARD*0.08f))<0.001f);
 puts("PASS: continuous bend response, symmetric bounds, LAP29 targets, isolated spikes, slew bound, sharp bend/reversal authority, speed recovery, preview loss, invalid reset, duplicate tick, tick wrap, irregular cadence, stale-history reset, independent bend speed");
 return 0;
}
