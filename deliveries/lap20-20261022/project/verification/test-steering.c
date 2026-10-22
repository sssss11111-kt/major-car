#include <stdio.h>
#include <math.h>
#include "steering_control.h"
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
static float absf(float x){return x<0?-x:x;}
int main(void){
 CHECK(absf(STEERING_WHEELBASE_M/tanf(STEERING_MAX_WHEEL_ANGLE_RAD)-0.2f)<0.0001f);
 CHECK(STEERING_MODEL_CALIBRATED==0);
 CHECK(absf(STEERING_TIGHT_CURVE_SPEED_SCALE*45.0f-24.0f)<0.001f);
 steering_result_t r=steering_control(-4,-3,1,92.5f,-1,61,124);
 CHECK(!r.curve && absf(r.command-94.5f)<0.001f && r.speed_scale==1.0f);
 r=steering_control(0,0,1,92.5f,-1,61,124);CHECK(r.command==92.5f);
 r=steering_control(-8,-12,1,92.5f,-1,61,124);
 CHECK(r.curve && r.command>102.1f && r.command<=124);
 CHECK(r.speed_scale<1.0f && r.speed_scale>=24.0f/45.0f);
 r=steering_control(-6,30,1,92.5f,-1,61,124);CHECK(r.error<0 && r.command>92.5f);
 r=steering_control(-8,0,0,92.5f,-1,61,124);CHECK(r.curve && r.command>100.5f);
 r=steering_control(-4,-90,0,92.5f,-1,61,124);CHECK(!r.curve && r.command==94.5f);
 float previous=92.5f;
 for(int e=0;e<=94;e++){
  r=steering_control(-e,-e,1,92.5f,-1,61,124);
  steering_result_t mirror=steering_control(e,e,1,92.5f,-1,61,124);
  CHECK(r.command>=previous && r.command<=124 && mirror.command>=61);
  CHECK(absf(r.command+mirror.command-185.0f)<0.001f);
  CHECK(absf(r.speed_scale-mirror.speed_scale)<0.001f);
  CHECK(absf(r.curvature_reference)<=5.0001f);
  CHECK(r.speed_scale>=24.0f/45.0f && r.speed_scale<=1.0f);
  previous=r.command;
 }
 CHECK(r.command==124 && r.limited && absf(r.speed_scale-24.0f/45.0f)<0.001f);
 puts("PASS: geometry reference, unchanged straight, earlier turn, monotonic symmetric commands, limits, speed schedule, missing preview, sign guard");return 0;
}
