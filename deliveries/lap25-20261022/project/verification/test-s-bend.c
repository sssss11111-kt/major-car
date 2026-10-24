#include <stdio.h>
#include <string.h>
#include <math.h>
#include "steering_control.h"
#include "main.h"
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %s:%d %s command=%.4f gain=%.4f\n",argv[1],__LINE__,#x,r.command,r.gain);return 1;}}while(0)
static float ab(float v){return v<0?-v:v;}
static steering_result_t request(int n,int p,int pv){
 return steering_control(n,p,(uint8_t)pv,SERVO_MOTOR_MIDDLE,SERVO_DIR,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
}
static steering_result_t update(steering_state_t *s,int n,int p,uint32_t tick){
 return steering_update(s,n,p,1,1,1,tick,SERVO_MOTOR_MIDDLE,SERVO_DIR,SERVO_MOTOR_R_MAX,SERVO_MOTOR_L_MAX);
}
int main(int argc,char **argv){
 steering_result_t r={0};
 if(argc!=2)return 2;
 if(!strcmp(argv[1],"linear")){
  for(int n=-40;n<=40;n++){
   r=request(n,n,1);
   CHECK(ab(r.command-(92.5f-0.5f*n))<0.001f);
   CHECK(ab(r.gain-0.5f)<0.001f);
  }
 }else if(!strcmp(argv[1],"preview")){
  for(int p=-94;p<=94;p++){
   r=request(0,p,1);CHECK(ab(r.command-92.5f)<0.001f);
  }
  r=request(0,94,1);CHECK(r.speed_scale*100.0f<=80.001f);
  float previous=request(-4,-40,1).command;
  for(int n=-3;n<=4;n++){
   r=request(n,-40,1);
   CHECK(ab(r.command-previous)<=0.751f);
   if(n>0)CHECK(r.command<=92.5f);
   previous=r.command;
  }
  for(int n=1;n<=40;n++){
   r=request(n,94,1);
   CHECK(ab(r.command-(92.5f-0.5f*n))<=1.601f);
   steering_result_t mirror=request(-n,-94,1);
   CHECK(ab(r.command+mirror.command-185)<0.001f);
  }
 }else if(!strcmp(argv[1],"limits")){
  CHECK(SERVO_MOTOR_L_MAX==119 && SERVO_MOTOR_R_MAX==66);
  for(int n=-94;n<=94;n++)for(int p=-94;p<=94;p+=2){
   r=request(n,p,1);CHECK(r.command>=69 && r.command<=116);
  }
  r=request(94,94,1);CHECK(r.command==69);
  CHECK(ab(r.speed_scale*100.0f-80)<0.001f);
  r=request(-94,-94,1);CHECK(r.command==116);
 }else if(!strcmp(argv[1],"reverse")){
  for(int sign=-1;sign<=1;sign+=2)for(uint32_t dt=1;dt<=4;dt*=2){
   steering_state_t s;uint32_t tick=0;steering_reset(&s,92.5f,0);
   for(int i=0;i<100;i++){tick+=dt;r=update(&s,sign*30,sign*40,tick);}
   float old=r.command;int released=0,crossed=0,budget_checked=0;
   for(int i=1;i<=20/(int)dt;i++){
    float previous=r.command;tick+=dt;r=update(&s,-sign*30,-sign*40,tick);
    CHECK(ab(r.command-previous)<=180.0f*dt*0.01f+0.001f);
    float before_offset=previous-92.5f,after_offset=r.command-92.5f;
    if(before_offset*after_offset<0){
     float remaining=dt*0.01f-ab(before_offset)/180.0f;
     CHECK(remaining>=0 && ab(after_offset)<=110.0f*remaining+0.001f);
     budget_checked=1;
    }
    if((r.command-old)*sign>0.5f)released=1;
    if((r.command-92.5f)*sign>=0)crossed=1;
    CHECK(r.command>=69 && r.command<=116);
   }
   CHECK(released && crossed && budget_checked);
  }
 }else if(!strcmp(argv[1],"spike")){
  steering_state_t s;steering_reset(&s,92.5f,0);
  for(uint32_t t=2;t<=40;t+=2)r=update(&s,0,0,t);
  r=update(&s,94,94,42);CHECK(r.command==92.5f);
  r=update(&s,0,0,44);CHECK(r.command==92.5f);
 }else return 2;
 printf("PASS: S-bend %s\n",argv[1]);return 0;
}
