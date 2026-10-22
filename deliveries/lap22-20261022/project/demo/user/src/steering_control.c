#include "steering_control.h"
#include <math.h>
static float clip(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
static float mag(float x){return x<0?-x:x;}
static float smooth(float x){x=clip(x,0,1);return x*x*(3-2*x);}
static float maxf(float a,float b){return a>b?a:b;}
static float median(float a,float b,float c)
{return a>b?(b>c?b:(a>c?c:a)):(a>c?a:(b>c?c:b));}

/* Pixel-domain bend demand, not a calibrated road-radius measurement. */
static steering_result_t desired(float near,float preview,uint8_t pv,
    float middle,float direction,float low,float high)
{
    steering_result_t out={0};
    float severity=smooth((mag(near)-6.0f)/24.0f);
    if(pv){
        severity=maxf(severity,smooth((mag(preview)-10.0f)/30.0f));
        severity=maxf(severity,smooth((mag(preview-near)-6.0f)/26.0f));
    }
    out.severity=severity;out.curve=(uint8_t)(severity>0.05f);
    out.gain=STEERING_STRAIGHT_GAIN+(STEERING_CURVE_GAIN-STEERING_STRAIGHT_GAIN)*severity;
    out.error=near;
    if(pv){
        /* Fade opposite preview authority; never reverse an established near target. */
        float authority=near*preview>=0?1.0f:clip((6.0f-mag(near))/4.0f,0,1);
        out.error+=STEERING_PREVIEW_BLEND*severity*authority*(preview-near);
        if(mag(near)>2.0f && out.error*near<0)out.error=0;
    }
    float offset=out.error*out.gain*direction;
    float span=offset>=0?high-middle:middle-low;
    float demand=span>0?offset/span:0;
    out.limited=(uint8_t)(mag(demand)>1);
    demand=clip(demand,-1,1);
    float shaped=atanf(demand*tanf(STEERING_MAX_WHEEL_ANGLE_RAD))/STEERING_MAX_WHEEL_ANGLE_RAD;
    float fraction=demand+severity*(shaped-demand);
    out.command=clip(middle+span*fraction,low,high);
    out.curvature_reference=fraction*tanf(STEERING_MAX_WHEEL_ANGLE_RAD)/STEERING_WHEELBASE_M;
    float load=maxf(clip((mag(fraction)-0.10f)/0.90f,0,1),0.75f*severity);
    out.speed_scale=1-(1-STEERING_TIGHT_CURVE_SPEED_SCALE)*load;
    out.bend_level=severity>0.7f?3:(severity>0.3f?2:(severity>0.05f?1:0));
    return out;
}
steering_result_t steering_control(int near,int preview,uint8_t pv,
    float middle,float direction,float low,float high)
{return desired((float)near,(float)preview,pv,middle,direction,low,high);}
void steering_reset(steering_state_t *s,float middle,uint32_t tick)
{
    steering_state_t empty={0};*s=empty;
    s->last_tick=tick;s->last.command=middle;s->last.speed_scale=1;
}
steering_result_t steering_update(steering_state_t *s,int near,int preview,
    uint8_t pv,uint8_t valid,uint8_t active,uint32_t tick,
    float middle,float direction,float low,float high)
{
    if(!valid || !active){
        steering_reset(s,middle,tick);
        steering_result_t out=desired((float)near,(float)preview,pv,middle,direction,low,high);
        out.command=middle;return out;
    }
    uint32_t elapsed=tick-s->last_tick;
    if(s->initialized && !elapsed)return s->last;
    float dt=(float)(elapsed>4?4:elapsed)*0.01f;
    if(!s->initialized){
        s->near_filter=(float)near;
        s->near_history[0]=s->near_history[1]=(float)near;
        s->initialized=1;dt=0.01f;
    }
    float n=median((float)near,s->near_history[0],s->near_history[1]);
    s->near_history[1]=s->near_history[0];s->near_history[0]=(float)near;
    if(pv){
        if(!s->preview_count){
            s->preview_history[0]=s->preview_history[1]=(float)preview;
            s->preview_filter=(float)preview;
        }
        float p=median((float)preview,s->preview_history[0],s->preview_history[1]);
        s->preview_history[1]=s->preview_history[0];s->preview_history[0]=(float)preview;
        if(s->preview_count<3)++s->preview_count;
        s->preview_filter+=dt/(0.04f+dt)*(p-s->preview_filter);
    }else s->preview_count=0; /* Missing preview is neither zero nor reusable history. */
    float tau=mag(n)>=20?0.02f:0.04f;
    s->near_filter+=dt/(tau+dt)*(n-s->near_filter);
    steering_result_t out=desired(s->near_filter,s->preview_filter,
        (uint8_t)(pv && s->preview_count>=2),middle,direction,low,high);
    const float thresholds[3]={0.05f,0.30f,0.70f};
    while(s->bend_level<3 && out.severity>thresholds[s->bend_level]+0.03f)++s->bend_level;
    while(s->bend_level>0 && out.severity<thresholds[s->bend_level-1]-0.03f)--s->bend_level;
    out.bend_level=s->bend_level;
    float rate=STEERING_SLEW_GENTLE+(STEERING_SLEW_TIGHT-STEERING_SLEW_GENTLE)*out.severity;
    float delta=out.command-s->last.command;
    out.slew_limited=(uint8_t)(mag(delta)>rate*dt+0.0001f);
    out.command=clip(s->last.command+clip(delta,-rate*dt,rate*dt),low,high);
    /* Remain slow while the commanded wheels are still returning from a bend. */
    float offset=out.command-middle;
    float span=offset>=0?high-middle:middle-low;
    float fraction=span>0?offset/span:0;
    float scale=1-(1-STEERING_TIGHT_CURVE_SPEED_SCALE)*clip((mag(fraction)-0.10f)/0.90f,0,1);
    if(out.speed_scale>scale)out.speed_scale=scale;
    float recovery=s->last.speed_scale+STEERING_SPEED_RECOVERY_PER_S*dt;
    if(out.speed_scale>recovery)out.speed_scale=recovery;
    out.curvature_reference=fraction*tanf(STEERING_MAX_WHEEL_ANGLE_RAD)/STEERING_WHEELBASE_M;
    s->last=out;s->last_tick=tick;return out;
}
