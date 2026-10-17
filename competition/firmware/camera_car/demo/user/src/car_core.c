/* Application algorithm/control code. Distributed under GPL-3.0-or-later. */
#include "car_core.h"
#include <string.h>
static float clampf(float v,float lo,float hi) { return v<lo?lo:(v>hi?hi:v); }
static int abs_i(int v) { return v<0?-v:v; }
void vision_config_default(vision_config_t *c) {
    c->otsu=1; c->fixed_threshold=100; c->white_road=1;
    c->first_row=40; c->last_row=112; c->row_step=6;
}
void vision_reset(vision_state_t *s) {
    s->previous_center=CAR_W/2;
    for(int y=0;y<CAR_H;y++) s->width[y]=(uint16_t)(30+y);
}
static uint8_t threshold_otsu(const uint32_t *hist,uint32_t total) {
    double sum=0,back_sum=0,best=-1;
    uint32_t back=0;
    uint8_t threshold=100;
    for(int i=0;i<256;i++) sum+=(double)i*hist[i];
    for(int i=0;i<255;i++) {
        double delta,score;
        back+=hist[i]; back_sum+=(double)i*hist[i];
        if(!back || back==total) continue;
        delta=back_sum/back-(sum-back_sum)/(total-back);
        score=(double)back*(total-back)*delta*delta;
        if(score>best) { best=score; threshold=(uint8_t)i; }
    }
    return threshold;
}
void vision_process(vision_state_t *s,const vision_config_t *c,const uint8_t *g,
                    uint8_t *b,vision_result_t *o) {
    uint32_t hist[256]={0},total=0;
    int low=255,high=0,first=c->first_row,last=c->last_row,step=c->row_step;
    float weighted=0,weights=0;
    int seed=s->previous_center;
    memset(o,0,sizeof *o);
    for(int y=0;y<CAR_H;y++) { o->left[y]=-1; o->right[y]=-1; o->center[y]=-1; }
    if(first<2) first=2;
    if(last>=CAR_H) last=CAR_H-1;
    if(first>last || step<1) return;
    for(int y=first;y<=last;y++) for(int x=0;x<CAR_W;x++) {
        int v=g[y*CAR_W+x]; hist[v]++; total++;
    }
    uint32_t accumulated=0;
    for(int i=0;i<256;i++) { accumulated+=hist[i]; if(accumulated>total/100) { low=i; break; } }
    accumulated=0;
    for(int i=255;i>=0;i--) { accumulated+=hist[i]; if(accumulated>total/100) { high=i; break; } }
    o->contrast=(uint8_t)(high-low);
    o->threshold=c->otsu?threshold_otsu(hist,total):c->fixed_threshold;
    for(int i=0;i<CAR_W*CAR_H;i++) {
        int white=g[i]>o->threshold;
        b[i]=(uint8_t)((c->white_road?white:!white)?255:0);
    }
    if(o->contrast<16) return;
    for(int y=last;y>=first;y-=step) {
        const uint8_t *row=b+y*CAR_W;
        int left=-1,right=-1,center=-1,found=-1;
        o->scanned_rows++;
        seed=seed<3?3:(seed>CAR_W-4?CAR_W-4:seed);
        for(int radius=0;radius<=24 && found<0;radius++) {
            int x=seed-radius;
            if(x>=2 && row[x] && row[x-1] && row[x+1]) found=x;
            x=seed+radius;
            if(found<0 && x<CAR_W-2 && row[x] && row[x-1] && row[x+1]) found=x;
        }
        if(found<0) continue;
        for(int x=found;x>=2;x--) if(row[x] && !row[x-1] && !row[x-2]) { left=x; break; }
        for(int x=found;x<CAR_W-2;x++) if(row[x] && !row[x+1] && !row[x+2]) { right=x; break; }
        if(left>=0 && right>=0) {
            int width=right-left;
            if(width<14 || width>CAR_W-10) continue;
            center=(left+right)/2;
            s->width[y]=(uint16_t)width;
            o->paired_rows++;
        } else if(left>=0) center=left+s->width[y]/2;
        else if(right>=0) center=right-s->width[y]/2;
        if(center<2 || center>=CAR_W-2) continue;
        o->left[y]=(int16_t)left; o->right[y]=(int16_t)right;
        o->center[y]=(int16_t)center;
        o->edge_flags[y]=(uint8_t)((left>=0?1:0)|(right>=0?2:0));
        o->valid_rows++;
        float weight=(float)(y-first+step);
        if(left<0 || right<0) weight*=0.25f;
        weighted+=center*weight; weights+=weight;
        seed=center;
    }
    if(weights>0) {
        float center=weighted/weights;
        o->error=center-(CAR_W-1)*0.5f;
        if(o->paired_rows>=3) s->previous_center=(int16_t)center;
    }
    o->valid=(uint8_t)(o->scanned_rows>=6 && o->paired_rows*2>=o->scanned_rows
                      && o->valid_rows*3>=o->scanned_rows*2);
}
void control_init(control_state_t *s,const control_config_t *c) {
    memset(s,0,sizeof *s); s->cfg=*c; s->servo_us=c->servo_center_us;
}
static void zero_outputs(control_state_t *s) {
    s->duty[0]=0; s->duty[1]=0;
    s->integral[0]=0; s->integral[1]=0;
    s->stalled_ticks[0]=0; s->stalled_ticks[1]=0;
    s->servo_us=s->cfg.servo_center_us;
}
void control_stop(control_state_t *s) {
    s->mode=CAR_IDLE; s->fault=FAULT_NONE;
    s->good_frames=0; s->bad_frames=0; s->derivative=0;
    zero_outputs(s);
}
void control_fault(control_state_t *s,car_fault_t f) {
    s->mode=CAR_FAULT; s->fault=f; s->good_frames=0;
    zero_outputs(s);
}
void control_frame(control_state_t *s,const vision_result_t *f,uint32_t now) {
    if(f->valid) {
        uint32_t dt=now-s->frame_ms;
        if(dt>s->cfg.frame_timeout_ms) s->good_frames=0;
        if(s->good_frames>0 && dt>0 && dt<=s->cfg.frame_timeout_ms)
            s->derivative=clampf((f->error-s->previous_error)*1000.0f/dt,-500,500);
        else s->derivative=0;
        s->frame_ms=now; s->previous_error=f->error; s->error=f->error;
        if(s->good_frames<255) s->good_frames++;
        s->bad_frames=0;
        if(s->mode==CAR_RUNNING)
            s->servo_us=clampf(s->cfg.servo_center_us+s->cfg.steer_sign*
                (s->cfg.steer_kp*s->error+s->cfg.steer_kd*s->derivative),
                s->cfg.servo_min_us,s->cfg.servo_max_us);
    } else {
        s->good_frames=0;
        if(s->bad_frames<255) s->bad_frames++;
        if(s->mode==CAR_RUNNING && s->bad_frames>=3) control_fault(s,FAULT_LINE);
    }
}
int control_start(control_state_t *s,uint32_t now) {
    if(s->mode!=CAR_IDLE || s->cfg.simulated || s->good_frames<3 ||
       (uint32_t)(now-s->frame_ms)>s->cfg.frame_timeout_ms ||
       (uint32_t)(now-s->heartbeat_ms)>s->cfg.heartbeat_timeout_ms) return 0;
    zero_outputs(s); s->motion_ms=now; s->mode=CAR_RUNNING; s->fault=FAULT_NONE;
    return 1;
}
int control_test(control_state_t *s,uint8_t motor,uint32_t now) {
    if(s->mode!=CAR_IDLE || s->cfg.simulated || motor<1 || motor>2) return 0;
    zero_outputs(s); s->mode=CAR_TEST; s->test_motor=motor;
    s->test_until=now+2000u;
    return 1;
}
static float pi_step(control_state_t *s,int wheel,float error) {
    float candidate=clampf(s->integral[wheel]+s->cfg.speed_ki*error*0.01f,
                           -s->cfg.pwm_max,s->cfg.pwm_max);
    float raw=s->cfg.speed_kp*error+candidate;
    if((raw>=0 && raw<=s->cfg.pwm_max) || (raw>s->cfg.pwm_max && error<0) ||
       (raw<0 && error>0)) s->integral[wheel]=candidate;
    return clampf(s->cfg.speed_kp*error+s->integral[wheel],0,s->cfg.pwm_max);
}
void control_tick(control_state_t *s,uint32_t now,int16_t left,int16_t right) {
    if(s->mode!=CAR_RUNNING && s->mode!=CAR_TEST) { zero_outputs(s); return; }
    if(s->cfg.simulated) { control_stop(s); return; }
    if((uint32_t)(now-s->heartbeat_ms)>s->cfg.heartbeat_timeout_ms) {
        control_fault(s,FAULT_FOREGROUND); return;
    }
    if(s->mode==CAR_TEST) {
        if((int32_t)(now-s->test_until)>=0) { control_stop(s); return; }
        /* Bench-only single-wheel test: 30%, bounded by test_until and watchdog. */
        s->duty[0]=s->test_motor==1?3000.0f:0;
        s->duty[1]=s->test_motor==2?3000.0f:0;
        return;
    }
    if((uint32_t)(now-s->frame_ms)>s->cfg.frame_timeout_ms) {
        control_fault(s,FAULT_FRAME); return;
    }
    if(left< -5 || right< -5 || abs_i(left)>2000 || abs_i(right)>2000) {
        control_fault(s,FAULT_ENCODER); return;
    }
    float slowdown=clampf(1.0f-(s->error<0?-s->error:s->error)/150.0f,0.5f,1);
    float ramp=clampf((uint32_t)(now-s->motion_ms)/500.0f,0,1);
    float target=s->cfg.speed_target*slowdown*ramp;
    s->duty[0]=pi_step(s,0,target-left);
    s->duty[1]=pi_step(s,1,target-right);
    int samples[2]={left,right};
    for(int i=0;i<2;i++) {
        if(s->duty[i]>=500 && abs_i(samples[i])<2) s->stalled_ticks[i]++;
        else s->stalled_ticks[i]=0;
        if(s->stalled_ticks[i]>=100) { control_fault(s,FAULT_ENCODER); return; }
    }
}
