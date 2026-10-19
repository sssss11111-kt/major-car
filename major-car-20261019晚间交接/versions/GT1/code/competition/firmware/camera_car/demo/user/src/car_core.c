/* Application algorithm/control code. Distributed under GPL-3.0-or-later. */
#include "car_core.h"
#include <string.h>
static float clampf(float v,float lo,float hi) { return v<lo?lo:(v>hi?hi:v); }
static int abs_i(int v) { return v<0?-v:v; }
void vision_config_default(vision_config_t *c) {
    c->otsu=1; c->fixed_threshold=100; c->white_road=1;
    c->first_row=40; c->last_row=112; c->row_step=6;
    /* STEP-03: conservative geometry guards at 188x120, calibrate on real tracks. */
    c->edge_run=3;c->max_row_jump=24;c->max_width_change=24;
}
void vision_reset(vision_state_t *s) {
    s->previous_center=CAR_W/2;s->trusted=0;
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
    int prior_left=-1,prior_right=-1;
    int edge_run=c->edge_run<2?2:(c->edge_run>6?6:c->edge_run);
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
        /* STEP-03: require a supported dark run; internal two-pixel noise is not a road edge. */
        for(int x=found;x>=edge_run;x--) {
            int dark=1;for(int k=1;k<=edge_run;k++)if(row[x-k])dark=0;
            if(row[x] && dark){left=x;break;}
        }
        for(int x=found;x<CAR_W-edge_run;x++) {
            int dark=1;for(int k=1;k<=edge_run;k++)if(row[x+k])dark=0;
            if(row[x] && dark){right=x;break;}
        }
        if(left>=0 && right>=0) {
            int width=right-left;
            if(width<14 || width>CAR_W-10) continue;
            center=(left+right)/2;
            /* STEP-03: reject a sudden row-wise flare or large change from trusted history. */
            if((prior_left>=0 && c->max_row_jump &&
                (abs_i(left-prior_left)>c->max_row_jump || abs_i(right-prior_right)>c->max_row_jump)) ||
               (s->trusted && c->max_width_change && abs_i(width-s->width[y])>c->max_width_change)) {
                o->rejected_rows++;continue;
            }
            prior_left=left;prior_right=right;
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
        /* STEP-03: commit frame history below, only after acceptance. */
    }
    o->valid=(uint8_t)(o->scanned_rows>=6 && o->paired_rows*2>=o->scanned_rows
                      && o->valid_rows*3>=o->scanned_rows*2);
    o->quality=o->scanned_rows?(uint8_t)(100u*o->paired_rows/o->scanned_rows):0;
    if(o->valid) {
        s->previous_center=(int16_t)(weighted/weights);s->trusted=1;
        for(int y=first;y<=last;y++)if(o->edge_flags[y]==3)s->width[y]=(uint16_t)(o->right[y]-o->left[y]);
    }
}
void control_init(control_state_t *s,const control_config_t *c) {
    memset(s,0,sizeof *s); s->cfg=*c; s->servo_us=c->servo_center_us;
}
static void zero_outputs(control_state_t *s) {
    s->duty[0]=0; s->duty[1]=0;
    s->integral[0]=0; s->integral[1]=0;
    s->stalled_ticks[0]=0; s->stalled_ticks[1]=0;
    s->servo_us=s->cfg.servo_center_us;
    s->bench_target=0; s->bench_started=0;
    s->drive_start_ticks[0]=s->drive_start_ticks[1]=0;
    s->drive_lease_active=0;
}
void control_stop(control_state_t *s) {
    s->ground_done=0;
    s->low_done=0;
    s->sustain_phase=0;s->sustain_result=0;
    s->mode=CAR_IDLE; s->fault=FAULT_NONE;
    s->good_frames=0; s->bad_frames=0; s->derivative=0;
    s->trial_done=0;s->trial_elapsed_ms=0;
    zero_outputs(s);
}
void control_fault(control_state_t *s,car_fault_t f) {
    s->ground_done=0;
    s->low_done=0;
    s->sustain_phase=0;s->sustain_result=0;
    s->mode=CAR_FAULT; s->fault=f; s->good_frames=0;
    s->trial_done=0;s->trial_elapsed_ms=0;
    zero_outputs(s);
}
void control_frame(control_state_t *s,const vision_result_t *f,uint32_t now) {
    control_frame_at(s,f,now,now);
}
void control_frame_at(control_state_t *s,const vision_result_t *f,uint32_t captured,uint32_t now) {
    /* STEP-04: stale computation cannot rejuvenate an old camera capture. */
    if((uint32_t)(now-captured)>s->cfg.frame_timeout_ms) {
        s->good_frames=0;
        if(s->mode==CAR_RUNNING || s->mode==CAR_STEER_TEST)control_fault(s,FAULT_FRAME);
        return;
    }
    if(s->good_frames && (int32_t)(captured-s->frame_ms)<=0)return;
    now=captured;
    if(f->valid) {
        uint32_t dt=now-s->frame_ms;
        if(dt>s->cfg.frame_timeout_ms) s->good_frames=0;
        if(s->good_frames>0 && dt>0 && dt<=s->cfg.frame_timeout_ms)
            s->derivative=clampf((f->error-s->previous_error)*1000.0f/dt,-500,500);
        else s->derivative=0;
        s->frame_ms=now; s->previous_error=f->error; s->error=f->error;
        if(s->good_frames<255) s->good_frames++;
        s->bad_frames=0;
        if(s->mode==CAR_RUNNING || s->mode==CAR_STEER_TEST)
            s->servo_us=clampf(s->cfg.servo_center_us+s->cfg.steer_sign*
                (s->cfg.steer_kp*s->error+s->cfg.steer_kd*s->derivative),
                s->cfg.servo_min_us,s->cfg.servo_max_us);
    } else {
        s->good_frames=0;
        if(s->bad_frames<255) s->bad_frames++;
        if((s->mode==CAR_RUNNING || s->mode==CAR_STEER_TEST) && s->bad_frames>=3) control_fault(s,FAULT_LINE);
    }
}
int control_drive_long_test(control_state_t *s,uint32_t now) {
    if(!control_drive_test(s,0,now))return 0;
    s->test_until=now+10200u;s->drive_lease_active=1;s->drive_command_ms=now;return 1;
}
void control_drive_keepalive(control_state_t *s,uint32_t now) {
    /* STEP-02: p renews both the independent-drive and tracking trial lease. */
    if((s->mode==CAR_DRIVE_TEST || s->mode==CAR_RUNNING || s->mode==CAR_CALIBRATION || s->mode==CAR_SUSTAIN || s->mode==CAR_LOW_SPEED || s->mode==CAR_GROUND_TEST) && s->drive_lease_active)s->drive_command_ms=now;
}
int control_drive_test(control_state_t *s,int turn,uint32_t now) {
    if(s->mode!=CAR_IDLE || s->cfg.simulated || turn< -1 || turn>1 ||
       (uint32_t)(now-s->heartbeat_ms)>s->cfg.heartbeat_timeout_ms) return 0;
    zero_outputs(s);s->mode=CAR_DRIVE_TEST;s->fault=FAULT_NONE;
    s->test_motor=3;s->drive_turn=(int8_t)turn;
    /* Allow steering to settle at zero motor output, then drive for 1 second. */
    s->motion_ms=now+200u;s->test_until=now+1200u;
    s->servo_us=turn<0?s->cfg.servo_max_us:(turn>0?s->cfg.servo_min_us:s->cfg.servo_center_us);
    return 1;
}
int control_steer_test(control_state_t *s,uint32_t now) {
    if(s->mode!=CAR_IDLE || s->cfg.simulated || s->good_frames<3 ||
       (uint32_t)(now-s->frame_ms)>s->cfg.frame_timeout_ms ||
       (uint32_t)(now-s->heartbeat_ms)>s->cfg.heartbeat_timeout_ms) return 0;
    zero_outputs(s); s->derivative=0; s->test_until=now+30000u;
    s->mode=CAR_STEER_TEST; s->fault=FAULT_NONE; return 1;
}
int control_tracking_ready(const control_state_t *s,uint32_t now) {
    return s->mode==CAR_IDLE && !s->cfg.simulated && s->good_frames>=3 &&
       (uint32_t)(now-s->frame_ms)<=s->cfg.frame_timeout_ms &&
       (uint32_t)(now-s->heartbeat_ms)<=s->cfg.heartbeat_timeout_ms;
}
static int low_config_valid(const control_state_t *s);
int control_start(control_state_t *s,uint32_t now) {
    if(!control_tracking_ready(s,now) || (s->cfg.tracking_low_speed && (!low_config_valid(s) || s->cfg.speed_target<0 || s->cfg.speed_target>40))) return 0;
    /* STEP-02: aim from fresh vision before energizing either wheel. */
    zero_outputs(s);s->derivative=0;s->low_confirmed=0;s->low_regulating=0;
    s->servo_us=clampf(s->cfg.servo_center_us+s->cfg.steer_sign*s->cfg.steer_kp*s->error,
                      s->cfg.servo_min_us,s->cfg.servo_max_us);
    s->motion_ms=now+s->cfg.tracking_settle_ms;
    s->test_until=s->motion_ms+s->cfg.tracking_duration_ms;
    s->drive_lease_active=(uint8_t)(s->cfg.tracking_lease_ms>0);s->drive_command_ms=now;
    s->trial_id++;s->trial_started_ms=now;s->trial_elapsed_ms=0;s->trial_done=0;
    s->mode=CAR_RUNNING;s->fault=FAULT_NONE;
    return 1;
}
int control_test(control_state_t *s,uint8_t motor,uint32_t now) {
    if(s->mode!=CAR_IDLE || s->cfg.simulated || motor<1 || motor>2) return 0;
    zero_outputs(s); s->mode=CAR_TEST; s->test_motor=motor;
    s->test_until=now+2000u;
    return 1;
}
int control_calibration_test(control_state_t *s,uint8_t motor,uint16_t duty,uint32_t now) {
    if(s->mode!=CAR_IDLE || s->cfg.simulated || motor<1 || motor>2 || duty>CAR_CALIBRATION_MAX_DUTY ||
       (uint32_t)(now-s->heartbeat_ms)>s->cfg.heartbeat_timeout_ms)return 0;
    zero_outputs(s);s->mode=CAR_CALIBRATION;s->fault=FAULT_NONE;
    s->test_motor=motor;s->calibration_duty=duty;s->test_until=now+2000u;
    s->drive_lease_active=1;s->drive_command_ms=now;return 1;
}
int control_speed_test(control_state_t *s,uint8_t motor,uint32_t now) {
    if(s->mode!=CAR_IDLE || s->cfg.simulated || motor<1 || motor>3) return 0;
    zero_outputs(s); s->mode=CAR_SPEED_TEST; s->test_motor=motor;
    s->motion_ms=now; s->test_until=now+4000u;
    return 1;
}
int control_sustain_test(control_state_t *s,uint8_t motor,uint16_t duty,uint32_t now) {
    if(s->mode!=CAR_IDLE || s->cfg.simulated || motor<1 || motor>2 || duty>CAR_SUSTAIN_KICK_DUTY ||
       (uint32_t)(now-s->heartbeat_ms)>s->cfg.heartbeat_timeout_ms)return 0;
    zero_outputs(s);s->mode=CAR_SUSTAIN;s->fault=FAULT_NONE;s->test_motor=motor;
    s->sustain_duty=duty;s->sustain_id++;s->sustain_started=now;s->sustain_hold_started=0;
    s->sustain_total_ms=s->sustain_hold_ms=0;s->sustain_phase=1;s->sustain_result=0;
    s->sustain_n=s->sustain_zeros=s->sustain_stall_ticks=0;s->sustain_min=s->sustain_max=0;
    s->sustain_sum=0;s->sustain_kick_ticks=0;
    s->drive_lease_active=1;s->drive_command_ms=now;return 1;
}
int control_speed_test_right(control_state_t *s,uint32_t now) {
    return control_speed_test(s,2,now);
}
static int low_config_valid(const control_state_t *s) {
    if(!(s->cfg.low_kp>0 && s->cfg.low_kp<=20 && s->cfg.low_ki>0 && s->cfg.low_ki<=100))return 0;
    for(unsigned w=0;w<2;w++)if(!(s->cfg.low_ff_offset[w]>=0 && s->cfg.low_ff_offset[w]<=1000 && s->cfg.low_ff_slope[w]>0 && s->cfg.low_ff_slope[w]<=20))return 0;
    return 1;
}
int control_low_test(control_state_t *s,uint8_t motor,uint8_t target,uint32_t now) {
    if(s->mode!=CAR_IDLE || s->cfg.simulated || motor<1 || motor>3 || target<10 || target>40 ||
       !low_config_valid(s) ||
       (uint32_t)(now-s->heartbeat_ms)>s->cfg.heartbeat_timeout_ms)return 0;
    zero_outputs(s);s->mode=CAR_LOW_SPEED;s->fault=FAULT_NONE;s->test_motor=motor;
    s->low_id++;s->low_started=now;s->low_elapsed=0;s->low_done=0;s->low_confirmed=0;s->low_target=target;s->low_regulating=0;
    memset(s->low_n,0,sizeof s->low_n);memset(s->low_sum,0,sizeof s->low_sum);
    memset(s->low_error2,0,sizeof s->low_error2);memset(s->low_abs_error,0,sizeof s->low_abs_error);
    memset(s->low_min,0,sizeof s->low_min);memset(s->low_max,0,sizeof s->low_max);
    s->drive_lease_active=1;s->drive_command_ms=now;return 1;
}
/* Shared GL2 ground PI: interpolated loaded feedforward, no minimum PWM. */
static void ground_pi_step(control_state_t *s,unsigned w,int measured,float target) {
    const float slopes[2]={15.5f,15.1f};
    float ff=896.0f+slopes[w]*target;
    float error=target-measured;unsigned bit=1u<<w;
    int entering=!(s->low_regulating&bit);
    if(entering){s->low_regulating|=(uint8_t)bit;s->integral[w]=clampf(-4.0f*error,-500,500);}
    float candidate=clampf(s->integral[w]+(entering?0:20.0f*error*.01f),-500,500);
    float raw=ff+4.0f*error+candidate;
    if((raw>=0 && raw<=1500) || (raw>1500 && error<0) || (raw<0 && error>0))s->integral[w]=candidate;
    s->duty[w]=clampf(ff+4.0f*error+s->integral[w],0,1500);
}

/* GH2: bounded both-wheel ground hold; no vision steering or PI changes. */
int control_ground_test(control_state_t *s,uint16_t duty,uint32_t now) {
    if(s->mode!=CAR_IDLE || s->cfg.simulated || duty<900 || duty>2000 ||
       (uint32_t)(now-s->heartbeat_ms)>s->cfg.heartbeat_timeout_ms)return 0;
    zero_outputs(s);s->mode=CAR_GROUND_TEST;s->fault=FAULT_NONE;
    s->ground_id++;s->ground_started=now;s->ground_elapsed=0;s->ground_hold_started=0;
    s->ground_phase=0;s->ground_done=0;s->ground_closed=0;s->low_regulating=0;s->ground_duty=duty;s->test_motor=3;
    memset(s->low_n,0,sizeof s->low_n);memset(s->low_sum,0,sizeof s->low_sum);
    memset(s->low_min,0,sizeof s->low_min);memset(s->low_max,0,sizeof s->low_max);
    s->drive_lease_active=1;s->drive_command_ms=now;return 1;
}
int control_ground_closed_test(control_state_t *s,uint32_t now) {
    if(!control_ground_test(s,1200,now))return 0;
    s->ground_closed=1;s->ground_duty=0;
    memset(s->low_error2,0,sizeof s->low_error2);memset(s->low_abs_error,0,sizeof s->low_abs_error);
    return 1;
}
static void ground_tick(control_state_t *s,uint32_t now,int left,int right) {
    s->ground_elapsed=now-s->ground_started;
    s->servo_us=s->cfg.servo_center_us;
    if((uint32_t)(now-s->drive_command_ms)>=500){control_fault(s,FAULT_LINK);return;}
    int samples[2]={left,right};
    for(unsigned w=0;w<2;w++)if(samples[w]<-5 || samples[w]>CAR_GROUND_ENCODER_LIMIT){control_fault(s,FAULT_ENCODER);return;}
    /* Absolute cap is independent of foreground/host progress. */
    if(s->ground_elapsed>=CAR_GROUND_MAX_MS){control_stop(s);return;}
    if(s->ground_elapsed<200){s->duty[0]=s->duty[1]=0;return;}
    if(s->ground_phase<2){
        s->ground_phase=1;
        for(unsigned w=0;w<2;w++){
            if(samples[w]>=5){if(s->drive_start_ticks[w]<3)s->drive_start_ticks[w]++;}else s->drive_start_ticks[w]=0;
            if(s->drive_start_ticks[w]>=3)s->bench_started|=(uint8_t)(1u<<w);
        }
        if(s->ground_elapsed>=400 && s->bench_started==3){
            s->ground_phase=2;s->ground_hold_started=now;
        }else if(s->ground_elapsed>=500){control_fault(s,FAULT_ENCODER);return;}
        else{s->duty[0]=s->duty[1]=3000;return;}
    }
    uint32_t held=now-s->ground_hold_started;
    if(held>=CAR_GROUND_HOLD_MS){control_stop(s);s->ground_done=1;s->ground_phase=3;return;}
    for(unsigned w=0;w<2;w++){
        if(s->ground_closed){
            /* GL2 candidate: two-point loaded interpolation at target20.
             * Seed at the estimated feedforward by cancelling entry P; PI may lower
             * output to zero. No fixed PWM floor or automatic re-kick. */
            ground_pi_step(s,w,samples[w],20);
            s->bench_target=20;
        }else s->duty[w]=s->ground_duty;
        if(samples[w]<2)s->stalled_ticks[w]++;else s->stalled_ticks[w]=0;
        if(s->stalled_ticks[w]>=50){control_fault(s,FAULT_ENCODER);return;}
        if(held>=1000){
            if(!s->low_n[w])s->low_min[w]=s->low_max[w]=(int16_t)samples[w];
            if(samples[w]<s->low_min[w])s->low_min[w]=(int16_t)samples[w];
            if(samples[w]>s->low_max[w])s->low_max[w]=(int16_t)samples[w];
            s->low_n[w]++;s->low_sum[w]+=samples[w];
            if(s->ground_closed){int e=20-samples[w];s->low_error2[w]+=(uint32_t)(e*e);s->low_abs_error[w]+=(uint32_t)abs_i(e);}
        }
    }
}

static int calibrated_wheel_step(control_state_t *s,unsigned w,int measured,float target,uint32_t elapsed) {
    unsigned bit=1u<<w;float error=target-measured;
        if(!(s->low_confirmed&bit)) {
            if(measured>=5){if(s->drive_start_ticks[w]<3)s->drive_start_ticks[w]++;}else s->drive_start_ticks[w]=0;
            if(elapsed>=30 && s->drive_start_ticks[w]>=3)s->low_confirmed|=(uint8_t)bit;
            else if(elapsed>=300){control_fault(s,FAULT_ENCODER);return 0;}
            else {s->duty[w]=CAR_LOW_KICK;return 1;}
        }
        if(elapsed<200){s->duty[w]=CAR_LOW_KICK;return 1;}
        if(s->cfg.tracking_ground && s->mode==CAR_RUNNING){
            ground_pi_step(s,w,measured,target);
            if(measured<2)s->stalled_ticks[w]++;else s->stalled_ticks[w]=0;
            if(s->stalled_ticks[w]>=50){control_fault(s,FAULT_ENCODER);return 0;}
            return 1;
        }
        float ff=s->cfg.low_ff_offset[w]+s->cfg.low_ff_slope[w]*target;
        /* L3: settle on measured feedforward, then match the existing output
         * on PI entry. Stall/feedback guards apply throughout this transfer. */
        if(elapsed<1000){s->duty[w]=clampf(ff,0,CAR_LOW_CAP);s->integral[w]=0;}
        else {
            int entering=!(s->low_regulating&bit);
            if(entering){s->low_regulating|=(uint8_t)bit;s->integral[w]=clampf(-s->cfg.low_kp*error,-500,500);}
            float candidate=clampf(s->integral[w]+(entering?0:s->cfg.low_ki*error*.01f),-500,500);
            float raw=ff+s->cfg.low_kp*error+candidate;
            if((raw>=0 && raw<=CAR_LOW_CAP) || (raw>CAR_LOW_CAP && error<0) || (raw<0 && error>0))s->integral[w]=candidate;
            s->duty[w]=clampf(ff+s->cfg.low_kp*error+s->integral[w],0,CAR_LOW_CAP);
        }
        if(measured<2)s->stalled_ticks[w]++;else s->stalled_ticks[w]=0;
        if(s->stalled_ticks[w]>=50){control_fault(s,FAULT_ENCODER);return 0;}
    return 1;
}
static void low_speed_tick(control_state_t *s,uint32_t now,int16_t left,int16_t right) {
    s->low_elapsed=now-s->low_started;
    if((uint32_t)(now-s->drive_command_ms)>=CAR_LOW_LEASE_MS){control_fault(s,FAULT_LINK);return;}
    int samples[2]={left,right};
    for(unsigned w=0;w<2;w++)if((s->test_motor&(1u<<w))?
        (samples[w]<-5 || samples[w]>450):(abs_i(samples[w])>10)) {control_fault(s,FAULT_ENCODER);return;}
    if(s->low_elapsed>=CAR_LOW_DURATION_MS){control_stop(s);s->low_done=1;return;}
    s->bench_target=s->low_target;
    for(unsigned w=0;w<2;w++) {
        unsigned bit=1u<<w;if(!(s->test_motor&bit)){s->duty[w]=0;continue;}
        int measured=samples[w],error=(int)s->low_target-measured;
        if(!calibrated_wheel_step(s,w,measured,s->low_target,s->low_elapsed))return;
        if(s->low_elapsed>=CAR_LOW_STATS_MS) {
            if(!s->low_n[w])s->low_min[w]=s->low_max[w]=(int16_t)measured;
            if(measured<s->low_min[w])s->low_min[w]=(int16_t)measured;
            if(measured>s->low_max[w])s->low_max[w]=(int16_t)measured;
            s->low_n[w]++;s->low_sum[w]+=measured;
            s->low_error2[w]+=(uint32_t)(error*error);s->low_abs_error[w]+=(uint32_t)abs_i(error);
        }
    }
}
/* STEP-01: one wheel controller shared by bench, drive and vision tracking.
 * It never writes servo_us; visual steering remains owned by control_frame. */
static int wheel_speed_step(control_state_t *s,unsigned w,int measured,float target,
                            uint32_t elapsed,int loaded,float kp,float ki,float cap,float bias) {
    unsigned bit=1u<<w;
    if(!(s->bench_started&bit)) {
        if(loaded) {
            if(measured>=5) {if(s->drive_start_ticks[w]<3)s->drive_start_ticks[w]++;}
            else s->drive_start_ticks[w]=0;
        }
        if(loaded?s->drive_start_ticks[w]>=3:measured>=20) s->bench_started|=(uint8_t)bit;
        else if(elapsed>=200) {control_fault(s,FAULT_ENCODER);return 0;}
        else {s->duty[w]=clampf(3000,0,cap);return 1;}
    }
    /* T2 STEP-01: bias may be reduced by PI; zero/stop guards stay outside. */
    bias=clampf(bias>0?bias:1500.0f,0,cap);
    float error=target-measured;
    float limit=cap<1500?cap:1500;
    float candidate=clampf(s->integral[w]+ki*error*.01f,-limit,limit);
    float raw=bias+kp*error+candidate;
    if((raw>=0 && raw<=cap) || (raw>cap && error<0) || (raw<0 && error>0))
        s->integral[w]=candidate;
    s->duty[w]=clampf(bias+kp*error+s->integral[w],0,cap);
    if(s->duty[w]>=500 && measured<2)s->stalled_ticks[w]++;
    else s->stalled_ticks[w]=0;
    if(s->stalled_ticks[w]>=50){control_fault(s,FAULT_ENCODER);return 0;}
    return 1;
}
static void speed_bench_tick(control_state_t *s,uint32_t now,int16_t left,int16_t right) {
    if((int32_t)(now-s->test_until)>=0){control_stop(s);return;}
    unsigned mask=s->test_motor==3?3u:(1u<<(s->test_motor-1u));
    int samples[2]={left,right};
    for(unsigned w=0;w<2;w++) {
        if((mask&(1u<<w))?(samples[w]<-5 || samples[w]>450):(abs_i(samples[w])>10)) {
            control_fault(s,FAULT_ENCODER);return;
        }
    }
    uint32_t elapsed=now-s->motion_ms;
    int ground=s->mode==CAR_DRIVE_TEST;
    s->bench_target=(ground?40.0f:80.0f)*clampf(elapsed/(ground?500.0f:1000.0f),0,1);
    s->servo_us=ground?(s->drive_turn<0?s->cfg.servo_max_us:
        (s->drive_turn>0?s->cfg.servo_min_us:s->cfg.servo_center_us)):s->cfg.servo_center_us;
    for(unsigned w=0;w<2;w++) {
        if(!(mask&(1u<<w))){s->duty[w]=0;continue;}
        if(!wheel_speed_step(s,w,samples[w],s->bench_target,elapsed,ground,4,20,3000,1500))return;
    }
}
void control_tick(control_state_t *s,uint32_t now,int16_t left,int16_t right) {
    if(s->mode!=CAR_RUNNING && s->mode!=CAR_TEST && s->mode!=CAR_SPEED_TEST && s->mode!=CAR_STEER_TEST && s->mode!=CAR_DRIVE_TEST && s->mode!=CAR_CALIBRATION && s->mode!=CAR_SUSTAIN && s->mode!=CAR_LOW_SPEED && s->mode!=CAR_GROUND_TEST) { zero_outputs(s); return; }
    if(s->cfg.simulated) { control_stop(s); return; }
    if((uint32_t)(now-s->heartbeat_ms)>s->cfg.heartbeat_timeout_ms) {
        control_fault(s,FAULT_FOREGROUND); return;
    }
    if(s->mode==CAR_GROUND_TEST){ground_tick(s,now,left,right);return;}
    if(s->mode==CAR_LOW_SPEED){low_speed_tick(s,now,left,right);return;}
    if(s->mode==CAR_SUSTAIN) {
        s->sustain_total_ms=now-s->sustain_started;
        if((uint32_t)(now-s->drive_command_ms)>=CAR_SUSTAIN_LEASE_MS){control_fault(s,FAULT_LINK);return;}
        int active=s->test_motor==1?left:right,other=s->test_motor==1?right:left;
        if(active< -5 || active>CAR_CALIBRATION_ENCODER_LIMIT || abs_i(other)>10){control_fault(s,FAULT_ENCODER);return;}
        if(s->sustain_phase==1) {
            if(active>=5){if(s->sustain_kick_ticks<3)s->sustain_kick_ticks++;}
            else s->sustain_kick_ticks=0;
            if(s->sustain_total_ms>=CAR_SUSTAIN_KICK_MIN_MS && s->sustain_kick_ticks>=3){
                s->sustain_phase=2;s->sustain_hold_started=now;
            }else if(s->sustain_total_ms>=CAR_SUSTAIN_KICK_MAX_MS){control_fault(s,FAULT_ENCODER);return;}
        }
        if(s->sustain_phase==2) {
            s->sustain_hold_ms=now-s->sustain_hold_started;
            if(s->sustain_hold_ms>=CAR_SUSTAIN_HOLD_MS){control_stop(s);s->sustain_phase=3;s->sustain_result=1;return;}
            if(active<2)s->sustain_stall_ticks++;else s->sustain_stall_ticks=0;
            if(s->sustain_hold_ms>=CAR_SUSTAIN_STATS_AFTER_MS){
                if(!s->sustain_n){s->sustain_min=(int16_t)active;s->sustain_max=(int16_t)active;}
                if(active<s->sustain_min)s->sustain_min=(int16_t)active;
                if(active>s->sustain_max)s->sustain_max=(int16_t)active;
                s->sustain_sum+=active;s->sustain_n++;if(active<2)s->sustain_zeros++;
            }
            if(s->sustain_stall_ticks>=50){control_stop(s);s->sustain_phase=3;s->sustain_result=2;return;}
        }
        unsigned w=s->test_motor-1u;s->duty[0]=s->duty[1]=0;
        s->duty[w]=s->sustain_phase==1?(float)CAR_SUSTAIN_KICK_DUTY:s->sustain_duty;
        return;
    }
    if(s->mode==CAR_CALIBRATION) {
        if((uint32_t)(now-s->drive_command_ms)>=500u){control_fault(s,FAULT_LINK);return;}
        if((int32_t)(now-s->test_until)>=0){control_stop(s);return;}
        int active=s->test_motor==1?left:right,other=s->test_motor==1?right:left;
        if(active< -5 || active>CAR_CALIBRATION_ENCODER_LIMIT || abs_i(other)>10){control_fault(s,FAULT_ENCODER);return;}
        s->duty[0]=s->test_motor==1?s->calibration_duty:0;
        s->duty[1]=s->test_motor==2?s->calibration_duty:0;
        return;
    }
    if(s->mode==CAR_STEER_TEST) {
        s->duty[0]=s->duty[1]=0;
        if((int32_t)(now-s->test_until)>=0) { control_stop(s); return; }
        if((uint32_t)(now-s->frame_ms)>s->cfg.frame_timeout_ms)
            control_fault(s,FAULT_FRAME);
        return;
    }
    if(s->mode==CAR_DRIVE_TEST) {
        if(s->drive_lease_active && (uint32_t)(now-s->drive_command_ms)>=500u) {
            control_fault(s,FAULT_LINK);return;
        }
        if((int32_t)(now-s->motion_ms)<0) {s->duty[0]=s->duty[1]=0;return;}
        speed_bench_tick(s,now,left,right);return;
    }
    if(s->mode==CAR_SPEED_TEST) {speed_bench_tick(s,now,left,right);return;}
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
    /* STEP-02: only a fresh explicit start arms these guards; stop never renews it. */
    if(s->drive_lease_active && (uint32_t)(now-s->drive_command_ms)>=s->cfg.tracking_lease_ms) {
        control_fault(s,FAULT_LINK);return;
    }
    if(s->cfg.tracking_duration_ms && (int32_t)(now-s->test_until)>=0) {
        uint32_t elapsed=now-s->trial_started_ms;
        control_stop(s);s->trial_elapsed_ms=elapsed;s->trial_done=1;return;
    }
    /* STEP-06 review: validate feedback even during zero-output steering preparation. */
    int feedback_limit=s->cfg.tracking_ground?120:450;
    if(left< -5 || right< -5 || abs_i(left)>feedback_limit || abs_i(right)>feedback_limit) {
        control_fault(s,FAULT_ENCODER); return;
    }
    if((int32_t)(now-s->motion_ms)<0){s->duty[0]=s->duty[1]=0;return;}
    float slowdown=clampf(1.0f-(s->error<0?-s->error:s->error)/150.0f,0.5f,1);
    float ramp=clampf((uint32_t)(now-s->motion_ms)/500.0f,0,1);
    float target=s->cfg.speed_target*slowdown*(s->cfg.tracking_low_speed?1.0f:ramp);
    s->bench_target=target; /* STEP-01: actual ramped target also exposed to telemetry. */
    if(s->cfg.speed_target<=0) {
        s->duty[0]=s->duty[1]=0;s->integral[0]=s->integral[1]=0;
        s->bench_started=0;s->drive_start_ticks[0]=s->drive_start_ticks[1]=0;
        s->low_confirmed=0;s->low_regulating=0;s->motion_ms=now;return;
    }
    int samples[2]={left,right};
    if(s->cfg.tracking_low_speed){
        for(unsigned w=0;w<2;w++)if(!calibrated_wheel_step(s,w,samples[w],target,now-s->motion_ms))return;
        s->bench_started=s->low_confirmed;return;
    }
    for(unsigned w=0;w<2;w++)
        if(!wheel_speed_step(s,w,samples[w],target,now-s->motion_ms,1,
                            s->cfg.speed_kp,s->cfg.speed_ki,s->cfg.pwm_max,
                            s->cfg.tracking_pwm_bias[w]))return;
}
