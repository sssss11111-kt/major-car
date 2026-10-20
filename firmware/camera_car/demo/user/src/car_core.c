/* Application algorithm/control code. Distributed under GPL-3.0-or-later. */
#include "car_core.h"
#include <string.h>
static float clampf(float v,float lo,float hi) { return v<lo?lo:(v>hi?hi:v); }
static int abs_i(int v) { return v<0?-v:v; }
void vision_config_default(vision_config_t *c) {
    c->otsu=1; c->fixed_threshold=100; c->white_road=1;
    c->first_row=40; c->last_row=112; c->row_step=6;
}
/* Width history is calibration evidence, never a fabricated default road. */
void vision_reset(vision_state_t *s) {
    memset(s,0,sizeof *s);
    s->previous_center=CAR_W/2;
    memset(s->width_age,255,sizeof s->width_age);
}
static uint8_t threshold_otsu(const uint32_t *hist,uint32_t total) {
    float sum=0,back_sum=0,best=-1;
    uint32_t back=0;
    uint8_t threshold=100;
    for(int i=0;i<256;i++) sum+=(float)i*hist[i];
    for(int i=0;i<255;i++) {
        float delta,score;
        if(!hist[i]) continue;
        back+=hist[i]; back_sum+=(float)i*hist[i];
        if(!back || back==total) continue;
        delta=back_sum/back-(sum-back_sum)/(total-back);
        score=(float)back*(total-back)*delta*delta;
        if(score>best) { best=score; threshold=(uint8_t)i; }
    }
    return threshold;
}
static int histogram_contrast(const uint32_t *hist,uint32_t total) {
    uint32_t n=0;
    int low=0,high=255;
    for(int i=0;i<256;i++) { n+=hist[i]; if(n>total/100) { low=i; break; } }
    n=0;
    for(int i=255;i>=0;i--) { n+=hist[i]; if(n>total/100) { high=i; break; } }
    return high-low;
}
/* A threshold split alone is not an edge: smooth illumination also splits.
   Compare actual samples on both sides; polarity is road-minus-background. */
static int edge_strength(const uint8_t *row,int x,int left,int white) {
    int inside=0,outside=0,n=0;
    for(int k=0;k<3;k++) {
        int a=left?x+k:x-k, b=left?x-1-k:x+1+k;
        if(a<0 || b<0 || a>=CAR_W || b>=CAR_W) break;
        inside+=row[a]; outside+=row[b]; n++;
    }
    if(n<2) return 0;
    return (white?inside-outside:outside-inside)/n;
}
/* Fit width against row using CURRENT, connected double-edge observations.
   A fit can sustain a bend beyond history lifetime; no fixed perspective
   formula or synthetic edge is fed back into this fit. */
static int model_width(const vision_result_t *o,int y,int last,int step) {
    float sy=0,sw=0,syy=0,syw=0,n=0;
    int ymin=last,ymax=0;
    for(int r=last;r>y;r-=step) if(o->edge_flags[r]==3) {
        float w=(float)(o->right[r]-o->left[r]);
        n++; sy+=r; sw+=w; syy+=(float)r*r; syw+=r*w;
        if(r<ymin) ymin=r;
        if(r>ymax) ymax=r;
    }
    if(n<3 || ymax-ymin<12) return 0;
    float denominator=n*syy-sy*sy;
    if(denominator<=0) return 0;
    float slope=(n*syw-sy*sw)/denominator, intercept=(sw-slope*sy)/n;
    if(slope< -0.1f || slope>2.0f) return 0;
    for(int r=last;r>y;r-=step) if(o->edge_flags[r]==3) {
        float residual=o->right[r]-o->left[r]-(intercept+slope*r);
        if(residual< -3 || residual>3) return 0;
    }
    int width=(int)(intercept+slope*y+0.5f);
    return width>=14 && width<=CAR_W-10?width:0;
}
void vision_process(vision_state_t *s,const vision_config_t *c,const uint8_t *g,
                    uint8_t *b,vision_result_t *o) {
    uint32_t global_hist[256]={0};
    uint8_t row_contrast[CAR_H]={0};
    int first=c->first_row,last=c->last_row,step=c->row_step;
    int near_pairs=0,below_y=-1,below_center=0,below_left=0,below_right=0;
    int near_center=-1,previous_y=-1,previous_center=0;
    float weighted=0,weights=0;
    memset(o,0,sizeof *o);
    memset(b,0,CAR_W*CAR_H);
    for(int y=0;y<CAR_H;y++) {
        o->left[y]=-1; o->right[y]=-1; o->center[y]=-1;
        if(s->width_age[y]<255) s->width_age[y]++;
    }
    if(s->history_valid && s->polarity!=c->white_road) vision_reset(s);
    if(first<2) first=2;
    if(last>=CAR_H) last=CAR_H-1;
    if(first>last || step<1) { s->history_valid=0; return; }
    o->scanned_rows=(uint16_t)((last-first)/step+1);
    /* Row-local histograms also generate a truthful full-resolution preview. */
    for(int y=0;y<CAR_H;y++) {
        uint32_t hist[256]={0};
        const uint8_t *row=g+y*CAR_W;
        uint8_t *dst=b+y*CAR_W;
        for(int x=0;x<CAR_W;x++) hist[row[x]]++;
        if(y>=first && y<=last)
            for(int i=0;i<256;i++) global_hist[i]+=hist[i];
        row_contrast[y]=(uint8_t)histogram_contrast(hist,CAR_W);
        o->row_threshold[y]=c->otsu?threshold_otsu(hist,CAR_W):c->fixed_threshold;
        for(int x=0;x<CAR_W;x++) {
            int white=row[x]>o->row_threshold[y];
            dst[x]=(uint8_t)((c->white_road?white:!white)?255:0);
        }
        /* Close only isolated one-pixel holes, preserving >=2px dark borders. */
        for(int x=1;x<CAR_W-1;x++)
            if(!dst[x] && dst[x-1] && dst[x+1]) dst[x]=255;
    }
    uint32_t total=(uint32_t)(last-first+1)*CAR_W;
    o->contrast=(uint8_t)histogram_contrast(global_hist,total);
    o->threshold=c->otsu?threshold_otsu(global_hist,total):c->fixed_threshold;
    for(int y=last;y>=first;y-=step) {
        const uint8_t *row=b+y*CAR_W, *raw=g+y*CAR_W;
        if(row_contrast[y]<16) continue;
        int dy=below_y<0?step:below_y-y;
        /* Do not jump across a long invisible section to a different road. */
        if(below_y>=0 && dy>3*step) break;
        int prediction=s->history_valid?s->previous_center:CAR_W/2;
        if(below_y>=0) {
            prediction=below_center;
            if(previous_y>below_y) {
                int motion=(below_center-previous_center)*dy/(previous_y-below_y);
                if(motion>2*dy) motion=2*dy;
                if(motion< -2*dy) motion=-2*dy;
                prediction+=motion;
            }
        }
        int learned=(s->history_valid && s->width_age[y]<=8)?s->width[y]:0;
        int fitted=model_width(o,y,last,step);
        int expected=fitted?fitted:learned;
        int best_score=100000,best_center=-1,best_left=-1,best_right=-1;
        int best_start=0,best_end=0;
        for(int x=0;x<CAR_W;) {
            if(!row[x]) { x++; continue; }
            int start=x;
            while(x<CAR_W && row[x]) x++;
            int end=x-1, span=end-start;
            if(span<6) continue;
            int left=start>=2?start:-1, right=end<CAR_W-2?end:-1;
            if(left>=0 && edge_strength(raw,left,1,c->white_road)<10) continue;
            if(right>=0 && edge_strength(raw,right,0,c->white_road)<10) continue;
            if(left<0 && right<0) continue;
            int center;
            if(left>=0 && right>=0) {
                if(span<14 || span>CAR_W-10) continue;
                center=(left+right)/2;
                /* History is a bounded geometry check, not a forced center. */
                if(learned && abs_i(span-learned)>learned/3+6) continue;
            } else {
                if(!expected || near_pairs<2 || span>expected+2) continue;
                int hidden=left>=0?left+expected:right-expected;
                if((left>=0 && hidden<CAR_W-2) || (right>=0 && hidden>1)) continue;
                center=left>=0?left+expected/2:right-expected/2;
            }
            if(center<2 || center>=CAR_W-2) continue;
            int limit=below_y>=0?2*dy+8:48;
            if(abs_i(center-prediction)>limit) continue;
            if(below_y>=0 && (end<below_left || start>below_right)) continue;
            int score=4*abs_i(center-prediction);
            if(learned && left>=0 && right>=0) score+=abs_i(span-learned);
            if(left<0 || right<0) score+=8;
            if(score<best_score) {
                best_score=score; best_center=center; best_left=left; best_right=right;
                best_start=start; best_end=end;
            }
        }
        if(best_center<0) continue;
        o->left[y]=(int16_t)best_left; o->right[y]=(int16_t)best_right;
        o->center[y]=(int16_t)best_center;
        o->edge_flags[y]=(uint8_t)((best_left>=0?1:0)|(best_right>=0?2:0));
        o->valid_rows++;
        if(o->edge_flags[y]==3) {
            o->paired_rows++;
            if(y>=last-(last-first)/3) near_pairs++;
        } else o->inferred_rows++;
        if(near_center<0) near_center=best_center;
        previous_y=below_y; previous_center=below_center;
        below_y=y; below_center=best_center; below_left=best_start; below_right=best_end;
        /* Give confirmed single-edge rows useful weight instead of erasing
           the very bend we are trying to steer into. Keep the original ROI. */
        float weight=(float)(y-first+step);
        weighted+=best_center*weight; weights+=weight;
    }
    if(weights>0) o->error=weighted/weights-(CAR_W-1)*0.5f;
    o->valid=(uint8_t)(o->scanned_rows>=6 && near_pairs>=2 &&
                      o->paired_rows>=2 && o->valid_rows*3>=o->scanned_rows*2);
    if(o->valid) {
        s->previous_center=(int16_t)near_center;
        s->history_valid=1; s->polarity=c->white_road;
        for(int y=last;y>=first;y-=step) if(o->edge_flags[y]==3) {
            s->width[y]=(uint16_t)(o->right[y]-o->left[y]);
            s->width_age[y]=0;
        }
    } else {
        /* Invalid frames cannot train widths or constrain reacquisition. */
        s->history_valid=0;
        memset(s->width_age,255,sizeof s->width_age);
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
    zero_outputs(s); s->mode=CAR_TEST; s->test_motor=motor; s->test_until=now+1000;
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
        s->duty[0]=s->test_motor==1?500.0f:0;
        s->duty[1]=s->test_motor==2?500.0f:0;
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
