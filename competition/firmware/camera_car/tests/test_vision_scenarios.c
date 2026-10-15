/* Deterministic synthetic fixtures. These do not model the actual lens/track. */
#include "car_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
enum { STRAIGHT, CURVE_LEFT, CURVE_RIGHT, S_BEND, SHADOW_MILD, SHADOW_STRONG,
       GLARE_OUTSIDE, GLARE_EDGE, GAP_LEFT, GAP_BOTH, MISSING_LEFT, MISSING_RIGHT,
       WHITE_FRAME, BLACK_FRAME, LOW_CONTRAST, HARD_CORNER, CLUSTER_NOISE };
enum { ACCURATE, REJECT, CHARACTERIZE };
typedef struct {
    const char *name;
    int shape, shift, dark, light, jitter, salt, expected;
} scenario_t;
static const scenario_t scenarios[]={
    {"straight",STRAIGHT,0,30,210,0,0,ACCURATE},
    {"straight_left",STRAIGHT,-24,30,210,0,0,ACCURATE},
    {"straight_right",STRAIGHT,24,30,210,0,0,ACCURATE},
    {"dim_light",STRAIGHT,0,8,65,0,0,ACCURATE},
    {"bright_light",STRAIGHT,0,120,240,0,0,ACCURATE},
    {"sharp_left_curve",CURVE_LEFT,0,30,210,0,0,ACCURATE},
    {"sharp_right_curve",CURVE_RIGHT,0,30,210,0,0,ACCURATE},
    {"s_bend",S_BEND,0,30,210,0,0,ACCURATE},
    {"horizontal_shadow",SHADOW_MILD,0,30,210,0,0,ACCURATE},
    {"severe_shadow",SHADOW_STRONG,0,30,210,0,0,REJECT},
    {"background_reflection",GLARE_OUTSIDE,0,30,210,0,0,ACCURATE},
    {"reflection_merges_right_edge",GLARE_EDGE,0,30,210,0,0,CHARACTERIZE},
    {"short_left_boundary_gap",GAP_LEFT,0,30,210,0,0,ACCURATE},
    {"short_both_boundary_gaps",GAP_BOTH,0,30,210,0,0,ACCURATE},
    {"missing_left_boundary",MISSING_LEFT,0,30,210,0,0,REJECT},
    {"missing_right_boundary",MISSING_RIGHT,0,30,210,0,0,REJECT},
    {"all_white",WHITE_FRAME,0,30,210,0,0,REJECT},
    {"all_black",BLACK_FRAME,0,30,210,0,0,REJECT},
    {"low_contrast",LOW_CONTRAST,0,110,118,0,0,REJECT},
    {"abrupt_corner_out_of_search",HARD_CORNER,0,30,210,0,0,REJECT},
    {"gray_noise",STRAIGHT,0,30,210,12,0,ACCURATE},
    {"isolated_salt_pepper",STRAIGHT,0,30,210,0,1,ACCURATE},
    {"curve_gray_noise",CURVE_RIGHT,0,30,210,12,0,ACCURATE},
    {"clustered_dark_noise",CLUSTER_NOISE,0,30,210,0,0,CHARACTERIZE}
};
static uint8_t image[CAR_W*CAR_H],binary[CAR_W*CAR_H];
static int truth_left[CAR_H],truth_right[CAR_H];
static unsigned checks,failures,accurate_count,reject_count,characterize_count;
static uint32_t random_state;
static uint32_t next_random(void) {
    random_state=random_state*1664525u+1013904223u; return random_state;
}
static int clip(int x) { return x<0?0:(x>255?255:x); }
static void check(int ok,const char *name,const char *condition) {
    checks++;
    if(!ok) { failures++; fprintf(stderr,"FAIL %s: %s\n",name,condition); }
}
static void render(const scenario_t *s,uint32_t seed) {
    random_state=seed;
    for(int y=0;y<CAR_H;y++) {
        int center=94+s->shift,half=44;
        if(s->shape==CURVE_LEFT || s->shape==CURVE_RIGHT) {
            int d=112-y; int bend=d*d/130;
            center+=s->shape==CURVE_LEFT?-bend:bend;
            half=15+y/3;
        }
        if(s->shape==S_BEND) {
            int d=112-y;
            center+=d<24?d:48-d;
            half=25+y/5;
        }
        if(s->shape==HARD_CORNER && y<=94) { center=160; half=12; }
        truth_left[y]=center-half; truth_right[y]=center+half;
        for(int x=0;x<CAR_W;x++) {
            int in_road=x>=truth_left[y] && x<=truth_right[y];
            int pixel=in_road?s->light:s->dark;
            if(s->shape==SHADOW_MILD && y>=60 && y<=82) pixel=in_road?90:15;
            if(s->shape==SHADOW_STRONG && y<=100) pixel=in_road?45:15;
            if(s->shape==GLARE_OUTSIDE && x<30 && y>=40 && y<=100) pixel=255;
            if(s->shape==GLARE_EDGE && x>=138 && x<=176 && y<=100) pixel=255;
            if(s->shape==MISSING_LEFT && x<50) pixel=s->light;
            if(s->shape==MISSING_RIGHT && x>138) pixel=s->light;
            if(y>=72 && y<=78) {
                if(s->shape==GAP_LEFT && x<50) pixel=s->light;
                if(s->shape==GAP_BOTH) pixel=s->light;
            }
            if(s->shape==WHITE_FRAME) pixel=255;
            if(s->shape==BLACK_FRAME) pixel=0;
            if(s->shape==CLUSTER_NOISE && (x==80 || x==81)) pixel=0;
            if(s->jitter) pixel+=((int)(next_random()%(uint32_t)(2*s->jitter+1))-s->jitter);
            image[y*CAR_W+x]=(uint8_t)clip(pixel);
        }
        if(s->salt) {
            /* Separate pepper pixels cannot form a two-pixel edge; avoid boundaries. */
            for(int band=0;band<3;band++) {
                int x=64+band*24+(int)(next_random()%8);
                image[y*CAR_W+x]=0;
            }
            image[y*CAR_W+16]=255; image[y*CAR_W+170]=255;
        }
    }
}
static void pgm(const char *directory,const char *name,const char *suffix,const uint8_t *data) {
    char path[1024];
    snprintf(path,sizeof path,"%s/%s-%s.pgm",directory,name,suffix);
    FILE *f=fopen(path,"wb");
    if(!f) { perror(path); exit(2); }
    fprintf(f,"P5\n%d %d\n255\n",CAR_W,CAR_H);
    if(fwrite(data,1,CAR_W*CAR_H,f)!=CAR_W*CAR_H) { fclose(f); exit(2); }
    fclose(f);
}
static void run_case(const scenario_t *s,uint32_t seed,FILE *report,const char *directory) {
    vision_state_t state;
    vision_config_t cfg;
    vision_result_t result;
    unsigned before=failures;
    float max_center_error=0;
    vision_config_default(&cfg); vision_reset(&state);
    /* Controlled fixture ROI; do not tie geometry tests to future default changes. */
    cfg.first_row=40; cfg.last_row=112; cfg.row_step=6;
    render(s,seed);
    vision_process(&state,&cfg,image,binary,&result);
    check(isfinite(result.error) && fabsf(result.error)<=94,s->name,"finite bounded error");
    check(result.paired_rows<=result.valid_rows && result.valid_rows<=result.scanned_rows,
          s->name,"paired <= valid <= scanned");
    check(state.previous_center>=0 && state.previous_center<CAR_W,s->name,"history stays in bounds");
    for(int y=0;y<CAR_H;y++) {
        int flags=result.edge_flags[y];
        check(result.center[y]==-1 || (result.center[y]>=0 && result.center[y]<CAR_W),s->name,"center coordinate");
        check(flags<=3,s->name,"edge bit flags");
        if(flags&1) check(result.left[y]>=0 && result.left[y]<CAR_W,s->name,"left coordinate");
        if(flags&2) check(result.right[y]>=0 && result.right[y]<CAR_W,s->name,"right coordinate");
        if(flags==3) {
            check(result.left[y]<=result.center[y] && result.center[y]<=result.right[y],s->name,"middle between edges");
            float err=fabsf(result.center[y]-(truth_left[y]+truth_right[y])/2.0f);
            if(err>max_center_error) max_center_error=err;
            if(s->expected==ACCURATE) {
                check(abs(result.left[y]-truth_left[y])<=1,s->name,"left matches fixture geometry");
                check(abs(result.right[y]-truth_right[y])<=1,s->name,"right matches fixture geometry");
                check(err<=1,s->name,"center matches fixture geometry");
            }
        }
    }
    for(int i=0;i<CAR_W*CAR_H;i++) {
        if(binary[i]!=0 && binary[i]!=255) { check(0,s->name,"binary values 0/255"); break; }
    }
    if(s->expected==ACCURATE) {
        accurate_count++;
        check(result.valid,s->name,"usable track detected");
        check(result.paired_rows>=8,s->name,"enough paired rows");
        if(s->shape==STRAIGHT || s->shape==SHADOW_MILD || s->shape==GLARE_OUTSIDE ||
           s->shape==GAP_LEFT || s->shape==GAP_BOTH)
            check(fabsf(result.error-(s->shift+0.5f))<=1,s->name,"straight offset within one pixel");
        if(s->shape==CURVE_LEFT) check(result.error< -3,s->name,"left curve direction");
        if(s->shape==CURVE_RIGHT) check(result.error>3,s->name,"right curve direction");
    } else if(s->expected==REJECT) {
        reject_count++; check(!result.valid,s->name,"ambiguous/no track rejected");
    } else characterize_count++;
    if(report) {
        fprintf(report,"{\"name\":\"%s\",\"expectation\":\"%s\",\"valid\":%u,\"threshold\":%u,\"error\":%.3f,\"paired\":%u,\"scanned\":%u,\"max_center_error\":%.3f,\"failures\":%u,\"rows\":[",
            s->name,s->expected==ACCURATE?"accurate":(s->expected==REJECT?"reject":"characterize"),
            result.valid,result.threshold,(double)result.error,result.paired_rows,result.scanned_rows,
            (double)max_center_error,failures-before);
        for(int y=0;y<CAR_H;y++) {
            if(y) fputc(',',report);
            fprintf(report,"[%d,%d,%d,%d,%d]",result.left[y],result.right[y],result.center[y],truth_left[y],truth_right[y]);
        }
        fprintf(report,"]}\n");
        pgm(directory,s->name,"gray",image); pgm(directory,s->name,"binary",binary);
        printf("%-34s %s valid=%u error=%7.2f paired=%u/%u\n",s->name,
            failures>before?"FAIL":(s->expected==CHARACTERIZE?"OBSERVE":"PASS"),
            result.valid,(double)result.error,result.paired_rows,result.scanned_rows);
    }
}
static void sequence_checks(void) {
    vision_state_t state;
    vision_config_t cfg;
    vision_result_t result;
    const int shifts[]={0,8,16,24,16,8,0,-8,-16,-24,-16,-8,0};
    unsigned before=failures;
    scenario_t fixture=scenarios[0];
    vision_reset(&state); vision_config_default(&cfg);
    cfg.first_row=40; cfg.last_row=112; cfg.row_step=6;
    for(unsigned i=0;i<sizeof shifts/sizeof shifts[0];i++) {
        fixture.shift=shifts[i]; render(&fixture,12345);
        vision_process(&state,&cfg,image,binary,&result);
        check(result.valid && fabsf(result.error-(shifts[i]+0.5f))<=1,
              "moving_track_sequence","follow gradual left/right movement without reset");
    }
    for(int frame=0;frame<3;frame++) {
        memset(image,255,sizeof image); vision_process(&state,&cfg,image,binary,&result);
        check(!result.valid && result.valid_rows==0,"lost_track_sequence","no ghost validity from history");
        for(int y=0;y<CAR_H;y++)
            check(result.center[y]==-1 && result.edge_flags[y]==0,
                  "lost_track_sequence","old boundary coordinates cleared");
    }
    fixture.shift=0; render(&fixture,12345);
    vision_process(&state,&cfg,image,binary,&result);
    check(result.valid && fabsf(result.error)<1,"recovery_sequence","recover near previous center");
    printf("Temporal sequences: %s (13 moving + 3 lost + 1 recovery frames)\n",failures==before?"PASS":"FAIL");
}
int main(int argc,char **argv) {
    FILE *report=NULL;
    if(argc>1) {
        char path[1024]; snprintf(path,sizeof path,"%s/results.jsonl",argv[1]);
        report=fopen(path,"w"); if(!report) { perror(path);return 2; }
    }
    for(unsigned i=0;i<sizeof scenarios/sizeof scenarios[0];i++)
        run_case(&scenarios[i],12345u,report,argc>1?argv[1]:NULL);
    if(report) fclose(report);
    /* Same geometries, independent deterministic jitter seeds: 128 repeatable cases. */
    for(unsigned seed=1;seed<=64;seed++) {
        run_case(&scenarios[20],seed,NULL,NULL); run_case(&scenarios[22],seed,NULL,NULL);
    }
    sequence_checks();
    printf("Scenario suite: %u accuracy contracts, %u rejection contracts, %u characterization cases; %u checks, %u failures\n",
        accurate_count,reject_count,characterize_count,checks,failures);
    return failures?1:0;
}
