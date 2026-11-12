/* White-corridor/geometry contracts using the actual vision implementation. */
#include "../demo/user/src/img_process.c"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);return 1;}}while(0)
static uint8_t raw[188*120],thr[120],ordinary[120],l[120],m[120],r[120];
static entry_path_t entry,out;
int main(void) {
    memset(thr,100,sizeof thr);
    for(int side=-1;side<=1;side+=2) {
        memset(raw,20,sizeof raw);memset(ordinary,0,sizeof ordinary);
        for(int y=18;y<=105;++y){ordinary[y]=(uint8_t)(94+side*(105-y)/6);
            for(int x=20;x<168;++x)raw[y*188+x]=220;}
        float control[8]={94,105,94,80,94+side*25,65,94+side*45,50};
        memset(&entry,0,sizeof entry);
        CHECK(ep_sample_curve(raw,thr,control,&entry));memcpy(entry.curve,control,sizeof control);
        int pv=0;CHECK(ip_path_controls(&entry,&pv) && pv);
        int previous_near=0,previous_preview=0;
        for(int k=0;k<=5;++k){
            CHECK(ip_entry_transition(raw,thr,ordinary,k/5.0f,&entry,&out));
            for(int i=1;i<out.count;++i)
                CHECK(ep_segment_white(raw,thr,out.points[i-1].x,out.points[i-1].y,out.points[i].x,out.points[i].y));
            if(k){CHECK(abs(out.error-previous_near)<=4);CHECK(abs(out.preview-previous_preview)<=12);}
            previous_near=out.error;previous_preview=out.preview;
        }
        /* A dark transverse obstacle forbids the transition, however smooth. */
        for(int y=76;y<=80;++y)memset(raw+y*188,20,188);
        CHECK(!ip_entry_transition(raw,thr,ordinary,0.5f,&entry,&out));
    }
    /* No row evidence may be invented from a horizontal/reversed path suffix. */
    memset(&entry,0,sizeof entry);entry.count=3;
    entry.points[0]=(entry_point_t){.x=94,.y=100};
    entry.points[1]=(entry_point_t){.x=95,.y=95};
    entry.points[2]=(entry_point_t){.x=110,.y=98};
    float x;CHECK(!ip_path_row(&entry,90,&x));int pv;
    CHECK(!ip_path_controls(&entry,&pv));
    img_process_reset();img_set_roundabout_guidance(-1);img_set_roundabout_entry(1);
    memset(raw,20,sizeof raw);
    for(int y=18;y<=105;++y)for(int x=24;x<=164;++x)raw[y*188+x]=220;
    img_set_entry_odometry(0,0,1,0);auxiliary_process(raw,120,188,127,l,m,r);
    CHECK(img_get_valid() && s_entry_weight>0);
    float weight=s_entry_weight;
    for(int n=0;n<5;++n){auxiliary_process(raw,120,188,127,l,m,r);CHECK(s_entry_weight==weight);}
    /* Repeated current timestamp cannot advance blend progress. Failure must
     * clear visual-inside evidence and cannot authorize a zero-weight escape. */
    img_process_reset();img_set_roundabout_guidance(-1);img_set_roundabout_entry(1);
    for(int n=1;n<=4;++n){
        memset(raw,20,sizeof raw);
        s_entry_inside=2;s_entry_weight=0;
        img_set_entry_odometry(0,0,n,0);
        auxiliary_process(raw,120,188,127,l,m,r);
        CHECK(!img_get_orbit_ready() && !img_get_valid() && s_entry_inside==0);
    }
    /* A straight current corridor may remain drivable during entry, but
     * software blend completion cannot establish clockwise circulation. */
    img_process_reset();img_set_roundabout_guidance(-1);img_set_roundabout_entry(1);
    memset(raw,20,sizeof raw);
    for(int y=18;y<=105;++y)for(int x=24;x<=164;++x)raw[y*188+x]=220;
    for(int n=1;n<=12;++n) {
        img_set_entry_odometry(0,0,n,0);auxiliary_process(raw,120,188,127,l,m,r);
        CHECK(img_get_valid() && !img_get_orbit_ready());
    }
    /* A fresh but incompatible control reference cannot force a sudden
     * handoff merely because a different current white corridor exists. */
    s_entry_control_seen=1;s_entry_control_error=20;s_entry_control_preview=40;
    img_set_entry_odometry(0,0,13,0);auxiliary_process(raw,120,188,127,l,m,r);
    CHECK(!img_get_valid() && !img_get_orbit_ready());
    puts("PASS mirrored current blends, obstacle rejection, partial horizon, duplicate tick, loss, straight-road non-orbit and fresh handoff rejection");
    return 0;
}
