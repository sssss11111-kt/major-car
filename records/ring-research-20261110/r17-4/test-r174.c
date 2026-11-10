#include <stdio.h>
#include <stdlib.h>
#include "img_process.c"

static int checks;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#condition); exit(1); } } while (0)
static uint8_t raw[188*120], thresholds[120], left[120], mid[120], right[120];

static void road(int shift) {
    memset(raw,20,sizeof raw);
    memset(thresholds,100,sizeof thresholds);
    for (int y=18;y<120;++y) {
        int center=94+shift*(105-y)/87, half=ip_half(y);
        for (int x=0;x<188;++x)
            if (x>=center-half && x<=center+half) raw[y*188+x]=220;
    }
}

int main(void) {
    /* Reuse the ordinary selector and its exact near/preview weighted errors. */
    for (int shift=-36;shift<=36;shift+=12) {
        img_process_reset(); s_scene_ok=1; s_ring_guide=-1;
        road(shift); ip_ring_bend_observe(raw,thresholds);
        CHECK(s_ring_bend.near_count==26);
        int expected=(int)(s_ring_bend.near_sum/s_ring_bend.near_weight)-94;
        if (expected>-3 && expected<3) expected=0;
        ip_ring_bend_apply(raw,thresholds,left,mid,right,0,0);
        CHECK(s_valid && s_error==expected && s_preview_valid);
        for (int y=80;y<=105;++y) CHECK(mid[y] && raw[y*188+mid[y]]>100);
    }

    /* The actual camera sees the cap between row 105 and the image bottom. */
    img_process_reset(); road(0); s_ring_guide=-1;
    memset(raw,220,sizeof raw);
    s_ring_miny=40; s_ring_maxy=105;
    for (int y=80;y<114;++y)
        for (int x=170;x<188;++x) raw[y*188+x]=20;
    CHECK(ip_ring_entrance_visible(raw,thresholds));
    for (int y=114;y<120;++y)
        for (int x=170;x<188;++x) raw[y*188+x]=20;
    CHECK(!ip_ring_entrance_visible(raw,thresholds));

    /* Three new image ticks, no duplicate-frame confirmation, loss resets it. */
    s_entry_mode=1; s_ring_entrance_seen=1; s_scene_ok=1;
    s_ring_bend.near_chain=26;s_ring_bend.near_bottom=105;
    s_odometry_tick=1;ip_ring_local_handoff(raw,thresholds);
    CHECK(s_ring_local_confirm==1 && !s_ring_local_active);
    ip_ring_local_handoff(raw,thresholds);
    CHECK(s_ring_local_confirm==1);
    s_scene_ok=0;s_odometry_tick=2;ip_ring_local_handoff(raw,thresholds);
    CHECK(!s_ring_local_confirm);
    s_scene_ok=1;
    for (int tick=3;tick<=5;++tick) {
        s_odometry_tick=(uint32_t)tick;ip_ring_local_handoff(raw,thresholds);
        CHECK(s_ring_local_active==(tick==5));
    }
    img_set_roundabout_entry(3);
    CHECK(s_ring_local_active);
    img_set_roundabout_entry(0);
    CHECK(!s_ring_local_active);

    /* End preview at the first missing bank, keep measured near steering. */
    img_process_reset(); road(24);s_scene_ok=1;s_ring_guide=-1;
    for (int y=65;y<=69;++y) memset(raw+y*188,220,188);
    ip_ring_bend_observe(raw,thresholds);
    CHECK(s_ring_bend.near_count==26);
    for (int y=18;y<65;++y) CHECK(!s_ring_bend.center[y]);
    ip_ring_bend_apply(raw,thresholds,left,mid,right,0,0);
    CHECK(s_valid && !s_preview_valid);

    /* Actual production EXIT retains the current right-forward ordinary bend. */
    img_process_reset();road(36);
    img_set_roundabout_guidance(-1);img_set_roundabout_entry(1);
    s_ring_local_active=1;
    img_set_roundabout_entry(3);
    img_set_entry_odometry(0,0,10,0);
    auxiliary_process(raw,120,188,127,left,mid,right);
    CHECK(img_get_ring_local_active() && img_get_valid());
    CHECK(!img_get_entry_path()->count && img_get_preview_error()>0);
    CHECK(img_get_exit_error()==img_get_preview_error());
    memset(raw,20,sizeof raw);img_set_entry_odometry(0,0,11,0);
    auxiliary_process(raw,120,188,127,left,mid,right);
    CHECK(!img_get_valid());
    img_process_reset();CHECK(!img_get_ring_local_active());
    printf("PASS %d R17-4 ordinary bend, full-image cap, duplicate tick, preview branch and EXIT checks\n",checks);
    return 0;
}
