#include "cross_straight.h"
#include <assert.h>
#include <stdio.h>

static uint8_t raw[188*120], threshold[120];

static void road(int opening_top, int opening_bottom, int shift)
{
    memset(raw,20,sizeof raw);
    memset(threshold,100,sizeof threshold);
    for(int y=0;y<=105;++y) {
        int half=25+y/3;
        int l=94+shift-half, r=94+shift+half;
        if(y>=opening_top && y<=opening_bottom) {l=0;r=187;}
        for(int x=l;x<=r;++x) if(x>=0 && x<188)raw[y*188+x]=220;
    }
}

int main(void)
{
    cross_straight_t s={0};
    road(120,120,0);
    cs_update(&s,raw,threshold,0,1);
    assert(!s.active);
    road(40,68,0);
    cs_update(&s,raw,threshold,20,1);
    assert(s.active && s.valid && s.opening && s.source==1);
    for(int y=65;y<=105;++y)assert(abs(s.center[y]-94)<=1);
    /* Only the visible departure stem survives. */
    road(65,105,0);
    cs_update(&s,raw,threshold,40,1);
    assert(s.active && s.valid && s.source==2);
    /* No boundary survives, but the just-observed line has current white clearance. */
    memset(raw,220,sizeof raw);
    for(int i=0;i<3;++i) {
        cs_update(&s,raw,threshold,60+20*i,1);
        assert(s.active && s.valid && s.source==3);
    }
    cs_update(&s,raw,threshold,120,1);
    assert(s.active && !s.valid);
    /* A new measured stem reacquires the same straight-through route. */
    road(65,105,0);
    cs_update(&s,raw,threshold,140,1);
    assert(s.active && s.valid);
    memset(raw+90*188,20,188);
    cs_update(&s,raw,threshold,160,1);
    assert(s.active && !s.valid);
    road(120,120,0);
    for(int i=0;i<2;++i) {
        cs_update(&s,raw,threshold,180+20*i,1);
        assert(s.active && s.valid && !s.recovered);
    }
    cs_update(&s,raw,threshold,220,1);
    assert(!s.active && s.valid && s.recovered);
    memset(&s,0,sizeof s);
    road(40,68,0);cs_update(&s,raw,threshold,0xfffffff0u,1);
    memset(raw,220,sizeof raw);
    cs_update(&s,raw,threshold,0xfffffff0u+CS_HOLD_TICKS+1u,1);
    assert(s.active && !s.valid);
    memset(&s,0,sizeof s);
    memset(raw,220,sizeof raw);cs_update(&s,raw,threshold,0,1);
    assert(!s.active && !s.valid);
    memset(raw,20,sizeof raw);cs_update(&s,raw,threshold,0,1);
    assert(!s.active && !s.valid);
    cross_straight_t a={0}, b={0};
    road(40,68,10);cs_update(&a,raw,threshold,0,1);
    for(int y=0;y<120;++y)for(int x=0;x<94;++x) {
        uint8_t value=raw[y*188+x];raw[y*188+x]=raw[y*188+187-x];raw[y*188+187-x]=value;
    }
    cs_update(&b,raw,threshold,0,1);
    assert(a.active && a.valid && b.active && b.valid);
    for(int y=65;y<=105;++y)assert(abs(a.center[y]+b.center[y]-187)<=1);
    cs_update(&a,raw,threshold,0,0);assert(!a.active && !a.valid);
    puts("PASS straight crossing: approach/departure fit, white clearance, bounded hold, blocked road, recovery, mirror and reset");
    return 0;
}
