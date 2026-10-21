#include "single_edge.h"
/* Reference from two CRC-verified fixed-pose straight snapshots, 180609/180619.
 * Horizontal width is an estimate in curves, not an observed missing edge.
 * Change camera pose/track width => recalibrate before enabling this model. */
static const uint8_t width_reference[SE_H] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,75,76,78,79,80,81,83,84,86,87,88,89,90,92,93,94,95,96,97,98,99,100,101,102,103,104,106,106,108,108,110,111,112,114,114,115,116,117,118,120,120,122,123,124,125,126,127,128,129,130,131,132,134,134,136,137,138,139,140,142,142,144,144,146,146,148,148,150,150,152,153,154,155,156,157,158,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
static uint8_t active, active_side, recovered;
static uint32_t since;
static int abs_i(int a) { return a < 0 ? -a : a; }
static uint8_t dual_valid(const uint8_t *b,const uint8_t *l,const uint8_t *m,const uint8_t *r,int y)
{
    return l[y]>0 && r[y]<SE_W-1 && r[y]>l[y]+10 && b[y*SE_W+m[y]]==255;
}
se_result_t single_edge_update(const uint8_t *b,const uint8_t *l,
    const uint8_t *m,const uint8_t *r,uint32_t tick,uint8_t allow_single)
{
    se_result_t out={0,SE_W/2,0,0,0};
    int y, side=0, prev_edge=-1, center=SE_W/2;
    if (!allow_single) { active=active_side=recovered=0; }
    out.degraded=active; out.since=since;
    /* A transient dual edge must not repeatedly reset a degraded episode. */
    if (active && (uint32_t)(tick-since)>=SE_TIMEOUT_TICKS) return out;
    if (dual_valid(b,l,m,r,SE_ROW)) {
        out.mode=1; out.center=m[SE_ROW]; out.model_matches=1;
        for(y=SE_ROW-2;y<=SE_ROW+2;++y) {
            if (!dual_valid(b,l,m,r,y) || !width_reference[y] ||
                abs_i((int)r[y]-l[y]-width_reference[y])>8 ||
                (y>SE_ROW-2 && (abs_i((int)l[y]-l[y-1])>4 || abs_i((int)r[y]-r[y-1])>4)))
                out.model_matches=0;
        }
        if(active) {
            if(out.model_matches) ++recovered; else recovered=0;
            if(recovered>=3) { active=active_side=recovered=0; }
        }
        out.degraded=active;out.since=since;return out;
    }
    recovered=0;
    if (!allow_single) return out;
    if(l[SE_ROW]>0 && r[SE_ROW]==SE_W-1) side=2;
    else if(l[SE_ROW]==0 && r[SE_ROW]<SE_W-1) side=3;
    else return out;
    if(active && active_side!=side) return out;
    /* Require a continuous measured edge and a white path toward every
     * candidate center in a five-row neighborhood. Never clamp offscreen
     * estimates into view and then call them valid. */
    for(y=SE_ROW-2;y<=SE_ROW+2;++y) {
        int edge, x, first, last, white=0, count;
        if(!width_reference[y]) return out;
        if(side==2) {
            if(l[y]<3 || l[y]>SE_W-4 || r[y]!=SE_W-1) return out;
            edge=l[y];center=edge+width_reference[y]/2;
            first=edge+3;last=center;
        } else {
            if(l[y]!=0 || r[y]<3 || r[y]>SE_W-4) return out;
            edge=r[y];center=edge-width_reference[y]/2;
            first=center;last=edge-3;
        }
        if(prev_edge>=0 && abs_i(edge-prev_edge)>4) return out;
        prev_edge=edge;
        if(center<1 || center>=SE_W-1 || b[y*SE_W+center]!=255 || first>last) return out;
        count=last-first+1;
        for(x=first;x<=last;++x) if(b[y*SE_W+x]==255) ++white;
        if(white*10<count*9) return out;
        if(y==SE_ROW) out.center=(uint8_t)center;
    }
    if(!active) {active=1;active_side=(uint8_t)side;since=tick;}
    out.mode=(uint8_t)side;out.degraded=1;out.since=since;return out;
}
