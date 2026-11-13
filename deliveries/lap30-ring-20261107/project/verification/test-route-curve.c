#include "../demo/user/src/img_process.c"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);return 1;}}while(0)
static uint8_t raw[188*120],thr[120];
static entry_path_t p,q;
int main(void){
 memset(raw,220,sizeof raw);memset(thr,100,sizeof thr);memset(&ep_context,0,sizeof ep_context);
 /* Identical measured curve prefixes, differing far tails/densities. */
 p.count=3;p.points[0]=(entry_point_t){.x=94,.y=105};p.points[1]=(entry_point_t){.x=94,.y=70};p.points[2]=(entry_point_t){.x=154,.y=70};
 q=p;q.count=5;q.points[1]=(entry_point_t){.x=94,.y=85};q.points[2]=(entry_point_t){.x=94,.y=70};q.points[3]=(entry_point_t){.x=154,.y=70};q.points[4]=(entry_point_t){.x=180,.y=70};
 int pv,qv;CHECK(ip_path_controls(&p,&pv)&&pv);CHECK(ip_path_controls(&q,&qv)&&qv);CHECK(p.error==q.error && p.preview==q.preview && p.preview>0);
 q.points[0].y=100;CHECK(!ip_path_controls(&q,&qv));
 q=p;q.count=2;CHECK(ip_path_controls(&q,&qv)&&!qv);
 /* Bottom anchor is part of the track ROI, not a forbidden black obstacle. */
 memset(&p,0,sizeof p);p.count=2;p.points[0]=(entry_point_t){.x=94,.y=105};p.points[1]=(entry_point_t){.x=182,.y=70};p.estimated_width=48;p.boundary_source=2;
 CHECK(ep_fit_curve(raw,thr,-1,0,&p));CHECK(ip_path_controls(&p,&pv)&&pv&&p.preview>0);
 /* A white image alone is no lateral-road evidence. */
 int paired;CHECK(!ip_turn_curve(raw,thr,-1,1,&p,&paired));
 /* A 32px-wide detached dark mark in the same white space is not an exit. */
 memset(raw,220,sizeof raw);
 for(int y=28;y<=48;++y)for(int x=150;x<188;++x)raw[y*188+x]=20;
 CHECK(!ip_turn_curve(raw,thr,-1,1,&p,&paired));
 for(int side=-1;side<=1;side+=2) {
  memset(raw,220,sizeof raw);
  for(int y=42;y<=47;++y)for(int x=108;x<=186;++x)raw[y*188+(side<0?x:187-x)]=20;
  CHECK(!ip_turn_curve(raw,thr,side,1,&p,&paired));
 }
 /* Mirrored continuous outer bank, interrupted bank, ambiguous openings
  * and a black barrier across every connection to the selected side. */
 for(int side=-1;side<=1;side+=2) {
  memset(raw,20,sizeof raw);
  for(int y=50;y<=105;++y)memset(raw+y*188,220,188);
  CHECK(ip_turn_curve(raw,thr,side,1,&p,&paired) && !paired);
  CHECK(p.preview*(side<0?1:-1)>0 && p.inferred);
  for(int y=18;y<=105;++y)for(int x=166;x<=169;++x)raw[y*188+(side<0?x:187-x)]=220;
  CHECK(!ip_turn_curve(raw,thr,side,1,&p,&paired));
  memset(raw,20,sizeof raw);
  for(int y=50;y<=105;++y)memset(raw+y*188,220,188);
  for(int y=18;y<=105;++y)for(int x=126;x<=129;++x)raw[y*188+(side<0?x:187-x)]=20;
  CHECK(!ip_turn_curve(raw,thr,side,1,&p,&paired));
  memset(raw,20,sizeof raw);
  for(int y=25;y<=48;++y)memset(raw+y*188,220,188);
  for(int y=60;y<=85;++y)memset(raw+y*188,220,188);
  CHECK(!ip_turn_curve(raw,thr,side,1,&p,&paired));
 }
 /* A measured forward road can include a remote branch bending back.
  * ORBIT must retain a usable curve turning only toward its latched side. */
 for(int side=-1;side<=1;side+=2) {
  static uint8_t halves[120];memset(halves,20,sizeof halves);
  memset(raw,20,sizeof raw);memset(&ep_context,0,sizeof ep_context);
  ep_context.active=1;ep_context.ring_phase=2;ep_context.half_width=halves;
  for(int y=18;y<=105;++y) {
   float t=(105-y)/78.0f;
   int center=(int)(94+300*t*t-230*t*t*t);
   for(int x=center-20;x<=center+20;++x)if(x>=0&&x<188)
    raw[y*188+(side<0?x:187-x)]=220;
  }
  CHECK(ep_build_measured_corridor(raw,thr,side,94,&p));
  CHECK(ip_path_controls(&p,&pv)&&pv);
  for(int i=0;i<=128;++i) {
   float x,y,dx,dy,ddx,ddy;
   ep_curve_sample(p.curve,i/128.0f,&x,&y,&dx,&dy,&ddx,&ddy);
   CHECK((side<0?1:-1)*(dx*ddy-dy*ddx)>=-0.01f);
  }
 }
 memset(&ep_context,0,sizeof ep_context);
 /* The current island margin also constrains an ordinary entry candidate. */
 static uint8_t il[120],ir[120],mask[120];
 memset(&p,0,sizeof p);p.count=2;p.points[0]=(entry_point_t){.x=94,.y=105};p.points[1]=(entry_point_t){.x=94,.y=80};
 for(int y=80;y<=105;++y){mask[y]=1;il[y]=100;ir[y]=120;}
 ep_context.fresh_island=1;ep_context.island_left=il;ep_context.island_right=ir;ep_context.island_mask=mask;
 CHECK(!ip_curve_clearance(&p,-1));
 for(int y=80;y<=105;++y)il[y]=110;
 CHECK(ip_curve_clearance(&p,-1));
 memset(&ep_context,0,sizeof ep_context);
 /* An opposite-only narrow corridor cannot bypass the orbit constraint
  * by falling back to raw row targets or a partial preview. */
 {
  uint8_t a[120],b[120],c[120];
  img_process_reset();img_set_roundabout_guidance(-1);img_set_roundabout_entry(2);
  memset(raw,20,sizeof raw);
  for(int y=18;y<=105;++y) {
   int center=94-(105-y)*(105-y)/50;
   for(int x=center-12;x<=center+12;++x)if(x>=0&&x<188)raw[y*188+x]=220;
  }
  img_set_entry_odometry(0,0,1,0);auxiliary_process(raw,120,188,127,a,b,c);
  CHECK(!img_get_valid() && !img_get_preview_valid() && img_get_entry_path()->count==0);
 }
 /* A gap beyond the controller's freshness budget must not leave a stale
  * reference blocking a newly measured corridor forever. */
 static uint8_t l[120],m[120],r[120];
 img_process_reset();img_set_roundabout_guidance(-1);img_set_roundabout_entry(1);
 memset(raw,20,sizeof raw);
 for(int y=18;y<=105;++y)for(int x=24;x<=164;++x)raw[y*188+x]=220;
 s_entry_control_seen=1;s_entry_control_error=30;s_entry_control_preview=50;s_entry_weight_tick=s_entry_control_tick=1;
 img_set_entry_odometry(0,0,30,0);auxiliary_process(raw,120,188,127,l,m,r);
 CHECK(img_get_valid() && img_get_entry_path()->count>=12 && !img_get_orbit_ready());
 /* Failed 20 ms processing must not refresh the last VALID reference.
  * Reacquisition still needs two current images after a sustained loss. */
 img_process_reset();img_set_roundabout_guidance(-1);img_set_roundabout_entry(1);
 s_entry_control_seen=1;s_entry_control_error=30;s_entry_control_preview=50;
 s_entry_control_tick=s_entry_weight_tick=1;
 memset(raw,20,sizeof raw);
 for(int tick=3;tick<=25;tick+=2){img_set_entry_odometry(0,0,tick,0);auxiliary_process(raw,120,188,127,l,m,r);CHECK(!img_get_valid());}
 CHECK(s_entry_control_tick==1);
 for(int y=18;y<=105;++y)for(int x=24;x<=164;++x)raw[y*188+x]=220;
 img_set_entry_odometry(0,0,27,0);auxiliary_process(raw,120,188,127,l,m,r);CHECK(!img_get_valid());
 auxiliary_process(raw,120,188,127,l,m,r);CHECK(!img_get_valid());
 img_set_entry_odometry(0,0,29,0);auxiliary_process(raw,120,188,127,l,m,r);CHECK(img_get_valid() && !img_get_orbit_ready());
 puts("PASS fixed horizons, mirror, true bank, detached marks, break, barrier, ambiguity, island clearance, stale-reference expiry and two-frame reacquisition");return 0;
}
