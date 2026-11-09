/* Analytic test calibration only, never the car's calibration. */
#define TRACK_GEOMETRY_CALIBRATED 1
#define TRACK_H0 0.0f
#define TRACK_H1 -0.01f
#define TRACK_H2 1.15f
#define TRACK_H3 -0.01f
#define TRACK_H4 0.0f
#define TRACK_H5 0.94f
#define TRACK_H6 0.0f
#define TRACK_H7 0.0f
#define TRACK_H8 1.0f
#define TRACK_MAX_CURVATURE 4.0f
#define TRACK_STEER_PER_RAD 30.0f
#define TRACK_LOOKAHEAD_M 0.4f
#include "ring_entry_path.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
 float x,y,k,e;
 assert(track_project(94,105,&x,&y));assert(fabsf(x-.1f)<1e-5f && fabsf(y)<1e-5f);
 assert(!track_project(NAN,80,&x,&y));
 assert(!track_project(94,120,&x,&y));
 assert(track_curvature(0,0,.1f,0,.2f,0,&k) && fabsf(k)<1e-5f);
 /* Three exact samples on radius 0.5m and mirrored circle. */
 for(int sign=-1;sign<=1;sign+=2) {
  assert(track_curvature(0,0,.5f*sinf(.2f),sign*.5f*(1-cosf(.2f)),
     .5f*sinf(.4f),sign*.5f*(1-cosf(.4f)),&k));
  assert(fabsf(k-sign*2)<.001f);
 }
 assert(!track_curvature(0,0,.1f,.1f,0,.2f,&k)); /* 0.1m radius */
 assert(!track_curvature(0,0,0,0,.2f,0,&k));
 entry_path_t p={0};p.count=3;
 p.points[0].x=94;p.points[0].y=105;
 p.points[1].x=94;p.points[1].y=90;
 p.points[2].x=94;p.points[2].y=70;
 assert(ep_metric_error(&p,&e) && fabsf(e)<1e-5f);
 p.points[2].x=84;assert(ep_metric_error(&p,&e) && e<0);
 float left=e;p.points[2].x=104;assert(ep_metric_error(&p,&e) && fabsf(e+left)<1e-4f);
 p.count=0;assert(!ep_metric_error(&p,&e));
 puts("PASS: metric projection, straight/arc curvature, mirror, infeasible radius, invalid data, pure-pursuit sign");
 return 0;
}
