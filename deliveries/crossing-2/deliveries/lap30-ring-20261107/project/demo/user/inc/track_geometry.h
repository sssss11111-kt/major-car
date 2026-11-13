#ifndef TRACK_GEOMETRY_H
#define TRACK_GEOMETRY_H
#include <math.h>
/* R17 physical geometry is opt-in. H maps image (column,row,1) to rear-axle
 * coordinates (forward metres, left metres,1). No guessed camera calibration.
 * Supply all TRACK_H0..8, TRACK_MAX_CURVATURE (1/m), TRACK_STEER_PER_RAD
 * (positive command units/rad for left turn), TRACK_LOOKAHEAD_M after measuring.
 * Calibration must also validate footprint clearance separately. */
#ifndef TRACK_GEOMETRY_CALIBRATED
#define TRACK_GEOMETRY_CALIBRATED 0
#endif
#if TRACK_GEOMETRY_CALIBRATED
#if !defined(TRACK_H0) || !defined(TRACK_H1) || !defined(TRACK_H2) || !defined(TRACK_H3) || !defined(TRACK_H4) || !defined(TRACK_H5) || !defined(TRACK_H6) || !defined(TRACK_H7) || !defined(TRACK_H8) || !defined(TRACK_MAX_CURVATURE) || !defined(TRACK_STEER_PER_RAD) || !defined(TRACK_LOOKAHEAD_M)
#error Complete measured track geometry and steering calibration is required
#endif
static inline int track_project(float x,float y,float *forward,float *left) {
 float w=TRACK_H6*x+TRACK_H7*y+TRACK_H8;
 if(!isfinite(w) || fabsf(w)<0.00001f)return 0;
 *forward=(TRACK_H0*x+TRACK_H1*y+TRACK_H2)/w;
 *left=(TRACK_H3*x+TRACK_H4*y+TRACK_H5)/w;
 return isfinite(*forward) && isfinite(*left) && *forward>=0;
}
/* Signed circumcircle curvature of samples on the continuous cubic, not rounded
 * display pixels. A duplicate point or invalid projection is not feasible. */
static inline int track_curvature(float ax,float ay,float bx,float by,float cx,float cy,float *k) {
 float ux=bx-ax,uy=by-ay,vx=cx-bx,vy=cy-by;
 float length=sqrtf((ux*ux+uy*uy)*(vx*vx+vy*vy)*((cx-ax)*(cx-ax)+(cy-ay)*(cy-ay)));
 if(!isfinite(length)||length<1e-9f)return 0;
 *k=2*(ux*vy-uy*vx)/length;
 return isfinite(*k) && fabsf(*k)<=TRACK_MAX_CURVATURE;
}
#endif
#endif
