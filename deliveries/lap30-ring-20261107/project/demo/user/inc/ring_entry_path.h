#ifndef RING_ENTRY_PATH_H
#define RING_ENTRY_PATH_H
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "track_geometry.h"

/* Two-dimensional entrance path. Points are ordered from the car to the
 * selected side opening. Left/right are relative to that local direction. */
#define EP_WIDTH 188
#define EP_TOP 18
#define EP_BOTTOM 105
#define EP_COLS 94
#define EP_ROWS 44
#define EP_CELLS (EP_COLS * EP_ROWS)
#define EP_MAX_POINTS 160
/* Image-space direction change between consecutive ordered segments. */
#define EP_MAX_SEGMENT_TURN_DEG 40.0f
#define EP_MIN_CURVE_POINT_DISTANCE 4
#define EP_WIDTH_MAX_TICKS 100U
#define EP_WIDTH_MAX_TRAVEL_M 0.20f

typedef struct {
    const uint8_t *island_left, *island_right, *island_mask, *half_width;
    uint32_t tick;
    float distance_m;
    uint8_t active, fresh_island, odometry_valid;
    uint8_t exit_mode;
    int16_t exit_reference;
} entry_context_t;

typedef struct {
    uint8_t x, y, left_x, left_y, right_x, right_y, measured;
} entry_point_t;
typedef struct {
    entry_point_t points[EP_MAX_POINTS];
    uint16_t count;
    uint8_t goal_x, goal_y, inferred, failure, boundary_source, support;
    int16_t error, preview;
    float curve_t[EP_MAX_POINTS]; /* Parameter of each accepted curve sample. */
    float estimated_width; /* Image-space width at the side opening. */
    float curve[8]; /* Cubic control points, used by the native controller path. */
} entry_path_t;

/* Calibrated pure pursuit produces a complete command-equivalent error.
 * Callers must disable pixel preview fusion when this succeeds. */
static inline int ep_metric_error(const entry_path_t *path,float *error) {
#if TRACK_GEOMETRY_CALIBRATED
    float x=0,y=0;
    if(path->count<2)return 0;
    for(int i=0;i<path->count;++i) {
        if(!track_project(path->points[i].x,path->points[i].y,&x,&y))return 0;
        if(x*x+y*y>=TRACK_LOOKAHEAD_M*TRACK_LOOKAHEAD_M)break;
    }
    float d=x*x+y*y;
    if(d<0.0001f)return 0;
    float curvature=2*y/d;
    if(fabsf(curvature)>TRACK_MAX_CURVATURE)return 0;
    /* The existing controller is u=middle-0.5*error. Keep its filtering and slew. */
    *error=-2.0f*TRACK_STEER_PER_RAD*atanf(0.200f*curvature);
    return isfinite(*error);
#else
    (void)path;(void)error;return 0;
#endif
}

#ifdef RING_ENTRY_IMPLEMENTATION
static uint8_t ep_clearance[EP_CELLS];
static uint32_t ep_cost[EP_CELLS];
static int16_t ep_parent[EP_CELLS], ep_position[EP_CELLS];
static uint16_t ep_heap[EP_CELLS];
static int ep_heap_count;
static int ep_previous_width, ep_previous_side, ep_width_age;
static float ep_width_reference_row, ep_previous_goal_y;
static int ep_previous_outer[3], ep_previous_inner[3];
static entry_context_t ep_context;
static uint32_t ep_width_tick;
static float ep_width_distance;
static uint8_t ep_width_odometry_valid;
static void ep_reset(void) {
    ep_previous_width = 0;
    ep_previous_side = 0;
    ep_width_age = 0;
    ep_width_reference_row = ep_previous_goal_y = 0;
    memset(ep_previous_outer,0,sizeof ep_previous_outer);
    memset(ep_previous_inner,0,sizeof ep_previous_inner);
}

static void ep_set_context(const entry_context_t *context) {
    ep_context = *context;
    if (ep_previous_width && ((uint32_t)(context->tick - ep_width_tick) > EP_WIDTH_MAX_TICKS ||
        (context->odometry_valid && ep_width_odometry_valid &&
         (context->distance_m < ep_width_distance ||
          context->distance_m - ep_width_distance > EP_WIDTH_MAX_TRAVEL_M)))) ep_reset();
}

static int ep_abs(int v) { return v < 0 ? -v : v; }
static int ep_white(const uint8_t *raw, const uint8_t *thresholds, int x, int y) {
    return x >= 0 && x < EP_WIDTH && y >= EP_TOP && y <= EP_BOTTOM &&
           raw[y * EP_WIDTH + x] > thresholds[y];
}

static int ep_turn_allowed(float ax, float ay, float bx, float by) {
    const float cosine = cosf(EP_MAX_SEGMENT_TURN_DEG * 0.01745329252f);
    float dot = ax * bx + ay * by;
    float length_product = (ax * ax + ay * ay) * (bx * bx + by * by);
    return length_product > 0 && dot > 0 &&
           dot * dot >= cosine * cosine * length_product;
}

static int ep_path_turn_allowed(const entry_path_t *path, int x, int y) {
    if (!path->count) return 1;
    const entry_point_t *last = &path->points[path->count - 1];
    if (last->x == x && last->y == y) return 0;
    if (path->count < 2) return 1;
    const entry_point_t *before = &path->points[path->count - 2];
    return ep_turn_allowed((float)last->x - before->x, (float)last->y - before->y,
                           (float)x - last->x, (float)y - last->y);
}

static int ep_segment_white(const uint8_t *raw, const uint8_t *thresholds,
                             int ax, int ay, int bx, int by) {
    int dx = bx - ax, dy = by - ay;
    int steps = ep_abs(dx) > ep_abs(dy) ? ep_abs(dx) : ep_abs(dy);
    if (!steps) return ep_white(raw, thresholds, ax, ay);
    for (int i = 0; i <= steps; ++i) {
        if (!ep_white(raw, thresholds, ax + dx * i / steps, ay + dy * i / steps)) return 0;
    }
    return 1;
}

/* Indexed binary heap bounds both work and memory to the image grid. */
static void ep_heap_update(int cell) {
    int pos = ep_position[cell];
    if (pos < 0) {
        pos = ep_heap_count++;
        ep_heap[pos] = (uint16_t)cell;
    }
    while (pos > 0) {
        int parent = (pos - 1) / 2;
        int other = ep_heap[parent];
        if (ep_cost[other] <= ep_cost[cell]) break;
        ep_heap[pos] = (uint16_t)other;
        ep_position[other] = (int16_t)pos;
        pos = parent;
    }
    ep_heap[pos] = (uint16_t)cell;
    ep_position[cell] = (int16_t)pos;
}
static int ep_heap_pop(void) {
    int result = ep_heap[0];
    int cell = ep_heap[--ep_heap_count];
    int pos = 0;
    ep_position[result] = -2;
    while (2 * pos + 1 < ep_heap_count) {
        int child = 2 * pos + 1;
        if (child + 1 < ep_heap_count && ep_cost[ep_heap[child + 1]] < ep_cost[ep_heap[child]]) ++child;
        if (ep_cost[cell] <= ep_cost[ep_heap[child]]) break;
        ep_heap[pos] = ep_heap[child];
        ep_position[ep_heap[pos]] = (int16_t)pos;
        pos = child;
    }
    if (ep_heap_count) {
        ep_heap[pos] = (uint16_t)cell;
        ep_position[cell] = (int16_t)pos;
    }
    return result;
}

static float ep_width_model(float row) {
    return IP_WIDTH_CAL_PX/2.0f + IP_HALF_PERSPECTIVE_SPAN*(row-IP_WIDTH_CAL_ROW)/(MT9V03X_H-1);
}
static int ep_estimated_width(int center) {
    float ratio = ep_width_model((float)center)/ep_width_model(ep_width_reference_row);
    if (ratio < 0.70f || ratio > 1.40f) return 0;
    return (int)(ep_previous_width*ratio+0.5f);
}
static int ep_edge_contrast(const uint8_t *raw,int x,int edge,int outer) {
    int white_row = outer ? edge+1 : edge-1;
    int dark_row = outer ? edge-2 : edge+2;
    if (white_row < EP_TOP || white_row > EP_BOTTOM || dark_row < EP_TOP || dark_row > EP_BOTTOM) return 0;
    return (int)raw[white_row*EP_WIDTH+x] - raw[dark_row*EP_WIDTH+x] >= 12;
}
static int ep_goal_column(int side);
/* Validate the selected boundary on a current 40-pixel contour. The nearest
 * previous contour selects its identity; it never supplies target pixels. */
static int ep_contour_support(const uint8_t *raw,const uint8_t *thresholds,
                               int side,int edge,int outer) {
    int support = 0, previous = edge;
    for (int column=0; column<11; ++column) {
        int x = ep_goal_column(side) + (side < 0 ? -4 : 4) * column;
        int best = -1, distance = 8;
        for (int row=previous-6; row<=previous+6; ++row) {
            int transition;
            if (row <= EP_TOP+2 || row >= EP_BOTTOM-2) continue;
            transition = outer ? (!ep_white(raw,thresholds,x,row-1) && ep_white(raw,thresholds,x,row))
                               : (ep_white(raw,thresholds,x,row) && !ep_white(raw,thresholds,x,row+1));
            if (transition && ep_edge_contrast(raw,x,row,outer) && ep_abs(row-previous)<distance) {
                best=row;distance=ep_abs(row-previous);
            }
        }
        if (best<0) continue;
        previous=best;++support;
    }
    return support;
}

/* Every current run is a candidate. Geometry stays provisional until a
 * complete connected, smooth path has passed validation. */
#define EP_MAX_OPENINGS 8

typedef struct {
    int outer, inner, center, width, source;
} ep_column_opening_t;

typedef struct {
    int goal_y, source, support;
    int goal_x;
    int outer[3], inner[3];
    float width, slope, score;
} ep_opening_t;

/* The goal follows the currently observed island extent. The outer image
 * border is only a crop limit; it cannot identify the selected ring branch. */
static int ep_goal_column(int side) {
    int edge = side < 0 ? 0 : EP_WIDTH - 1;
    if (!ep_context.active) return side < 0 ? 182 : 5;
    if (!ep_context.fresh_island) return -1;
    for (int y = EP_TOP; y <= EP_BOTTOM; ++y) {
        if (!ep_context.island_mask[y]) continue;
        if (side < 0 && ep_context.island_right[y] > edge) edge = ep_context.island_right[y];
        if (side > 0 && ep_context.island_left[y] < edge) edge = ep_context.island_left[y];
    }
    if (side < 0) {
        edge -= 6;
        return edge > EP_WIDTH/2 + 12 ? edge : -1;
    }
    edge += 6;
    return edge < EP_WIDTH/2 - 12 ? edge : -1;
}

static int ep_opening_borders_island(const uint8_t *raw, const uint8_t *thresholds,
                                     int x, int last_white_row) {
    if (!ep_context.active) return 1;
    int y = last_white_row + 1;
    return ep_context.fresh_island && y >= EP_TOP && y <= EP_BOTTOM &&
           ep_context.island_mask[y] &&
           x > ep_context.island_left[y] && x < ep_context.island_right[y] &&
           !ep_white(raw, thresholds, x, y);
}

static int ep_column_openings(const uint8_t *raw, const uint8_t *thresholds,
                              int side, int column, ep_column_opening_t *out) {
    int goal_x = ep_goal_column(side);
    if (goal_x < 0) return 0;
    int x = goal_x + (side < 0 ? -4 : 4) * column;
    int count = 0, y = EP_TOP;
    while (y <= EP_BOTTOM && count < EP_MAX_OPENINGS) {
        int start, end, width, center, source;
        while (y <= EP_BOTTOM && !ep_white(raw, thresholds, x, y)) ++y;
        start = y;
        while (y <= EP_BOTTOM && ep_white(raw, thresholds, x, y)) ++y;
        end = y - 1;
        width = end - start + 1;
        if (width < 10) continue;
        /* During entry/orbit, the current island must border this branch on
         * its lower side. A white run BELOW that island is an exit candidate. */
        if (!ep_opening_borders_island(raw, thresholds, x, end)) continue;
        if (start > EP_TOP + 2 && end < EP_BOTTOM - 1) {
            source = 1;
        } else {
            int outer = start > EP_TOP + 2 && end == EP_BOTTOM;
            if (!outer && !(start == EP_TOP && end < EP_BOTTOM - 2)) continue;
            if (ep_previous_side != side || !ep_previous_width) continue;
            if (!ep_edge_contrast(raw, x, outer ? start : end, outer)) continue;
            if (outer && ep_abs(start - ep_previous_outer[column]) > 8) continue;
            if (!outer && ep_abs(end - ep_previous_inner[column]) > 8) continue;
            width = ep_estimated_width((int)ep_previous_goal_y);
            if (width < 10) continue;
            center = outer ? start + (width - 1) / 2 : end - width / 2;
            width = ep_estimated_width(center);
            if (width < 10) continue;
            if (outer) end = start + width - 1;
            else start = end - width + 1;
            source = outer ? 2 : 3;
        }
        center = start + (width - 1) / 2;
        if (center < EP_TOP + 3 || center > EP_BOTTOM - 3 ||
            !ep_white(raw, thresholds, x, center)) continue;
        if (ep_previous_side == side && ep_previous_width) {
            int expected = ep_estimated_width(center);
            int tolerance = expected / 2;
            if (tolerance < 8) tolerance = 8;
            if (!expected || ep_abs(width - expected) > tolerance ||
                ep_abs(center - (int)ep_previous_goal_y) > 12) continue;
        }
        out[count].outer = start;
        out[count].inner = end;
        out[count].center = center;
        out[count].width = width;
        out[count].source = source;
        ++count;
    }
    return count;
}

static int ep_collect_openings(const uint8_t *raw, const uint8_t *thresholds,
                               int side, ep_opening_t *out) {
    ep_column_opening_t columns[3][EP_MAX_OPENINGS];
    int counts[3], count = 0;
    for (int column = 0; column < 3; ++column) {
        counts[column] = ep_column_openings(raw, thresholds, side, column, columns[column]);
        if (!counts[column]) return 0;
    }
    for (int seed = 0; seed < counts[0]; ++seed) {
        ep_opening_t candidate = {0};
        int previous = columns[0][seed].center, sum = 0, has_outer = 0, has_inner = 0;
        for (int column = 0; column < 3; ++column) {
            int best = -1, difference = 11;
            for (int i = 0; i < counts[column]; ++i) {
                int delta = ep_abs(columns[column][i].center - previous);
                if ((!column && i != seed) || delta >= difference) continue;
                best = i;
                difference = delta;
            }
            if (best < 0) break;
            const ep_column_opening_t *observed = &columns[column][best];
            candidate.outer[column] = observed->outer;
            candidate.inner[column] = observed->inner;
            candidate.width += observed->width / 3.0f;
            sum += observed->center;
            previous = observed->center;
            has_outer |= observed->source == 2;
            has_inner |= observed->source == 3;
            if (column != 2) continue;
            if (has_outer && has_inner) break;
            candidate.source = has_outer ? 2 : (has_inner ? 3 : 1);
            candidate.goal_x = ep_goal_column(side);
            candidate.goal_y = sum / 3;
            candidate.slope = (columns[0][seed].center - previous) / 8.0f;
            candidate.support = 3;
            if (candidate.source != 1) {
                int outer = candidate.source == 2;
                int edge = outer ? candidate.outer[0] : candidate.inner[0];
                candidate.support = ep_contour_support(raw, thresholds, side, edge, outer);
                if (candidate.support < 7) break;
            }
            if (ep_previous_side == side && ep_previous_width) {
                int expected = ep_estimated_width(candidate.goal_y);
                candidate.score = fabsf(candidate.goal_y - ep_previous_goal_y) +
                                  12.0f * fabsf(candidate.width - expected) / expected;
            }
            /* Observation count is a small preference, never a hard override
             * of the current corridor's temporal and geometric association. */
            if (candidate.source != 1) candidate.score += 1.0f;
            int position = count++;
            while (position && out[position - 1].score > candidate.score) {
                out[position] = out[position - 1];
                --position;
            }
            out[position] = candidate;
        }
    }
    return count;
}

static void ep_commit_opening(const ep_opening_t *opening, int side) {
    if (opening->source == 1) {
        ep_previous_width = (int)opening->width;
        ep_width_reference_row = (float)opening->goal_y;
        ep_previous_side = side;
        ep_width_age = 0;
        ep_width_tick = ep_context.tick;
        ep_width_distance = ep_context.distance_m;
        ep_width_odometry_valid = ep_context.odometry_valid;
    } else if (ep_width_age < 60000) {
        ++ep_width_age;
    }
    ep_previous_goal_y = (float)opening->goal_y;
    memcpy(ep_previous_outer, opening->outer, sizeof ep_previous_outer);
    memcpy(ep_previous_inner, opening->inner, sizeof ep_previous_inner);
}

static void ep_curve_sample(const float *control, float t, float *x, float *y,
                            float *dx, float *dy, float *ddx, float *ddy);

static void ep_measure_edges(const uint8_t *raw, const uint8_t *thresholds,
                             entry_path_t *path) {
    for (int i = 0; i < path->count; ++i) {
        float x, y, dx, dy, ddx, ddy;
        float scale;
        ep_curve_sample(path->curve,path->curve_t[i],&x,&y,&dx,&dy,&ddx,&ddy);
        scale = fabsf(dx) > fabsf(dy) ? fabsf(dx) : fabsf(dy);
        entry_point_t *point = &path->points[i];
        if (!scale) continue;
        point->measured = 0;
        for (int side = -1; side <= 1; side += 2) {
            int last_x = point->x, last_y = point->y;
            int hit = 0;
            for (int distance = 1; distance < EP_WIDTH; ++distance) {
                int x = (int)(point->x + side * (-dy) * distance / scale + 0.5f);
                int y = (int)(point->y + side * dx * distance / scale + 0.5f);
                if (x < 0 || x >= EP_WIDTH || y < EP_TOP || y > EP_BOTTOM) break;
                if (!ep_white(raw, thresholds, x, y)) { hit = 1; break; }
                last_x = x; last_y = y;
            }
            if (side < 0) {
                point->left_x = (uint8_t)last_x; point->left_y = (uint8_t)last_y;
                if (hit) point->measured |= 1;
            } else {
                point->right_x = (uint8_t)last_x; point->right_y = (uint8_t)last_y;
                if (hit) point->measured |= 2;
            }
        }
    }
}

/* Cubic entrance curves have a continuous tangent. Their final tangent comes
 * from the current side opening's two measured edges. Boundary-normal distances
 * select the curve that follows the corridor rather than cutting its corner. */
static void ep_curve_sample(const float *control, float t, float *x, float *y,
                            float *dx, float *dy, float *ddx, float *ddy) {
    float u = 1.0f - t;
    *x = u*u*u*control[0] + 3*u*u*t*control[2] + 3*u*t*t*control[4] + t*t*t*control[6];
    *y = u*u*u*control[1] + 3*u*u*t*control[3] + 3*u*t*t*control[5] + t*t*t*control[7];
    *dx = 3*(u*u*(control[2]-control[0]) + 2*u*t*(control[4]-control[2]) + t*t*(control[6]-control[4]));
    *dy = 3*(u*u*(control[3]-control[1]) + 2*u*t*(control[5]-control[3]) + t*t*(control[7]-control[5]));
    *ddx = 6*(u*(control[4]-2*control[2]+control[0]) + t*(control[6]-2*control[4]+control[2]));
    *ddy = 6*(u*(control[5]-2*control[3]+control[1]) + t*(control[7]-2*control[5]+control[3]));
}

static float ep_curve_score(const uint8_t *raw, const uint8_t *thresholds,
                             const float *control, int turn_side, float measured_width, int boundary_source) {
    float score = 0;
    int last_x = -1, last_y = -1;
    for (int sample = 0; sample <= 128; ++sample) {
        float x, y, dx, dy, ddx, ddy;
        int px, py;
        ep_curve_sample(control, sample / 128.0f, &x, &y, &dx, &dy, &ddx, &ddy);
        px = (int)(x + 0.5f); py = (int)(y + 0.5f);
        if (!ep_white(raw, thresholds, px, py) || !ep_white(raw, thresholds, px-1, py) ||
            !ep_white(raw, thresholds, px+1, py) || !ep_white(raw, thresholds, px, py-1) ||
            !ep_white(raw, thresholds, px, py+1)) return -1;
        if (ep_context.active && ep_context.fresh_island && ep_context.island_mask[py] &&
            ((turn_side < 0 && px > ep_context.island_left[py]) ||
             (turn_side > 0 && px < ep_context.island_right[py]))) return -1;
        if (last_x >= 0) {
            int steps = ep_abs(px-last_x) > ep_abs(py-last_y) ? ep_abs(px-last_x) : ep_abs(py-last_y);
            if (steps > 3) return -1;
            for (int k = 1; k < steps; ++k) {
                int xx = last_x + (px-last_x)*k/steps;
                int yy = last_y + (py-last_y)*k/steps;
                if (!ep_white(raw, thresholds, xx, yy)) return -1;
            }
        }
        last_x = px; last_y = py;
        if (sample % 4 == 0) {
            float scale = fabsf(dx) > fabsf(dy) ? fabsf(dx) : fabsf(dy);
            float speed_sq = dx*dx + dy*dy;
            float cross = dx*ddy - dy*ddx;
            int inner_distance = 0, inner_x = 0, inner_y = 0;
            int inner_side = turn_side < 0 ? 1 : -1;
            if (boundary_source == 2) inner_side = -inner_side;
            if (scale < 1 || speed_sq < 1) return -1;
            /* Reject image-space hairpins before curvature smoothing is scored. */
            if (cross*cross > 0.0225f*speed_sq*speed_sq*speed_sq) return -1;
            score += 100*cross*cross/(speed_sq*speed_sq*speed_sq);
            for (int side = -1; side <= 1; side += 2) {
                for (int distance = 1; distance < EP_WIDTH; ++distance) {
                    int xx = (int)(x + side*(-dy)*distance/scale + 0.5f);
                    int yy = (int)(y + side*dx*distance/scale + 0.5f);
                    if (xx < 0 || xx >= EP_WIDTH || yy < EP_TOP || yy > EP_BOTTOM) break;
                    if (!ep_white(raw, thresholds, xx, yy)) {
                        if (side == inner_side) {
                            inner_distance = distance;
                            inner_x = xx; inner_y = yy;
                        }
                        break;
                    }
                }
            }
            {
                /* The currently selected road edge constrains the turn. Offset it
                 * by HALF the estimated road width, allowing the width's
                 * existing row-perspective model to grow toward the camera. */
                float t = sample / 128.0f;
                float weight = t < 0.4f ? t/0.4f : 1.0f;
                float model_here = IP_WIDTH_CAL_PX/2.0f + IP_HALF_PERSPECTIVE_SPAN*(y-IP_WIDTH_CAL_ROW)/(MT9V03X_H-1);
                float model_goal = IP_WIDTH_CAL_PX/2.0f + IP_HALF_PERSPECTIVE_SPAN*(control[7]-IP_WIDTH_CAL_ROW)/(MT9V03X_H-1);
                float end_dx = control[6]-control[4], end_dy = control[7]-control[5];
                float normal_scale = fabsf(end_dx)/sqrtf(end_dx*end_dx+end_dy*end_dy);
                float half_width = 0.5f*measured_width*normal_scale*model_here/model_goal;
                if (inner_distance && half_width >= 3) {
                    float actual_distance = inner_distance*sqrtf(speed_sq)/scale;
                    float difference = (actual_distance-half_width)/half_width;
                    int gx = 0, gy = 0;
                    score += 100*weight*weight*difference*difference;
                    /* A short binary gradient estimates the currently observed edge
                     * tangent. Its normal must agree with the curve normal. */
                    for (int offset=-2; offset<=2; ++offset) {
                        gx += ep_white(raw,thresholds,inner_x+2,inner_y+offset) -
                              ep_white(raw,thresholds,inner_x-2,inner_y+offset);
                        gy += ep_white(raw,thresholds,inner_x+offset,inner_y+2) -
                              ep_white(raw,thresholds,inner_x+offset,inner_y-2);
                    }
                    if (gx*gx+gy*gy >= 4) {
                        float normal_dot = gx*(-dy)+gy*dx;
                        float alignment = normal_dot*normal_dot/((gx*gx+gy*gy)*speed_sq);
                        score += 35*weight*weight*(1.0f-alignment);
                    }
                } else {
                    score += 25*weight*weight;
                }
            }
        }
    }
    return score;
}

/* Four-pixel spacing avoids 45-degree one-pixel stair steps caused by rounding
 * an otherwise smooth curve. Validate the actual exported integer segments,
 * including the final endpoint and every connecting white segment. */
static int ep_sample_curve(const uint8_t *raw, const uint8_t *thresholds,
                            const float *control, entry_path_t *path) {
    path->count = 0;
#if TRACK_GEOMETRY_CALIBRATED
    float gx[3]={0},gy[3]={0};
    for(int i=0;i<=32;++i) {
        float x,y,dx,dy,ddx,ddy,k;
        ep_curve_sample(control,i/32.0f,&x,&y,&dx,&dy,&ddx,&ddy);
        gx[0]=gx[1];gx[1]=gx[2];gy[0]=gy[1];gy[1]=gy[2];
        if(!track_project(x,y,&gx[2],&gy[2]))return 0;
        if(i>=2 && !track_curvature(gx[0],gy[0],gx[1],gy[1],gx[2],gy[2],&k))return 0;
    }
#endif
    for (int sample = 0; sample <= 128; ++sample) {
        float x, y, dx, dy, ddx, ddy;
        int px, py;
        ep_curve_sample(control, sample / 128.0f, &x, &y, &dx, &dy, &ddx, &ddy);
        px = (int)(x + 0.5f); py = (int)(y + 0.5f);
        if (path->count) {
            const entry_point_t *last = &path->points[path->count - 1];
            int delta_x = px - last->x, delta_y = py - last->y;
            if (delta_x * delta_x + delta_y * delta_y <
                EP_MIN_CURVE_POINT_DISTANCE * EP_MIN_CURVE_POINT_DISTANCE) {
                if (sample < 128) continue;
                /* Merge the short terminal tail into the preceding segment. */
                --path->count;
            }
        }
        entry_point_t *point = &path->points[path->count];
        memset(point, 0, sizeof *point);
        point->x = (uint8_t)px; point->y = (uint8_t)py;
        path->curve_t[path->count++] = sample / 128.0f;
    }
    int count = path->count;
    path->count = 0;
    for (int i = 0; i < count; ++i) {
        const entry_point_t *point = &path->points[i];
        if (!ep_path_turn_allowed(path, point->x, point->y)) return 0;
        if (!ep_white(raw, thresholds, point->x, point->y)) return 0;
        if (i && !ep_segment_white(raw, thresholds, path->points[i-1].x,
                                  path->points[i-1].y, point->x, point->y)) return 0;
        ++path->count;
    }
    return path->count >= 12;
}

static int ep_fit_curve(const uint8_t *raw, const uint8_t *thresholds,
                         int side, float end_slope, entry_path_t *path) {
    float start_x = path->points[0].x, start_y = path->points[0].y;
    float end_x = path->points[path->count-1].x, end_y = path->points[path->count-1].y;
    float width = fabsf(end_x-start_x), height = fabsf(end_y-start_y);
    float direction = side < 0 ? 1.0f : -1.0f;
    float tangent_scale = sqrtf(1.0f+end_slope*end_slope);
    float best = -1;
    static entry_path_t sampled_path;
    for (int a = 0; a < 5; ++a) {
        for (int b = 0; b < 5; ++b) {
            float first_length = height*(0.40f+0.15f*a);
            float last_length = width*(0.35f+0.15f*b);
            float control[8], score;
            if (first_length < 16) first_length = 16;
            if (last_length < 16) last_length = 16;
            if (first_length > 120) first_length = 120;
            if (last_length > 120) last_length = 120;
            control[0]=start_x; control[1]=start_y;
            control[2]=start_x; control[3]=start_y-first_length;
            control[4]=end_x-direction*last_length/tangent_scale;
            control[5]=end_y-end_slope*last_length/tangent_scale;
            control[6]=end_x; control[7]=end_y;
            score = ep_curve_score(raw,thresholds,control,side,path->estimated_width,path->boundary_source);
            if (score >= 0 && (best < 0 || score < best) &&
                ep_sample_curve(raw, thresholds, control, &sampled_path)) {
                best = score;
                memcpy(path->curve,control,sizeof control);
                memcpy(path->points,sampled_path.points,sizeof path->points);
                memcpy(path->curve_t,sampled_path.curve_t,sizeof path->curve_t);
                path->count = sampled_path.count;
            }
        }
    }
    if (best < 0) return 0;
    return path->count >= 12;
}

/* Select control and preview by distance along the path. Extending a distant
 * endpoint must not move the near controller to half of that longer path. */
static void ep_control_points(entry_path_t *path) {
    int near_index = path->count - 1, preview_index = path->count - 1;
    float distance = 0;
    for (int i = 1; i < path->count; ++i) {
        float dx = (float)path->points[i].x - path->points[i-1].x;
        float dy = (float)path->points[i].y - path->points[i-1].y;
        distance += sqrtf(dx*dx + dy*dy);
        if (distance >= 30.0f && near_index == path->count - 1) near_index = i;
        if (distance >= 60.0f) { preview_index = i; break; }
    }
    path->error = (int16_t)(path->points[near_index].x - EP_WIDTH/2);
    path->preview = (int16_t)(path->points[preview_index].x - EP_WIDTH/2);
}

static int ep_build_opening(const uint8_t *raw, const uint8_t *thresholds,
                            int side, int start_x, const ep_opening_t *opening, entry_path_t *path) {
    int goal_y = opening->goal_y;
    int goal_x = ep_context.active ? opening->goal_x : (side < 0 ? 182 : 5);
    int start, goal, current;
    memset(path, 0, sizeof *path);
    path->inferred = (uint8_t)(opening->source != 1);
    path->boundary_source = (uint8_t)opening->source;
    path->support = (uint8_t)opening->support;
    /* Each grid cell must have an entirely white 2x2 footprint. */
    for (int y = 0; y < EP_ROWS; ++y) {
        for (int x = 0; x < EP_COLS; ++x) {
            int px = 2 * x, py = EP_TOP + 2 * y;
            int cell = y * EP_COLS + x;
            ep_clearance[cell] = (uint8_t)((ep_white(raw, thresholds, px, py) &&
                ep_white(raw, thresholds, px + 1, py) && ep_white(raw, thresholds, px, py + 1) &&
                ep_white(raw, thresholds, px + 1, py + 1)) ? 200 : 0);
            ep_cost[cell] = UINT32_MAX;
            ep_parent[cell] = -1;
            ep_position[cell] = -1;
        }
    }
    /* Manhattan distance to actual dark pixels, in grid cells. */
    for (int cell = 0; cell < EP_CELLS; ++cell) {
        int value = ep_clearance[cell];
        if (cell >= EP_COLS && value > ep_clearance[cell - EP_COLS] + 1) value = ep_clearance[cell - EP_COLS] + 1;
        if (cell % EP_COLS && value > ep_clearance[cell - 1] + 1) value = ep_clearance[cell - 1] + 1;
        ep_clearance[cell] = (uint8_t)value;
    }
    for (int cell = EP_CELLS - 1; cell >= 0; --cell) {
        int value = ep_clearance[cell];
        if (cell < EP_CELLS - EP_COLS && value > ep_clearance[cell + EP_COLS] + 1) value = ep_clearance[cell + EP_COLS] + 1;
        if (cell % EP_COLS < EP_COLS - 1 && value > ep_clearance[cell + 1] + 1) value = ep_clearance[cell + 1] + 1;
        ep_clearance[cell] = (uint8_t)value;
    }
    start = ((100 - EP_TOP) / 2) * EP_COLS + start_x / 2;
    goal = ((goal_y - EP_TOP) / 2) * EP_COLS + goal_x / 2;
    if (start_x < 6 || start_x > EP_WIDTH - 7 || ep_clearance[start] < 2 || ep_clearance[goal] < 2) {
        path->failure = 2; return 0;
    }
    ep_heap_count = 0;
    ep_cost[goal] = 0;
    ep_heap_update(goal);
    while (ep_heap_count) {
        current = ep_heap_pop();
        if (current == start) break;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int x = current % EP_COLS + dx;
                int y = current / EP_COLS + dy;
                int next;
                uint32_t cost;
                if ((!dx && !dy) || x < 0 || x >= EP_COLS || y < 0 || y >= EP_ROWS) continue;
                next = y * EP_COLS + x;
                if (ep_clearance[next] < 2 || ep_position[next] == -2) continue;
                if (dx && dy && (!ep_clearance[current + dx] || !ep_clearance[current + dy * EP_COLS])) continue;
                /* Prefer clearance from BOTH walls over the old straight edge. */
                cost = ep_cost[current] + (dx && dy ? 14U : 10U) + 120U / ep_clearance[next];
                if (cost >= ep_cost[next]) continue;
                ep_cost[next] = cost;
                ep_parent[next] = (int16_t)current;
                ep_heap_update(next);
            }
        }
    }
    if (ep_parent[start] < 0) { path->failure = 3; return 0; }
    current = start;
    while (path->count < EP_MAX_POINTS) {
        entry_point_t *point = &path->points[path->count++];
        memset(point, 0, sizeof *point);
        point->x = (uint8_t)(2 * (current % EP_COLS));
        point->y = (uint8_t)(EP_TOP + 2 * (current / EP_COLS));
        if (current == goal) break;
        current = ep_parent[current];
        if (current < 0) { path->count = 0; path->failure = 4; return 0; }
    }
    if (current != goal || path->count < 12) { path->count = 0; path->failure = 4; return 0; }
    path->goal_x = (uint8_t)goal_x; path->goal_y = (uint8_t)goal_y;
    path->estimated_width = opening->width;
    if (!ep_fit_curve(raw,thresholds,side,opening->slope,path)) {
        path->count = 0; path->failure = 6; return 0;
    }
    ep_measure_edges(raw, thresholds, path);
    /* Follow a point along the ordered path, independent of image row order. */
    {
        int lookahead = path->count / 2;
        int preview = path->count * 3 / 4;
        path->error = (int16_t)(path->points[lookahead].x - EP_WIDTH / 2);
        path->preview = (int16_t)(path->points[preview].x - EP_WIDTH / 2);
        if (ep_context.active) ep_control_points(path);
    }
    return 1;
}
static int ep_build(const uint8_t *raw, const uint8_t *thresholds,
                    int side, int start_x, entry_path_t *path) {
    ep_opening_t openings[EP_MAX_OPENINGS];
    static entry_path_t trial_path;
    int count, accepted = -1, failure = 1;
    memset(path, 0, sizeof *path);
    if (!side) { path->failure = 1; return 0; }
    count = ep_collect_openings(raw, thresholds, side, openings);
    for (int i = 0; i < count; ++i) {
        /* A second similarly associated, feasible corridor is ambiguous. */
        if (accepted >= 0 && openings[i].score > openings[accepted].score + 6.0f) break;
        if (!ep_build_opening(raw, thresholds, side, start_x, &openings[i], &trial_path)) {
            if (trial_path.failure > failure) failure = trial_path.failure;
            continue;
        }
        if (accepted >= 0) {
            memset(path, 0, sizeof *path);
            path->failure = 7;
            return 0;
        }
        accepted = i;
        *path = trial_path;
    }
    if (accepted < 0) { path->failure = (uint8_t)failure; return 0; }
    ep_commit_opening(&openings[accepted], side);
    return 1;
}

/* Fit the complete currently measured forward corridor with one cubic. The
 * row observations constrain the fit, but are never exported as steering
 * targets. This prevents edge-visibility changes from producing a snake. */
static int ep_build_measured_corridor(const uint8_t *raw, const uint8_t *thresholds,
                                      int side, int start_x, entry_path_t *path) {
    float target[EP_ROWS], weight[EP_ROWS];
    int left[EP_ROWS], right[EP_ROWS];
    int rows = 0, observations = 0, near_observations = 0;
    int reference = start_x, previous_left = start_x, previous_right = start_x;
    static entry_path_t candidate, best_path;
    memset(path, 0, sizeof *path);
    path->failure = 8;
    if (!side || !ep_white(raw, thresholds, start_x, EP_BOTTOM)) return 0;
    for (int y = EP_BOTTOM; y >= EP_TOP+8; y -= 2) {
        int selected_left = -1, selected_right = -1, selected_center = 0;
        int selected_score = 1000000;
        float selected_weight = 0;
        for (int x = 0; x < EP_WIDTH;) {
            int begin, end, center, score;
            while (x < EP_WIDTH && !ep_white(raw, thresholds, x, y)) ++x;
            begin = x;
            while (x < EP_WIDTH && ep_white(raw, thresholds, x, y)) ++x;
            end = x-1;
            if (end-begin+1 < 10) continue;
            if (!rows && (begin > start_x-2 || end < start_x+2)) continue;
            if (rows && (end < previous_left || begin > previous_right)) continue;
            center = (begin+end)/2;
            float confidence = 1;
            if (ep_context.active) {
                int half = ep_context.half_width[y];
                if (!begin && end == EP_WIDTH-1) {
                    center = reference;
                    confidence = 0;
                } else if (!begin) center = end-half;
                else if (end == EP_WIDTH-1) center = begin+half;
                if (ep_context.fresh_island && ep_context.island_mask[y]) {
                    if (side < 0 && end > ep_context.island_left[y]+1) continue;
                    if (side > 0 && begin < ep_context.island_right[y]-1) continue;
                    confidence = 2;
                }
            }
            /* Exit search needs measured road banks, not an unbounded white
             * opening or a transverse near edge. The selected branch must stay
             * compatible with the last accepted exit endpoint. */
            if (ep_context.exit_mode && y <= 75) {
                int expected = 2*ep_context.half_width[y];
                if (begin<=2 || end>=EP_WIDTH-3 ||
                    end-begin+1<24 || end-begin+1>expected+20) continue;
                if (ep_context.exit_mode==2 &&
                    ep_abs(center-(EP_WIDTH/2+ep_context.exit_reference))>40) continue;
            }
            if (center < begin+2) center = begin+2;
            if (center > end-2) center = end-2;
            score = ep_abs(center-reference);
            if (score >= selected_score) continue;
            selected_score = score;
            selected_left = begin; selected_right = end;
            selected_center = center; selected_weight = confidence;
        }
        if (selected_left < 0) break;
        /* An observed edge that jumps to another bank ends this lookahead.
         * Joining both banks would bend the curve into a different branch. */
        if (ep_context.active && !ep_context.fresh_island && !ep_context.exit_mode && rows &&
            selected_weight > 0 && weight[rows-1] > 0 &&
            ((selected_left > 0 && previous_left > 0 &&
              ep_abs(selected_left-previous_left) > 16) ||
             (selected_right < EP_WIDTH-1 && previous_right < EP_WIDTH-1 &&
              ep_abs(selected_right-previous_right) > 16))) break;
        left[rows] = selected_left; right[rows] = selected_right;
        target[rows] = (float)selected_center;
        weight[rows] = selected_weight;
        if (selected_weight > 0) {
            ++observations;
            if (y >= EP_BOTTOM-30) ++near_observations;
        }
        reference = selected_center;
        previous_left = selected_left; previous_right = selected_right;
        ++rows;
    }
    path->support = (uint8_t)observations;
    path->goal_y = (uint8_t)(EP_BOTTOM-2*(rows-1));
#ifdef EP_FIT_DIAGNOSTICS
    ep_fit_trace(rows,observations,near_observations,target,weight,left,right);
#endif
    if (rows < 12 || observations < 12 || near_observations < 4) { path->failure = 10; return 0; }
    int measured_rows = rows;
    for (rows = measured_rows; rows >= 12; rows -= 4) {
        float bb = 0, be = 0, ee = 0, bx = 0, ex = 0;
        int fit_observations = 0, fit_near = 0, fit_single = 0;
        float height = 2.0f*(rows-1);
        for (int i = 0; i < rows; ++i) {
            float t = i/(float)(rows-1), b = 3*(1-t)*t*t, e = t*t*t;
            float delta = target[i]-start_x, w = weight[i];
            bb += w*b*b; be += w*b*e; ee += w*e*e;
            bx += w*b*delta; ex += w*e*delta;
            if (w > 0) {
                ++fit_observations;
                if (2*i <= 30) ++fit_near;
                if (!left[i] || right[i] == EP_WIDTH-1) ++fit_single;
            }
        }
        if (fit_observations < 12 || fit_near < 4) continue;
        if (ep_context.exit_mode && EP_BOTTOM-(int)height>55) continue;
        /* Integral of squared x'' penalizes bends across the entire curve. */
        bb += 0.072f; be -= 0.036f; ee += 0.024f;
        float determinant = bb*ee-be*be;
        if (determinant < 0.0001f) { path->failure = 11; return 0; }
        float fitted_b = (bx*ee-ex*be)/determinant;
        float fitted_e = (ex*bb-bx*be)/determinant;
        float initial_b_limit = 0.025f*height*height;
        if (fitted_b > initial_b_limit) fitted_b = initial_b_limit;
        if (fitted_b < -initial_b_limit) fitted_b = -initial_b_limit;
        float best_score = -1;
        for (int bend = -3; bend <= 3; ++bend) {
            for (int end = -3; end <= 3; ++end) {
                float b = fitted_b + 6*bend, e = fitted_e + 6*end;
                float control[8] = {(float)start_x, EP_BOTTOM, (float)start_x, EP_BOTTOM-height/3,
                                    start_x+b, EP_BOTTOM-2*height/3, start_x+e, EP_BOTTOM-height};
                float score = 0.002f*(36*b*b-36*b*e+12*e*e);
                int feasible = 1;
                for (int i = 0; i < rows; ++i) {
                    float t = i/(float)(rows-1);
                    float x = start_x + 3*(1-t)*t*t*b + t*t*t*e;
                    float delta = x-target[i];
                    if (x < left[i]+2 || x > right[i]-2) { feasible = 0; break; }
                    score += weight[i]*delta*delta;
                }
                if (!feasible || (best_score >= 0 && score >= best_score)) continue;
                if (!ep_sample_curve(raw, thresholds, control, &candidate)) continue;
                /* Check the continuous curve as well as the exported integer path. */
                for (int i = 0; i <= 128; ++i) {
                    float x,y,dx,dy,ddx,ddy;
                    ep_curve_sample(control,i/128.0f,&x,&y,&dx,&dy,&ddx,&ddy);
                    int px = (int)(x+0.5f), py = (int)(y+0.5f);
                    float speed2 = dx*dx+dy*dy, cross = dx*ddy-dy*ddx;
                    if (!ep_white(raw,thresholds,px-1,py) || !ep_white(raw,thresholds,px+1,py) ||
                        cross*cross > 0.0225f*speed2*speed2*speed2 ||
                        (ep_context.active && ep_context.fresh_island && ep_context.island_mask[py] &&
                         ((side < 0 && px > ep_context.island_left[py]) ||
                          (side > 0 && px < ep_context.island_right[py])))) { feasible = 0; break; }
                }
                if (!feasible) continue;
                best_score = score;
                best_path = candidate;
                memcpy(best_path.curve,control,sizeof control);
            }
        }
        if (best_score < 0) continue;
        *path = best_path;
        path->failure = 0;
        path->boundary_source = 5;
        path->support = (uint8_t)fit_observations;
        path->inferred = (uint8_t)(fit_single > fit_observations/2);
        path->goal_x = path->points[path->count-1].x;
        path->goal_y = path->points[path->count-1].y;
        path->estimated_width = (float)(right[rows/2]-left[rows/2]+1);
        ep_measure_edges(raw,thresholds,path);
        ep_control_points(path);
        return 1;
    }
    path->failure = 12;
    return 0;
}
#endif /* RING_ENTRY_IMPLEMENTATION */
#endif
