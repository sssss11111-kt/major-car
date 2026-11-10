/* Common route guards and bounded image-space geometry. GPL-3.0-or-later. */
#include "special_route.h"
#include <math.h>

int special_route_valid(const special_route_config_t *route)
{
    return route->enabled && route->route_valid &&
           route->entry_port >= PORT_A && route->entry_port <= PORT_D &&
           route->exit_port >= PORT_A && route->exit_port <= PORT_D &&
           (route->direction == ROUTE_CW || route->direction == ROUTE_CCW);
}

int special_fsm_expired(special_fsm_timer_t *timer, const special_fsm_config_t *cfg,
                        int path_current, uint32_t now)
{
    if (!cfg->stable_frames || !cfg->loss_timeout_ms || !cfg->maneuver_timeout_ms ||
        now - timer->started_ms >= cfg->maneuver_timeout_ms ||
        now - timer->last_path_ms >= cfg->loss_timeout_ms) return 1;
    if (path_current) timer->last_path_ms = now;
    return 0;
}

int special_fsm_stable(special_fsm_timer_t *timer, unsigned required, int condition)
{
    if (!condition) {
        timer->stable = 0;
        return 0;
    }
    if (timer->stable < required) ++timer->stable;
    if (timer->stable < required) return 0;
    timer->stable = 0;
    return 1;
}

int special_path_center(int left, int right, unsigned width, int16_t *center)
{
    if (left < -1 || left >= CAR_W || right < -1 || right >= CAR_W) return 0;
    if (left >= 0 && right >= 0) {
        if (left >= right) return 0;
    } else {
        if ((left < 0 && right < 0) || width < 2 || width >= CAR_W) return 0;
        if (left < 0) left = right - (int)width;
        else right = left + (int)width;
    }
    if (left + right < 0 || left + right > 2 * (CAR_W - 1)) return 0;
    *center = (int16_t)((left + right) / 2);
    return 1;
}

static float curvature(const special_path_point_t *a, const special_path_point_t *b,
                       const special_path_point_t *c)
{
    int ax = b->x - a->x, ay = a->y - b->y;
    int bx = c->x - b->x, by = b->y - c->y;
    int cx = ax + bx, cy = ay + by;
    float length_product = sqrtf((float)(ax * ax + ay * ay) * (bx * bx + by * by) *
                                 (cx * cx + cy * cy));
    return 2.0f * (ay * bx - ax * by) / length_product;
}

static float path_x(const special_path_point_t *a, const special_path_point_t *b, int y)
{
    float fraction = (a->y - y) / (float)(a->y - b->y);
    return a->x + (b->x - a->x) * fraction;
}

int special_path_target(const special_path_t *path, unsigned lookahead_y,
                        special_plan_output_t *out)
{
    *out = (special_plan_output_t){0};
    if (path->count < 3 || path->count > SPECIAL_PATH_POINTS || lookahead_y >= CAR_H) return 0;
    for (unsigned i = 0; i < path->count; ++i) {
        const special_path_point_t *p = &path->points[i];
        if (p->x < 0 || p->x >= CAR_W || p->y < 0 || p->y >= CAR_H ||
            (i > 0 && p->y >= path->points[i - 1].y)) return 0;
    }
    if (lookahead_y > (unsigned)path->points[0].y ||
        lookahead_y < (unsigned)path->points[path->count - 1].y) return 0;
    for (unsigned i = 2; i < path->count; ++i) {
        float k = curvature(&path->points[i - 2], &path->points[i - 1], &path->points[i]);
        if (i == 2) out->curvature_px_inv = k;
        if (fabsf(k) > out->max_curvature_px_inv) out->max_curvature_px_inv = fabsf(k);
    }
    for (unsigned i = 1; i < path->count; ++i) {
        const special_path_point_t *a = &path->points[i - 1], *b = &path->points[i];
        if (lookahead_y < (unsigned)b->y) continue;
        out->target_error = path_x(a, b, (int)lookahead_y) - (CAR_W - 1) * 0.5f;
        break;
    }
    out->target_valid = 1;
    return 1;
}

int special_ring_entry_target(const special_plan_evidence_t *e, special_plan_output_t *out)
{
    special_plan_output_t measured;
    *out = (special_plan_output_t){0};
    if (!e->current || !e->calibrated ||
        !isfinite(e->entry_min_clearance_px) || e->entry_min_clearance_px <= 0 ||
        !isfinite(e->entry_max_curvature_px_inv) || e->entry_max_curvature_px_inv <= 0 ||
        !isfinite(e->entry_max_join_offset_px) || e->entry_max_join_offset_px <= 0 ||
        e->entry_reference_x < 0 || e->entry_reference_x >= CAR_W ||
        (e->entry_inner_side != -1 && e->entry_inner_side != 1) ||
        !special_path_target(&e->entry, e->lookahead_y, &measured) ||
        measured.max_curvature_px_inv > e->entry_max_curvature_px_inv) return 0;
    const special_path_point_t *first = &e->entry.points[0];
    if (e->entry_inner_bottom >= CAR_H || first->y > e->entry_inner_bottom ||
        e->entry.points[e->entry.count - 1].y < e->entry_inner_top ||
        fabsf((float)(first->x - e->entry_reference_x)) > e->entry_max_join_offset_px) return 0;
    for (unsigned i = 1; i < e->entry.count; ++i) {
        const special_path_point_t *a = &e->entry.points[i - 1], *b = &e->entry.points[i];
        for (int y = a->y; y >= b->y; --y) {
            int inner = e->entry_inner_x[y];
            float clearance = (inner - path_x(a, b, y)) * e->entry_inner_side;
            if (inner < 0 || inner >= CAR_W || clearance < e->entry_min_clearance_px) return 0;
        }
    }
    *out = measured;
    return 1;
}
