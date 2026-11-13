#ifndef CENTER_ROUTE_H
#define CENTER_ROUTE_H

/* Included only by img_process.c after its private state and path helpers. */
static int ip_fit_center(const uint8_t *mid, int top, int bottom, float control[4])
{
    float weights[120] = {0};
    for (int y = top; y <= bottom; ++y)
        weights[y] = mid[y] ? (y >= 80 ? 3.0f : 1.0f) : 0;
    for (int pass = 0; pass < 3; ++pass) {
        float matrix[4][5] = {{0}};
        int count = 0;
        for (int y = top; y <= bottom; ++y) {
            if (!mid[y]) continue;
            float t = (float)(bottom - y) / (bottom - top);
            float u = 1 - t, basis[4] = {u*u*u, 3*u*u*t, 3*u*t*t, t*t*t};
            for (int i = 0; i < 4; ++i) {
                matrix[i][4] += weights[y] * basis[i] * mid[y];
                for (int j = 0; j < 4; ++j) matrix[i][j] += weights[y] * basis[i] * basis[j];
            }
            ++count;
        }
        if (count < 12) return 0;
        const float penalty[2][4] = {{1,-2,1,0},{0,1,-2,1}};
        for (int k = 0; k < 2; ++k)
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    matrix[i][j] += 1.5f * penalty[k][i] * penalty[k][j];
        for (int i = 0; i < 4; ++i) {
            float pivot = matrix[i][i];
            if (fabsf(pivot) < 0.0001f) return 0;
            for (int j = i; j < 5; ++j) matrix[i][j] /= pivot;
            for (int row = 0; row < 4; ++row) {
                if (row == i) continue;
                float factor = matrix[row][i];
                for (int j = i; j < 5; ++j) matrix[row][j] -= factor * matrix[i][j];
            }
        }
        for (int i = 0; i < 4; ++i) control[i] = matrix[i][4];
        for (int y = top; y <= bottom; ++y) {
            if (!mid[y]) continue;
            float t = (float)(bottom - y) / (bottom - top), u = 1 - t;
            float x = u*u*u*control[0] + 3*u*u*t*control[1] + 3*u*t*t*control[2] + t*t*t*control[3];
            float residual = fabsf(x - mid[y]);
            weights[y] = (y >= 80 ? 3.0f : 1.0f) * (residual > 6 ? 6 / residual : 1);
        }
    }
    return 1;
}

static void ip_center_errors(const uint8_t *mid)
{
    long near_sum = 0, preview_sum = 0;
    int near_weight = 0, preview_count = 0;
    for (int y = 55; y <= IP_TRACK_BOTTOM; ++y) {
        if (!mid[y]) continue;
        if (y <= 75) { preview_sum += mid[y]; ++preview_count; }
        if (y >= IP_ERROR_ROW_TOP) {
            int weight = IP_ERROR_NEAR + y - IP_ERROR_ROW_TOP;
            near_sum += mid[y] * weight; near_weight += weight;
        }
    }
    s_near_valid = near_weight > 0;
    if (near_weight) s_error = (int16_t)(near_sum / near_weight - IP_W / 2);
    s_preview_valid = (uint8_t)(s_valid && preview_count >= 10);
    s_preview_error = s_preview_valid ? (int16_t)(preview_sum / preview_count - IP_W / 2) : 0;
    s_ring_entry_error = s_error;
    s_have_error = 1;
}

static void ip_smooth_center(const uint8_t *raw, const uint8_t *thresholds, uint8_t *mid)
{
    uint8_t observed[IP_H];
    memcpy(observed, mid, sizeof observed);
    s_center_route.count = 0;
    int bottom=IP_TRACK_BOTTOM;
    while (bottom>=90 && !observed[bottom]) --bottom;
    for (int top = IP_TRACK_TOP; bottom>=90 && top <= 78; top += 4) {
        float x[4], control[8];
        if (!ip_fit_center(observed, top, bottom, x)) continue;
        for (int i = 0; i < 4; ++i) {
            control[2*i] = x[i];
            control[2*i+1] = bottom - (bottom-top) * i / 3.0f;
        }
        entry_path_t path = {0};
        uint8_t center[IP_H] = {0};
        int valid = 1;
        for (int y = top; y <= bottom; ++y) {
            float t = (float)(bottom-y) / (bottom-top), u = 1-t;
            int value = (int)(u*u*u*x[0] + 3*u*u*t*x[1] + 3*u*t*t*x[2] + t*t*t*x[3] + .5f);
            if (!ep_white(raw, thresholds, value, y) ||
                (y > top && ip_abs(value - center[y-1]) > 2)) { valid = 0; break; }
            center[y] = (uint8_t)value;
        }
        if (!valid) continue;
        for(int y=bottom; y>=top; y-=4) {
            if(!ep_path_turn_allowed(&path,center[y],y)) {valid=0;break;}
            if(path.count && !ep_segment_white(raw,thresholds,
                path.points[path.count-1].x,path.points[path.count-1].y,center[y],y)) {valid=0;break;}
            path.points[path.count++] = (entry_point_t){.x=center[y],.y=(uint8_t)y};
        }
        if(!valid || path.count<6)continue;
        for(int i=1;i<path.count && valid;++i) {
            const entry_point_t *a=&path.points[i-1], *b=&path.points[i];
            int steps=ip_abs((int)b->x-a->x);
            if(ip_abs((int)b->y-a->y)>steps)steps=ip_abs((int)b->y-a->y);
            for(int k=0;k<=steps;++k) {
                int px=(int)floorf(a->x+((float)b->x-a->x)*k/steps+.5f);
                int py=(int)floorf(a->y+((float)b->y-a->y)*k/steps+.5f);
                if(!ep_white(raw,thresholds,px,py)) {valid=0;break;}
            }
        }
        if(!valid)continue;
        memcpy(mid, center, sizeof center);
        s_center_route = path;
        s_center_route.boundary_source = 6;
        memcpy(s_center_route.curve, control, sizeof control);
        ip_center_errors(mid);
        s_center_route.error = s_error; s_center_route.preview = s_preview_error;
        return;
    }
    memset(mid, 0, IP_H);
    s_valid = s_near_valid = s_preview_valid = 0;
}

/* During the handoff the near bank can still be outside the image.
 * Seed the usual smoothing pass with this frame's continuous measured
 * corridor. Carry its last measured midpoint only across the open stem. */
static void ip_seed_departure(uint8_t *mid)
{
    const cross_straight_t *s = &s_cross_straight;
    int stem = 0, support = 0;
    for (int y = 48; y <= CS_BOTTOM; ++y) {
        int l = s->bend_left[y], r = s->bend_right[y];
        if (!l || !r || r-l < 40) continue;
        stem = (l+r)/2;
        ++support;
    }
    if (support < 12) return;
    for (int y = CS_TOP; y <= CS_BOTTOM; ++y) {
        int l = s->bend_left[y], r = s->bend_right[y];
        if (l && r && r-l >= 40) mid[y] = (uint8_t)((l+r)/2);
        else if (y >= 72) mid[y] = (uint8_t)stem;
        else if (y < 40) {
            int target = stem;
            /* The outer bank is almost horizontal at the far end. Sample
               the bank above the target to keep that target inside the turn. */
            for (int i = 0; i < s->outer_count; ++i)
                if (s->outer_y[i] >= y-10 && s->outer_y[i] <= y-5)
                    target = s->outer_x[i];
            int limit = (40-y)*2;
            int offset = (target-stem)*s->bend_side;
            if (offset > 0) mid[y] = (uint8_t)(stem + s->bend_side*(offset < limit ? offset : limit));
        }
    }
}

/* At the far end of a bend, columns cut across the road. Pair the
 * traced outer bank with the first inner bank of the same white run. */
static void ip_follow_bend_2d(const uint8_t *raw, const uint8_t *thresholds)
{
    const cross_straight_t *bend = &s_cross_straight;
    if (s_center_route.count < 6 || !bend->outer_count) return;
    const entry_point_t *start = &s_center_route.points[0];
    const entry_point_t *near = &s_center_route.points[4];
    int direction = bend->bend_side;
    int support = 0, goal_x = 0, goal_y = 0, last_y = 0, last_x = 0;
    for (int i = 0; i < bend->outer_count; ++i) {
        int x = bend->outer_x[i], outer = bend->outer_y[i];
        if ((x-start->x)*direction < 25 || outer > 40) continue;
        int top = outer;
        while (top <= outer+5 && !ep_white(raw, thresholds, x, top)) ++top;
        if (top > outer+5) continue;
        int bottom = top;
        while (bottom <= EP_BOTTOM && ep_white(raw, thresholds, x, bottom)) ++bottom;
        if (bottom > EP_BOTTOM || bottom-top < 13 || bottom-top > 48) continue;
        int y = (top+bottom-1)/2;
        if (support && (abs(y-last_y)>4 || abs(x-last_x)>6)) support = 0;
        last_x = x; last_y = y;
        if (++support >= 5) { goal_x=x; goal_y=y; }
    }
    if (!goal_x || start->y-goal_y < 28) return;
    float rise = start->y-goal_y;
    float span = abs(goal_x-start->x);
    static entry_path_t candidate;
    for (int reach = 0; reach < 3; ++reach) {
        float lead = rise*(.38f+.11f*reach);
        float dx = (near->x-start->x)*lead/(start->y-near->y);
        float control[8] = {start->x,start->y,start->x+dx,start->y-lead,
                            goal_x-direction*span*.40f,goal_y,goal_x,goal_y};
        if (!ep_sample_curve(raw,thresholds,control,&candidate)) continue;
        memcpy(candidate.curve,control,sizeof control);
        candidate.boundary_source = 8;
        candidate.error = s_error;
        /* Use a forward point on the same 2D route for steering preview. */
        for (int i=0;i<candidate.count;++i) {
            if (candidate.points[i].y > 64 && i+1 < candidate.count) continue;
            candidate.preview = (int16_t)(candidate.points[i].x-IP_W/2);
            break;
        }
        s_center_route = candidate;
        s_preview_error = candidate.preview;
        s_preview_valid = 1;
        return;
    }
}

static void ip_apply_cross(uint8_t *left, uint8_t *mid, uint8_t *right)
{
    memset(left, 0, IP_H); memset(right, 0, IP_H);
    memcpy(mid, s_cross_straight.center, IP_H);
    memset(&s_center_route, 0, sizeof s_center_route);
    s_center_route.inferred = 1; s_center_route.boundary_source = 7;
    for (int y = CS_BOTTOM; y >= CS_TOP; --y) {
        if (!mid[y]) continue;
        int l = (int)(s_cross_straight.left[0] + s_cross_straight.left[1]*y + .5f);
        int r = (int)(s_cross_straight.right[0] + s_cross_straight.right[1]*y + .5f);
        /* Fade the measured-to-model offset over the adjacent missing rows.
           Preserve current contour coordinates wherever they were observed. */
        for (int side = 0; side < 2; ++side) {
            const uint8_t *observed = side ? s_cross_straight.measured_right : s_cross_straight.measured_left;
            const float *model = side ? s_cross_straight.right : s_cross_straight.left;
            int *edge = side ? &r : &l;
            for (int d = 0; d <= 12; ++d) {
                int row = y-d;
                if (row < CS_TOP || !observed[row]) row = y+d;
                if (row > CS_BOTTOM || !observed[row]) continue;
                float offset = observed[row] - (model[0]+model[1]*row);
                *edge += (int)roundf(offset * (12-d)/12);
                break;
            }
        }
        if (l > 2 && l < mid[y]) left[y] = (uint8_t)l;
        if (r > mid[y] && r < IP_W-3) right[y] = (uint8_t)r;
        if ((CS_BOTTOM-y)%4) continue;
        if(!ep_path_turn_allowed(&s_center_route,mid[y],y)) {
            memset(mid,0,IP_H); s_center_route.count=0;
            s_valid=s_near_valid=s_preview_valid=0;
            return;
        }
        entry_point_t *point = &s_center_route.points[s_center_route.count++];
        *point = (entry_point_t){mid[y],(uint8_t)y,s_cross_straight.measured_left[y],(uint8_t)y,
                                s_cross_straight.measured_right[y],(uint8_t)y,
                                s_cross_straight.measured[y]};
    }
    s_valid = 1;
    ip_center_errors(mid);
    s_center_route.error=s_error; s_center_route.preview=s_preview_error;
    s_cross_straight.near_error=s_error;
    s_cross_straight.preview_error=s_preview_error;
}

#endif
