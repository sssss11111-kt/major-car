#define RING_ENTRY_IMPLEMENTATION
#include "img_process.h"
#undef RING_ENTRY_IMPLEMENTATION
#include "string.h"

static int ip_abs(int v) { return v < 0 ? -v : v; }

/* ---------- 可调常量 ---------- */
#define IP_W              (MT9V03X_W)
#define IP_H              (MT9V03X_H)
#define IP_BANDS          8          /* 纵向分带数，每带 IP_H/IP_BANDS 行 */
#define IP_BAND_MIN_ROWS  8          /* 少于此行数的带直接沿用相邻带阈值 */
#define IP_LOW_MARGIN_PCT 22         /* 阈值下限 = 带内最暗 + (最亮-最暗)*22/100 */
#define IP_MIN_DR          10        /* 一帧内最亮-最暗低于这个值，视为无有效场景 */

static uint8_t s_scene_ok;
#define IP_TRACK_TOP      18         /* 只处理 y>=IP_TRACK_TOP 的行 */
#define IP_TRACK_BOTTOM   (IP_BODY_MASK_START_ROW-2) /* Row 105 */
#define IP_RUN_MIN        10         /* 白色连通段最少像素 */
#define IP_RUN_GOOD       24
#define IP_JUMP_MAX       22         /* 相邻两行中线允许的最大跳变（像素） */
#define IP_BOTH_SIDE_MIN  3
#define IP_VALID_ROWS_MIN 30
#define IP_HALF_ALPHA     4
#define IP_ERROR_ROW_TOP  (IP_H-40)  /* y=80..105 */
#define IP_ERROR_ROW_BOT  IP_TRACK_BOTTOM
#define IP_ERROR_NEAR     8

/* ---------- 帧间状态 ---------- */
static uint8_t  s_half[IP_H];
static int16_t  s_last_center[IP_H];
static uint8_t  s_row_thr[IP_H];
static uint8_t  s_half_ready;
static int16_t  s_half_bias;
static uint8_t  s_half_bias_ready;
static uint8_t  s_have_error;
static uint8_t  s_valid;
static int16_t  s_error;
static uint8_t  s_usable_rows;
static uint8_t  s_both_rows;
static int16_t  s_preview_error;
static uint8_t  s_preview_valid;
static uint8_t  s_stripe_rows, s_stripe_recovery;
static uint8_t  s_stripe_mask[IP_H];
static uint8_t  s_ring_candidate, s_near_valid, s_ring_preview_valid;
static uint8_t s_diag_near_rows;
static uint8_t s_road_recovered;
static uint8_t  s_ring_evidence, s_ring_corner_row, s_ring_apex_row;
static uint8_t  s_ring_miny, s_ring_maxy;
static int16_t  s_ring_entry_error;
static int      s_ring_guide;
static uint8_t s_entry_mode, s_entry_locked, s_entry_status;
static entry_path_t s_entry_path;
static entry_bridge_guard_t s_bridge_guard;
static uint8_t s_bridge_near_threshold[7];
static float s_odometry_distance, s_odometry_yaw, s_bridge_curvature;
static float s_motion_distance, s_motion_yaw;
static uint32_t s_odometry_tick, s_motion_tick;
static int16_t s_bridge_error, s_bridge_preview;
static uint8_t s_odometry_valid, s_motion_ready, s_bridge_stable;

static void ip_bridge_reset(void) {
    memset(&s_bridge_guard,0,sizeof s_bridge_guard);
    s_bridge_curvature=0;
    s_bridge_stable=s_motion_ready=0;
}
static void ip_bridge_observe(const uint8_t *thresholds) {
    if (!s_odometry_valid) { ip_bridge_reset(); return; }
    if (s_motion_ready) {
        float travel=s_odometry_distance-s_motion_distance;
        uint32_t elapsed=s_odometry_tick-s_motion_tick;
        if (elapsed>20U || travel<0 || travel>0.10f) {
            s_bridge_stable=0;s_motion_ready=0;
        } else if (travel>=0.002f) {
            float curvature=(s_odometry_yaw-s_motion_yaw)/57.2957795f/travel;
            if (fabsf(curvature)>8.0f) s_bridge_stable=0;
            else if (!s_bridge_stable || fabsf(curvature-s_bridge_curvature)>1.5f) {
                s_bridge_curvature=curvature;s_bridge_stable=1;
            } else {
                s_bridge_curvature=0.75f*s_bridge_curvature+0.25f*curvature;
                if (s_bridge_stable<10) ++s_bridge_stable;
            }
            s_motion_ready=0;
        }
    }
    if (!s_motion_ready) {
        s_motion_distance=s_odometry_distance;s_motion_yaw=s_odometry_yaw;
        s_motion_tick=s_odometry_tick;s_motion_ready=1;
    }
    memcpy(s_bridge_near_threshold,thresholds+96,sizeof s_bridge_near_threshold);
    s_bridge_guard.visual_tick=s_odometry_tick;
    s_bridge_guard.distance_m=s_odometry_distance;
    s_bridge_guard.yaw_deg=s_odometry_yaw;
    s_bridge_guard.curvature=s_bridge_curvature;
    s_bridge_guard.stable=(uint8_t)(s_bridge_stable>=3);
    s_bridge_guard.active=0;
    s_bridge_error=s_ring_entry_error;s_bridge_preview=s_preview_error;
}
static int ip_bridge_apply(const uint8_t *raw,const uint8_t *thresholds) {
    float travel, heading_error, correction;
    if (!s_odometry_valid || !entry_bridge_allowed(&s_bridge_guard,s_odometry_distance,s_odometry_yaw,s_odometry_tick)) return 0;
    /* An observed dark area immediately under the car is a hard rejection. */
    for (int y=96; y<=102; ++y)
        for (int x=90; x<=98; ++x)
            if (raw[y*IP_W+x] <= (s_scene_ok ? thresholds[y] : s_bridge_near_threshold[y-96])) return 0;
    travel=s_odometry_distance-s_bridge_guard.distance_m;
    heading_error=s_bridge_guard.yaw_deg+s_bridge_guard.curvature*travel*57.2957795f-s_odometry_yaw;
    correction=heading_error*0.5f;
    if (correction>1.5f) correction=1.5f;
    if (correction< -1.5f) correction= -1.5f;
    s_bridge_guard.active=1;
    s_entry_path.count=0;
    s_entry_path.boundary_source=4;
    s_entry_status=5;
    s_valid=s_near_valid=s_preview_valid=s_ring_preview_valid=1;
    s_error=s_ring_entry_error=(int16_t)(s_bridge_error+correction);
    s_preview_error=s_bridge_preview;
    return 1;
}
static uint8_t  s_ring_l[IP_H], s_ring_r[IP_H], s_ring_mask[IP_H];
/* A candidate can blink for one or two camera frames at the throat. Keep only
 * the last geometry that was measured from this frame, and reuse it briefly
 * while the current white branch still validates every generated point. */
static uint8_t s_ring_hold_l[IP_H], s_ring_hold_r[IP_H], s_ring_hold_mask[IP_H];
static uint8_t s_ring_hold_age, s_ring_hold_miny, s_ring_hold_maxy;
static uint8_t s_ring_hold_corner, s_ring_hold_apex, s_ring_hold_evidence;

static int ip_clamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int ip_half_default(int y)
{
    float offset = (float)(y - IP_WIDTH_CAL_ROW) / (float)(IP_H - 1);
    return (int)(IP_WIDTH_CAL_PX / 2.0f + IP_HALF_PERSPECTIVE_SPAN * offset + 0.5f);
}

static int ip_width_matches(int y, int width)
{
    return ip_abs(width - 2 * ip_half_default(y)) <= IP_WIDTH_TOLERANCE_PX;
}

static int ip_stripe_pattern(const uint8_t *src, int thr, int *pattern_center)
{
    int x = 0, count = 0, first = 0, last = 0, previous_end = -1;
    int min_dark = IP_W, max_dark = 0, min_gap = IP_W, max_gap = 0;
    while (x < IP_W) {
        int start, end, width, gap;
        while (x < IP_W && src[x] > thr) ++x;
        start = x;
        while (x < IP_W && src[x] <= thr) ++x;
        end = x - 1;
        width = end - start + 1;
        if (start <= 2 || end >= IP_W-3 || width < 2 || width > 18) {
            if (count >= 4) break;
            count = 0; previous_end = -1;
            min_dark = min_gap = IP_W; max_dark = max_gap = 0;
            continue;
        }
        gap = previous_end >= 0 ? start-previous_end-1 : 0;
        if (previous_end >= 0 && (gap < 2 || gap > 26)) {
            if (count >= 4) break;
            count = 0; min_dark = min_gap = IP_W; max_dark = max_gap = 0;
        }
        if (!count) first = start;
        else { if (gap < min_gap) min_gap = gap; if (gap > max_gap) max_gap = gap; }
        if (width < min_dark) min_dark = width;
        if (width > max_dark) max_dark = width;
        last = end; previous_end = end; ++count;
    }
    if (count < 4 || last-first+1 < 40 || max_dark > min_dark*3 || max_gap > min_gap*3)
        return 0;
    *pattern_center = (first+last)/2;
    return 1;
}

static int ip_stripe_edges(const uint8_t *src, int thr, int y, int *left, int *right)
{
    int l = 0, r = IP_W-1;
    while (l < IP_W && src[l] <= thr) ++l;
    while (r >= l && src[r] <= thr) --r;
    if (l <= 2 || r >= IP_W-3 || !ip_width_matches(y, r-l+1)) return 0;
    *left = l; *right = r;
    return 1;
}

static int ip_half(int y)
{
    int v;
    if (y < 0 || y >= IP_H) return ip_half_default(0);
    v = s_half[y] ? s_half[y] : ip_half_default(y);
    v += s_half_bias;
    if (v < 12) v = 12;
    if (v > IP_W / 2) v = IP_W / 2;
    return v;
}

static uint8_t ip_band_threshold(const uint32_t *hist, uint32_t total, uint8_t *bright_supported)
{
    int lo = 0, hi = 255;
    uint32_t sum = 0, w = 0;
    double best = -1.0, back = 0.0;
    uint8_t thr = 100;

    if (bright_supported) *bright_supported = 0;
    if (!total) return 100;
    while (lo < 255 && !hist[lo]) ++lo;
    while (hi > lo && !hist[hi]) --hi;
    if (hi <= lo) return (uint8_t)hi;

    for (int i = lo; i <= hi; ++i) sum += (uint32_t)i * hist[i];
    for (int i = lo; i < hi; ++i) {
        w += hist[i];
        if (!w || w == total) continue;
        back += (double)i * hist[i];
        double score = (double)w * (double)(total - w) *
                       (back / w - (double)(sum - (uint32_t)back) / (double)(total - w)) *
                       (back / w - (double)(sum - (uint32_t)back) / (double)(total - w));
        if (score > best) { best = score; thr = (uint8_t)i; }
    }
    {
        int floor_thr = lo + (hi - lo) * IP_LOW_MARGIN_PCT / 100;
        if (thr < floor_thr) thr = (uint8_t)ip_clamp(floor_thr, lo, 254);
    }
    /* White road is the supported bright population, not every pixel above
     * the dark/background Otsu split. A percentile rejects isolated highlights;
     * require a compact bright population before applying this second floor.
     * Keep the floor exposure-relative (no fixed 230/255 camera threshold). */
    {
        uint32_t cumulative = 0, bright_count = 0;
        int p10 = lo, p85 = hi, p95 = hi, got_low = 0, got_bright = 0;
        for (int i = lo; i <= hi; ++i) {
            cumulative += hist[i];
            if (!got_low && cumulative * 100U >= total * 10U) {p10=i;got_low=1;}
            if (!got_bright && cumulative * 100U >= total * 85U) {p85=i;got_bright=1;}
            if (cumulative * 100U >= total * 95U) {p95=i;break;}
        }
        for (int i = ip_clamp(p85-6,lo,hi); i <= hi; ++i) bright_count += hist[i];
        if (p85-p10 >= 24 && bright_count * 100U >= total * 15U && hi-p85 <= 12) {
            if (bright_supported) *bright_supported = 1;
            int margin = ip_clamp((p85-p10)/6,8,32);
            int white_floor = p85-margin;
            if (thr < white_floor) thr = (uint8_t)white_floor;
        } else if (p95 - p10 >= 24 && hi - p95 <= ip_clamp((p95 - p10) / 8, 4, 12)) {
            /* A narrow road can occupy only 5..15% of a far image band. Its
             * compact bright population still supports an exposure-relative
             * floor, including when the road itself is shaded. */
            int margin = ip_clamp((p95 - p10) / 6, 8, 32);
            int white_floor = p95 - margin;
            if (bright_supported) *bright_supported = 1;
            if (thr < white_floor) thr = (uint8_t)white_floor;
        }
    }
    return thr;
}

static int ip_row_run(const uint8_t *bin, const uint8_t *src, int thr,
                      int pred_left, int expect_hw, int ref_center,
                      int *left, int *right)
{
    int best_l = -1, best_r = -1, best_score = -1000000, best_d = 0x7fff;
    int max_wide = (expect_hw * 2) + (expect_hw / 2) + 12;
    int x = 0;
    (void)thr;

    while (x < IP_W) {
        int l, r, len, score, d, wide, dev;
        while (x < IP_W && !bin[x]) ++x;
        if (x >= IP_W) break;
        l = x;
        while (x < IP_W && bin[x]) ++x;
        r = x - 1;
        len = r - l + 1;
        if (len < IP_RUN_MIN) continue;

        if (len > max_wide) {
            int cut = -1, best = 24, side = 0;
            for (int k = l + 10; k < r - 6; ++k) {
                int dl = (int)src[k] - (int)src[k - 2];
                int dr = (int)src[k] - (int)src[k + 2];
                int d0 = dl > dr ? dl : dr;
                if (d0 > best) { best = d0; cut = k; side = (dl >= dr) ? 1 : 2; }
            }
            if (cut > 0) {
                if (side == 1) { l = cut; } else { r = cut; }
                len = r - l + 1;
                if (len < IP_RUN_MIN) continue;
            }
        }
        wide = (len > 150) ? 150 : len;
        d    = ((l + r) / 2) - ref_center;
        if (d < 0) d = -d;
        dev = (wide / 2) - expect_hw;
        if (dev < 0) dev = -dev;
        score = wide * 4 - d - dev * 5;
        if (pred_left >= 0) {
            int e = l - pred_left;
            if (e < 0) e = -e;
            score -= e * 3;
        }
        if (score > best_score || (score == best_score && d < best_d)) {
            best_score = score; best_d = d; best_l = l; best_r = r;
        }
    }
    if (best_l < 0) return 0;
    *left = best_l;
    *right = best_r;
    return 1;
}


/* Close a side opening only while approaching on a measured straight corridor.
 * Coordinates are mirrored for a left-side opening. Both endpoints and the
 * opposite edge come from this frame; synthesized edges never teach width. */
static uint8_t s_approach_edge[IP_H], s_approach_side[IP_H];
static uint8_t s_approach_active, s_approach_width_side, s_approach_width_age;
static float s_approach_width_bottom, s_approach_width_slope;
static uint32_t s_approach_width_tick;

/* Learn perspective from measured pairs below the opening, never from repair. */
static void ip_approach_learn(const int *edge, const int *opposite, int bottom, int side)
{
    int first = bottom + 3;
    float sum_y = 0, sum_w = 0, sum_yy = 0, sum_yw = 0;
    int count = IP_TRACK_BOTTOM - first + 1;
    if (count < 12) return;
    for (int y = first; y <= IP_TRACK_BOTTOM; ++y) {
        float dy = (float)(y - IP_TRACK_BOTTOM);
        int width = edge[y] - opposite[y];
        if (opposite[y] <= 2 || edge[y] >= IP_W - 3 || width < IP_RUN_GOOD) return;
        sum_y += dy; sum_w += width; sum_yy += dy * dy; sum_yw += dy * width;
    }
    float slope = (count * sum_yw - sum_y * sum_w) / (count * sum_yy - sum_y * sum_y);
    float width_bottom = (sum_w - slope * sum_y) / count;
    if (slope < 0.3f || slope > 2.0f) return;
    for (int y = first; y <= IP_TRACK_BOTTOM; ++y)
        if (fabsf(edge[y] - opposite[y] - width_bottom - slope * (y - IP_TRACK_BOTTOM)) > 3.0f) return;
    s_approach_width_bottom = width_bottom;
    s_approach_width_slope = slope;
    s_approach_width_side = (uint8_t)side;
    s_approach_width_age = 0;
    s_approach_width_tick = s_odometry_tick;
}

/* Continue using this frame's opposite edge and far island anchor after the
 * near endpoint leaves the image. The entire reconstructed corridor is checked. */
static int ip_approach_continue(const uint8_t *raw, const uint8_t *thr,
                                const int *edge, const int *opposite, int side)
{
    int top = -1;
    float anchor_delta = 0;
    if (side != s_approach_width_side || s_approach_width_age > 30 ||
        s_odometry_tick - s_approach_width_tick > 250U) return 0;
    if (s_ring_guide && side != (s_ring_guide < 0 ? 2 : 1)) return 0;
    for (int y = IP_TRACK_BOTTOM - 6; y >= IP_TRACK_TOP + 2; --y) {
        float width = s_approach_width_bottom + s_approach_width_slope * (y - IP_TRACK_BOTTOM);
        float delta = edge[y] - opposite[y] - width;
        if (opposite[y] <= 2 || edge[y] < 0 || edge[y] >= IP_W - 3 || fabsf(delta) > 6.0f ||
            edge[y - 1] < 0 || edge[y - 2] < 0 || ip_abs(edge[y] - edge[y - 1]) > 6 ||
            ip_abs(edge[y - 1] - edge[y - 2]) > 6) continue;
        int x = edge[y] + 1, dark = 0;
        while (x < IP_W && raw[y * IP_W + (side == 2 ? x : IP_W - 1 - x)] <= thr[y]) { ++dark; ++x; }
        /* A previously observed island may now be clipped at the side.
         * Require an opening below the anchor, excluding the near outer edge. */
        float lower_width = s_approach_width_bottom + s_approach_width_slope * (y + 6 - IP_TRACK_BOTTOM);
        if (dark < 6 || edge[y + 6] - opposite[y + 6] - lower_width < 8.0f) continue;
        top = y; anchor_delta = delta;
        break;
    }
    if (top < 0) return 0;
    int missing = 0;
    for (int y = top; y <= IP_TRACK_BOTTOM; ++y) {
        float width = s_approach_width_bottom + s_approach_width_slope * (y - IP_TRACK_BOTTOM);
        int repair = (int)(opposite[y] + width + anchor_delta * (IP_TRACK_BOTTOM - y) / (IP_TRACK_BOTTOM - top) + 0.5f);
        int opposite_fit = opposite[top] + (opposite[IP_TRACK_BOTTOM] - opposite[top]) * (y - top) / (IP_TRACK_BOTTOM - top);
        if (opposite[y] <= 2 || ip_abs(opposite[y] - opposite_fit) > 3 || repair >= IP_W - 2 ||
            repair - opposite[y] < IP_RUN_GOOD || edge[y] < repair - 8) return 0;
        if (repair > edge[y]) repair = edge[y];
        if (edge[y] > repair + 8) ++missing;
        for (int x = opposite[y] + 1; x < repair; ++x)
            if (raw[y * IP_W + (side == 2 ? x : IP_W - 1 - x)] <= thr[y]) return 0;
    }
    if (missing < 6) return 0;
    for (int y = top + 1; y <= IP_TRACK_BOTTOM; ++y) {
        float width = s_approach_width_bottom + s_approach_width_slope * (y - IP_TRACK_BOTTOM);
        int repair = (int)(opposite[y] + width + anchor_delta * (IP_TRACK_BOTTOM - y) / (IP_TRACK_BOTTOM - top) + 0.5f);
        if (repair > edge[y]) repair = edge[y];
        s_approach_edge[y] = (uint8_t)(side == 2 ? repair : IP_W - 1 - repair);
        s_approach_side[y] = (uint8_t)side;
    }
    s_approach_active = 1;
    return 1;
}
static void ip_approach_bridge(const uint8_t *raw, const uint8_t *thr)
{
    int left[IP_H], right[IP_H];
    memset(s_approach_side, 0, sizeof s_approach_side);
    s_approach_active = 0;
    if (s_approach_width_side && s_approach_width_age < 255) ++s_approach_width_age;
    if (s_entry_locked || s_entry_mode > 1 || s_stripe_recovery || !s_scene_ok) {
        s_approach_width_side = 0;
        return;
    }
    for (int y = IP_TRACK_TOP; y <= IP_TRACK_BOTTOM; ++y) {
        const uint8_t *row = raw + y * IP_W;
        int l = IP_W / 2, r = l;
        left[y] = right[y] = -1;
        if (row[l] <= thr[y]) continue;
        while (l > 0 && row[l - 1] > thr[y]) --l;
        while (r < IP_W - 1 && row[r + 1] > thr[y]) ++r;
        left[y] = l; right[y] = r;
    }
    for (int side = 1; side <= 2; ++side) {
        int edge[IP_H], opposite[IP_H];
        for (int y = IP_TRACK_TOP; y <= IP_TRACK_BOTTOM; ++y) {
            edge[y] = left[y] < 0 ? -1 : (side == 2 ? right[y] : IP_W - 1 - left[y]);
            opposite[y] = left[y] < 0 ? -1 : (side == 2 ? left[y] : IP_W - 1 - right[y]);
        }
        if (s_ring_guide && side != (s_ring_guide < 0 ? 2 : 1)) continue;
        for (int bottom = IP_TRACK_BOTTOM - 9; bottom >= IP_TRACK_TOP + 8; --bottom) {
            int top = -1, supported = 1;
            if (edge[bottom] < 0 || edge[bottom] >= IP_W - 3 ||
                edge[bottom - 1] - edge[bottom] < 8) continue;
            for (int y = bottom; y <= bottom + 9; ++y) {
                if (edge[y] < 0 || edge[y] >= IP_W - 3 || opposite[y] <= 2 ||
                    (y > bottom && (ip_abs(edge[y] - edge[y - 1]) > 2 ||
                                    ip_abs(opposite[y] - opposite[y - 1]) > 2))) supported = 0;
            }
            if (!supported || ip_abs(edge[bottom + 6] + opposite[bottom + 6] -
                                     edge[bottom] - opposite[bottom]) > 8) continue;
            for (int y = bottom - 5; y >= IP_TRACK_TOP + 2 && bottom - y <= 65; --y) {
                /* Estimate the approach tangent below the corner tip. */
                int predicted = edge[bottom] + (edge[bottom + 9] - edge[bottom + 3]) * (y - bottom) / 6;
                if (edge[y] < 0 || edge[y] >= IP_W - 3 ||
                    ip_abs(edge[y] - predicted) > 8 || edge[y - 1] < 0 || edge[y - 2] < 0 ||
                    ip_abs(edge[y - 1] - edge[y]) > 4 || ip_abs(edge[y - 2] - edge[y - 1]) > 4) continue;
                /* The far endpoint must border an island with visible road
                 * beyond it, not merely an arbitrary outside bend. */
                int x = edge[y] + 1, dark = 0, white = 0;
                while (x < IP_W && raw[y * IP_W + (side == 2 ? x : IP_W - 1 - x)] <= thr[y]) { ++dark; ++x; }
                while (x < IP_W && raw[y * IP_W + (side == 2 ? x : IP_W - 1 - x)] > thr[y]) { ++white; ++x; }
                if (dark < 6 || white < 5) continue;
                top = y;
                break;
            }
            if (top < 0) continue;
            for (int y = top; y <= bottom; ++y) {
                int repair = edge[top] + (edge[bottom] - edge[top]) * (y - top) / (bottom - top);
                int opposite_fit = opposite[top] + (opposite[bottom] - opposite[top]) * (y - top) / (bottom - top);
                if (opposite[y] <= 2 || ip_abs(opposite[y] - opposite_fit) > 3 ||
                    repair - opposite[y] < IP_RUN_GOOD || edge[y] < repair - 2) { supported = 0; break; }
                /* Reject if any repaired corridor would cross a current dark pixel. */
                for (int x = opposite[y] + 1; x < repair; ++x)
                    if (raw[y * IP_W + (side == 2 ? x : IP_W - 1 - x)] <= thr[y]) supported = 0;
            }
            if (!supported) continue;
            for (int y = top + 1; y < bottom; ++y) {
                int repair = edge[top] + (edge[bottom] - edge[top]) * (y - top) / (bottom - top);
                s_approach_edge[y] = (uint8_t)(side == 2 ? repair : IP_W - 1 - repair);
                s_approach_side[y] = (uint8_t)side;
            }
            s_approach_active = 1;
            ip_approach_learn(edge, opposite, bottom, side);
            return;
        }
        if (ip_approach_continue(raw, thr, edge, opposite, side)) return;
    }
}

#define IP_ROI_PIXELS (IP_W*(IP_TRACK_BOTTOM-IP_TRACK_TOP+1))
static uint8_t s_exterior[IP_H][(IP_W+7)/8];
static uint16_t s_exterior_queue[IP_ROI_PIXELS];
static void ip_exterior_add(const uint8_t *raw,const uint8_t *thr,int x,int y,int *tail)
{
    uint8_t bit=(uint8_t)(1U<<(x&7));
    if((s_exterior[y][x>>3]&bit) || raw[y*IP_W+x]>thr[y])return;
    s_exterior[y][x>>3]|=bit;
    s_exterior_queue[(*tail)++]=(uint16_t)(y*IP_W+x);
}
static void ip_mark_exterior(const uint8_t *raw,const uint8_t *thr)
{
    int head=0,tail=0;
    memset(s_exterior,0,sizeof s_exterior);
    for(int x=0;x<IP_W;++x) {
        ip_exterior_add(raw,thr,x,IP_TRACK_TOP,&tail);
        ip_exterior_add(raw,thr,x,IP_TRACK_BOTTOM,&tail);
    }
    for(int y=IP_TRACK_TOP;y<=IP_TRACK_BOTTOM;++y) {
        ip_exterior_add(raw,thr,0,y,&tail);
        ip_exterior_add(raw,thr,IP_W-1,y,&tail);
    }
    while(head<tail) {
        int idx=s_exterior_queue[head++],x=idx%IP_W,y=idx/IP_W;
        for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
            int xx=x+dx,yy=y+dy;
            if(xx>=0 && xx<IP_W && yy>=IP_TRACK_TOP && yy<=IP_TRACK_BOTTOM)
                ip_exterior_add(raw,thr,xx,yy,&tail);
        }
    }
}

/* Compute auxiliary geometric evidences (corners, outer boundary continuity,
 * and track width constriction/expansion) for an island spanning [miny, maxy]
 * with branch on `side` (-1 = left branch / right island; +1 = right branch / left island). */
static uint8_t ip_eval_auxiliary_evidence(const uint8_t *raw, const uint8_t *thr,
                                          int miny, int maxy, int side,
                                          uint8_t *out_corner_row, uint8_t *out_apex_row)
{
    uint8_t ev = 0;
    int apex_y = (miny + maxy) / 2;
    int apex_x = (side < 0) ? IP_W : -1;
    int outer_cont = 0, best_outer_cont = 0, last_outer = -1;
    int constrict_rows = 0, expand_rows = 0;

    for (int y = miny; y <= maxy; ++y) {
        const uint8_t *src = raw + (unsigned)y * IP_W;
        int inner_x = (side < 0) ? (int)s_ring_l[y] : (int)s_ring_r[y];
        int outer_x = inner_x;
        if (side < 0) {
            if (inner_x < apex_x) { apex_x = inner_x; apex_y = y; }
            while (outer_x > 0 && src[outer_x - 1] > thr[y]) --outer_x;
            if (outer_x > 2) {
                if (last_outer >= 0 && ip_abs(outer_x - last_outer) <= 5 &&
                    outer_x >= last_outer - 1) ++outer_cont;
                else outer_cont = 1;
                last_outer = outer_x;
                if (outer_cont > best_outer_cont) best_outer_cont = outer_cont;
            }
            if (inner_x - outer_x + 1 <= 2 * ip_half_default(y) - 12 &&
                inner_x - outer_x + 1 >= IP_RUN_MIN) ++constrict_rows;
        } else {
            if (inner_x > apex_x) { apex_x = inner_x; apex_y = y; }
            while (outer_x < IP_W - 1 && src[outer_x + 1] > thr[y]) ++outer_x;
            if (outer_x < IP_W - 3) {
                if (last_outer >= 0 && ip_abs(outer_x - last_outer) <= 5 &&
                    outer_x <= last_outer + 1) ++outer_cont;
                else outer_cont = 1;
                last_outer = outer_x;
                if (outer_cont > best_outer_cont) best_outer_cont = outer_cont;
            }
            if (outer_x - inner_x + 1 <= 2 * ip_half_default(y) - 12 &&
                outer_x - inner_x + 1 >= IP_RUN_MIN) ++constrict_rows;
        }
    }
    *out_apex_row = (uint8_t)apex_y;
    *out_corner_row = (uint8_t)maxy;

    /* 1. Arc apex inflection: inner boundary protrudes inward relative to both ends. */
    {
        int top_x = (side < 0) ? (int)s_ring_l[miny] : (int)s_ring_r[miny];
        int bot_x = (side < 0) ? (int)s_ring_l[maxy] : (int)s_ring_r[maxy];
        int d_top = (side < 0) ? (top_x - apex_x) : (apex_x - top_x);
        int d_bot = (side < 0) ? (bot_x - apex_x) : (apex_x - bot_x);
        if (d_top >= 6 && d_bot >= 6) ev |= IP_RING_EV_CORNER_APEX;
    }

    /* 2. Lower bifurcation corner + throat expansion below maxy. */
    for (int y = maxy + 1; y <= IP_TRACK_BOTTOM && y <= maxy + 12; ++y) {
        const uint8_t *src = raw + (unsigned)y * IP_W;
        int l = (side < 0) ? (int)s_ring_l[maxy] : (int)s_ring_r[maxy];
        int r = l;
        while (l > 0 && src[l - 1] > thr[y]) --l;
        while (r < IP_W - 1 && src[r + 1] > thr[y]) ++r;
        if (r - l + 1 >= 2 * ip_half_default(y) + IP_WIDTH_TOLERANCE_PX) ++expand_rows;
        if (side < 0 && r - (int)s_ring_l[maxy] >= 12) ev |= IP_RING_EV_CORNER_LOW;
        if (side > 0 && (int)s_ring_r[maxy] - l >= 12) ev |= IP_RING_EV_CORNER_LOW;
    }

    if (best_outer_cont >= 12) ev |= IP_RING_EV_OUTER_CONT;
    if (constrict_rows >= 8 || (constrict_rows >= 4 && expand_rows >= 2))
        ev |= IP_RING_EV_WIDTH_ANOMALY;
    return ev;
}

/* Side-clipped island detector & guided branch segmenter.
 * Separates (A) unguided candidate confirmation (strict closed top & bottom)
 * from (B) guided branch segmentation when s_ring_guide != 0 (allowing the
 * island to cross IP_TRACK_BOTTOM as the car enters/orbits the ring). */
static int ip_side_island(const uint8_t *raw,const uint8_t *thr,int *out_guided_seg)
{
    int found_candidate=0;
    *out_guided_seg=0;
    memset(s_exterior,0,sizeof s_exterior);
    for(int sy=IP_TRACK_TOP;sy<=IP_TRACK_BOTTOM;++sy)for(int sx=0;sx<IP_W;++sx) {
        int head=0,tail=0,minx=IP_W,maxx=0,miny=IP_H,maxy=0;
        int row_l[IP_H],row_r[IP_H];
        if((s_exterior[sy][sx>>3]&(1U<<(sx&7))) || raw[sy*IP_W+sx]>thr[sy])continue;
        ip_exterior_add(raw,thr,sx,sy,&tail);
        while(head<tail) {
            int idx=s_exterior_queue[head++],x=idx%IP_W,y=idx/IP_W;
            if(x<minx)minx=x;if(x>maxx)maxx=x;
            if(y<miny)miny=y;if(y>maxy)maxy=y;
            for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
                int xx=x+dx,yy=y+dy;
                if(xx>=0 && xx<IP_W && yy>=IP_TRACK_TOP && yy<=IP_TRACK_BOTTOM)
                    ip_exterior_add(raw,thr,xx,yy,&tail);
            }
        }
        {
            int h=maxy-miny+1,w=maxx-minx+1,side=(minx==0 ? 1:-1);
            int edge_rows=0,top_width=0,bottom_width=0,wide=0,branch_rows=0;
            if((minx!=0 && maxx!=IP_W-1) || (minx==0 && maxx==IP_W-1))continue;
            if(s_ring_guide && side!=s_ring_guide)continue;
            if(h<20 || w<20 || w>120 || tail<320 || tail*3<h*w)continue;
            for(int y=miny;y<=maxy;++y){row_l[y]=IP_W;row_r[y]=-1;}
            for(int i=0;i<tail;++i) {
                int x=s_exterior_queue[i]%IP_W,y=s_exterior_queue[i]/IP_W;
                if(x<row_l[y])row_l[y]=x;if(x>row_r[y])row_r[y]=x;
            }
            for(int y=miny;y<=maxy;++y) {
                int width=row_r[y]-row_l[y]+1;
                if(width>wide)wide=width;
                if(y<miny+3 && width>top_width)top_width=width;
                if(y>maxy-3 && width>bottom_width)bottom_width=width;
                if(row_l[y]==0 || row_r[y]==IP_W-1)++edge_rows;
                int x=side<0 ? row_l[y]-1:row_r[y]+1,count=0;
                while(x>=0 && x<IP_W && raw[y*IP_W+x]>thr[y]){++count;x+=side;}
                if(count>=IP_RUN_MIN)++branch_rows;
            }
            /* Strict candidate gate: must not touch top/bottom ROI, both ends taper. */
            if(miny>=IP_TRACK_TOP+2 && maxy<=IP_TRACK_BOTTOM-3 &&
               h>=24 && w>=24 && w<=110 && tail>=400 &&
               edge_rows>=6 && branch_rows>=h-3 &&
               wide-top_width>=12 && wide-bottom_width>=12) {
                uint8_t c_row=0, a_row=0, aux_ev=0;
                memset(s_ring_mask,0,sizeof s_ring_mask);
                for(int y=miny;y<=maxy;++y) {
                    s_ring_mask[y]=(uint8_t)(side<0 ? 1:2);
                    s_ring_l[y]=(uint8_t)(row_l[y]>0 ? row_l[y]-1:0);
                    s_ring_r[y]=(uint8_t)(row_r[y]<IP_W-1 ? row_r[y]+1:IP_W-1);
                }
                aux_ev = ip_eval_auxiliary_evidence(raw, thr, miny, maxy, side, &c_row, &a_row);
                /* Allow miny==IP_TRACK_TOP+2 only when auxiliary corner + width + continuity agree. */
                if(miny>=IP_TRACK_TOP+3 || (aux_ev & (IP_RING_EV_CORNER_APEX|IP_RING_EV_WIDTH_ANOMALY))) {
                    s_ring_evidence = (uint8_t)(IP_RING_EV_SIDE_ISLAND | aux_ev);
                    s_ring_corner_row = c_row;
                    s_ring_apex_row = a_row;
                    s_ring_miny = (uint8_t)miny;
                    s_ring_maxy = (uint8_t)maxy;
                    found_candidate=1;
                    *out_guided_seg=1;
                } else {
                    memset(s_ring_mask,0,sizeof s_ring_mask);
                }
            } else if(!found_candidate && s_ring_guide && side==s_ring_guide &&
                      !(miny==IP_TRACK_TOP && maxy==IP_TRACK_BOTTOM) &&
                      /* A top-clipped far background bank is not the current
                       * inner edge. Require near-field support when its upper
                       * closure is outside the ROI (feedback frames 130/137). */
                      (miny>IP_TRACK_TOP || maxy>=IP_ERROR_ROW_TOP+5) &&
                      edge_rows>=6 && branch_rows>=h-4 &&
                      ((miny>=IP_TRACK_TOP+2 && wide-top_width>=10) ||
                       (maxy<=IP_TRACK_BOTTOM-2 && wide-bottom_width>=10))) {
                /* Guided branch continuation: car is already in RB_ENTER/RB_ORBIT and
                 * the island touches one vertical ROI border (e.g. maxy==105).
                 * Keep branch segmentation alive without asserting a new candidate. */
                uint8_t c_row=0, a_row=0;
                memset(s_ring_mask,0,sizeof s_ring_mask);
                for(int y=miny;y<=maxy;++y) {
                    s_ring_mask[y]=(uint8_t)(side<0 ? 1:2);
                    s_ring_l[y]=(uint8_t)(row_l[y]>0 ? row_l[y]-1:0);
                    s_ring_r[y]=(uint8_t)(row_r[y]<IP_W-1 ? row_r[y]+1:IP_W-1);
                }
                (void)ip_eval_auxiliary_evidence(raw, thr, miny, maxy, side, &c_row, &a_row);
                s_ring_corner_row = c_row;
                s_ring_apex_row = a_row;
                s_ring_miny = (uint8_t)miny;
                s_ring_maxy = (uint8_t)maxy;
                *out_guided_seg=1;
            }
        }
    }
    return found_candidate;
}

static void ip_island(const uint8_t *raw,const uint8_t *thresholds)
{
    int chain=0,best=0,last_l=-1,last_r=-1,min_gap=IP_W,max_gap=0,cap=0,found=0;
    int full_miny=IP_H, full_maxy=0;
    s_ring_candidate=0;
    s_ring_evidence=0;
    s_ring_corner_row=0;
    s_ring_apex_row=0;
    s_ring_miny=0;
    s_ring_maxy=0;
    memset(s_ring_mask,0,sizeof s_ring_mask);
    if(!s_scene_ok || s_stripe_recovery)return;
    for(int y=IP_TRACK_TOP;y<=IP_TRACK_BOTTOM;++y) {
        const uint8_t *src=raw+y*IP_W;
        int starts[3],ends[3],n=0,x=0;
        while(x<IP_W) {
            int l,r;
            while(x<IP_W && src[x]<=thresholds[y])++x;
            l=x;
            while(x<IP_W && src[x]>thresholds[y])++x;
            r=x-1;
            if(r-l+1>=10) {
                if(n<3){starts[n]=l;ends[n]=r;}
                ++n;
            }
        }
        if(n==2 && starts[1]-ends[0]-1>=10 && starts[1]-ends[0]-1<=100 &&
           ends[0]>12 && starts[1]<IP_W-13) {
            int l=ends[0]+1,r=starts[1]-1;
            if(last_l>=0 && ip_abs(l-last_l)<=8 && ip_abs(r-last_r)<=8)++chain;
            else {
                int cx=(l+r)/2;
                chain=1;min_gap=IP_W;max_gap=0;cap=0;
                for(int yy=y-1;yy>=y-8 && yy>=IP_TRACK_TOP+2;--yy) {
                    if(raw[yy*IP_W+cx]>thresholds[yy] &&
                       raw[(yy-1)*IP_W+cx]>thresholds[yy-1] &&
                       raw[(yy-2)*IP_W+cx]>thresholds[yy-2]){cap=1;break;}
                }
            }
            last_l=l;last_r=r;
            if(r-l+1<min_gap)min_gap=r-l+1;
            if(r-l+1>max_gap)max_gap=r-l+1;
            if(chain>=8 && max_gap-min_gap>=8 && cap)found=1;
            if(chain>best)best=chain;
            s_ring_mask[y]=3;s_ring_l[y]=(uint8_t)ends[0];s_ring_r[y]=(uint8_t)starts[1];
            if(y<full_miny)full_miny=y;
            if(y>full_maxy)full_maxy=y;
        }else{chain=0;last_l=last_r=-1;}
    }
    if(found) {
        int enclosed_chain=0,enclosed_best=0,enclosed_area=0;
        int chain_area=0,chain_width=0,best_width=0,best_top=0,best_bottom=0;
        ip_mark_exterior(raw,thresholds);
        for(int y=IP_TRACK_TOP;y<=IP_TRACK_BOTTOM;++y) {
            int bounded=s_ring_mask[y],dark=0;
            if(bounded)for(int x=s_ring_l[y]+1;x<s_ring_r[y];++x) {
                if(raw[y*IP_W+x]<=thresholds[y]) {
                    ++dark;
                    if(s_exterior[y][x>>3]&(1U<<(x&7))){bounded=0;break;}
                }
            }
            if(bounded && dark>=10) {
                ++enclosed_chain;chain_area+=dark;
                if(dark>chain_width)chain_width=dark;
                if(enclosed_chain>enclosed_best) {
                    enclosed_best=enclosed_chain;enclosed_area=chain_area;
                    best_width=chain_width;best_top=y-enclosed_chain+1;best_bottom=y;
                }
            }else {enclosed_chain=0;chain_area=0;chain_width=0;}
        }
        /* Use the enclosed component's own extent, not every two-run row in
         * the image. Small bricks/marks also have white on both sides: require
         * a sustained obstruction at road scale, consistent with side islands.
         * One third of the calibrated road width is a confirmation gate, not
         * a claim to classify every obstacle or estimate physical dimensions. */
        found=enclosed_best>=24 && enclosed_area>=400 && best_width>=24 &&
              best_width>=2*ip_half_default((best_top+best_bottom)/2)/3;
        if(found) {
            full_miny=best_top;full_maxy=best_bottom;
            for(int y=IP_TRACK_TOP;y<=IP_TRACK_BOTTOM;++y)
                if(y<best_top || y>best_bottom)s_ring_mask[y]=0;
        }
        if(found && full_miny<=full_maxy) {
            uint8_t c_row=0, a_row=0;
            int eval_side = s_ring_guide ? s_ring_guide : -1;
            uint8_t aux_ev = ip_eval_auxiliary_evidence(raw, thresholds, full_miny, full_maxy, eval_side, &c_row, &a_row);
            s_ring_evidence = (uint8_t)(IP_RING_EV_FULL_ISLAND | aux_ev);
            s_ring_corner_row = c_row;
            s_ring_apex_row = a_row;
            s_ring_miny = (uint8_t)full_miny;
            s_ring_maxy = (uint8_t)full_maxy;
        }
    }
    if(!found) {
        int guided_seg=0;
        found=ip_side_island(raw,thresholds,&guided_seg);
        if(!found && !guided_seg) {
            if(s_ring_guide && best>=8 && full_miny<=full_maxy &&
               (full_miny>IP_TRACK_TOP || full_maxy>=IP_ERROR_ROW_TOP+5)) {
                uint8_t c_row=0, a_row=0;
                (void)ip_eval_auxiliary_evidence(raw, thresholds, full_miny, full_maxy, s_ring_guide, &c_row, &a_row);
                s_ring_corner_row = c_row;
                s_ring_apex_row = a_row;
                s_ring_miny = (uint8_t)full_miny;
                s_ring_maxy = (uint8_t)full_maxy;
            } else {
                memset(s_ring_mask,0,sizeof s_ring_mask);
            }
        }
    }
    /* Preserve only a short-lived, current-frame-measured branch geometry. A
     * missing candidate therefore cannot erase the entry path immediately, but
     * the target pixels are still checked against this frame below. */
    if (s_ring_miny && s_ring_maxy >= s_ring_miny) {
        memcpy(s_ring_hold_l, s_ring_l, sizeof s_ring_l);
        memcpy(s_ring_hold_r, s_ring_r, sizeof s_ring_r);
        memcpy(s_ring_hold_mask, s_ring_mask, sizeof s_ring_mask);
        s_ring_hold_miny=s_ring_miny; s_ring_hold_maxy=s_ring_maxy;
        s_ring_hold_corner=s_ring_corner_row; s_ring_hold_apex=s_ring_apex_row;
        s_ring_hold_evidence=s_ring_evidence; s_ring_hold_age=0;
    } else if (s_ring_guide && s_ring_hold_age < 3 &&
               s_ring_hold_miny && s_ring_hold_maxy >= s_ring_hold_miny) {
        ++s_ring_hold_age;
        memcpy(s_ring_l, s_ring_hold_l, sizeof s_ring_l);
        memcpy(s_ring_r, s_ring_hold_r, sizeof s_ring_r);
        memcpy(s_ring_mask, s_ring_hold_mask, sizeof s_ring_mask);
        s_ring_miny=s_ring_hold_miny; s_ring_maxy=s_ring_hold_maxy;
        s_ring_corner_row=s_ring_hold_corner; s_ring_apex_row=s_ring_hold_apex;
        s_ring_evidence=s_ring_hold_evidence;
    } else if (!s_ring_guide) {
        s_ring_hold_age=0;
    }
    s_ring_candidate=(uint8_t)found;
}

void binaryzation_process(unsigned char *_img, const unsigned short _rows,
                          const unsigned short _cols, const unsigned int _threshold_value)
{
    unsigned int size = (unsigned int)_rows * _cols;
    for (unsigned int i = 0; i < size; ++i)
        _img[i] = (_img[i] > _threshold_value) ? 255 : 0;
}

void get_hist_gram(unsigned char *_img, const unsigned short _rows,
                   const unsigned short _cols, short *_hist_gram)
{
    memset(_hist_gram, 0, 256 * sizeof(short));
    for (unsigned int i = 0; i < (unsigned int)_rows * _cols; ++i)
        ++_hist_gram[_img[i]];
}

unsigned char get_threshold_otsu(const short *_hist_gram)
{
    uint32_t hist[256];
    uint32_t total = 0;
    for (int i = 0; i < 256; ++i) { hist[i] = (uint32_t)(_hist_gram[i] < 0 ? 0 : _hist_gram[i]); total += hist[i]; }
    return ip_band_threshold(hist, total, 0);
}

void img_process_reset(void)
{
    memset(s_half, 0, sizeof s_half);
    memset(s_last_center, 0xff, sizeof s_last_center);
    s_half_ready = 0;
    s_half_bias = 0;
    s_half_bias_ready = 0;
    s_have_error = 0;
    s_scene_ok = 0;
    s_valid = 0;
    s_error = 0;
    s_preview_error = 0;
    s_preview_valid = 0;
    s_usable_rows = 0;
    s_both_rows = 0;
    s_stripe_rows = s_stripe_recovery = 0;
    s_ring_candidate = s_near_valid = s_ring_preview_valid = 0;
    s_road_recovered = 0;
    s_ring_evidence = s_ring_corner_row = s_ring_apex_row = 0;
    s_ring_miny = s_ring_maxy = 0;
    s_ring_entry_error = 0;
    s_ring_guide = 0;
    s_entry_mode = s_entry_locked = s_entry_status = 0;
    memset(s_approach_side, 0, sizeof s_approach_side);
    s_approach_active = s_approach_width_side = s_approach_width_age = 0;
    ip_bridge_reset();s_odometry_valid=0;
    memset(&s_entry_path, 0, sizeof s_entry_path);
    ep_reset();
    memset(s_ring_mask, 0, sizeof s_ring_mask);
    memset(s_ring_hold_l, 0, sizeof s_ring_hold_l);
    memset(s_ring_hold_r, 0, sizeof s_ring_hold_r);
    memset(s_ring_hold_mask, 0, sizeof s_ring_hold_mask);
    s_ring_hold_age=s_ring_hold_miny=s_ring_hold_maxy=0;
    s_ring_hold_corner=s_ring_hold_apex=s_ring_hold_evidence=0;
    memset(s_stripe_mask, 0, sizeof s_stripe_mask);
}

uint8_t img_get_road_recovered(void) { return s_valid && s_near_valid && s_road_recovered; }
void img_set_roundabout_entry(uint8_t enabled) {
    if (!enabled) {
        s_entry_locked = s_entry_status = 0;
        s_entry_path.count = 0;
        ep_reset();
        ip_bridge_reset();
    }
    s_entry_mode = enabled;
}
void img_set_entry_odometry(float distance_m,float yaw_deg,uint32_t tick,uint8_t valid) {
    s_odometry_distance=distance_m;s_odometry_yaw=yaw_deg;
    s_odometry_tick=tick;s_odometry_valid=valid;
}
const entry_bridge_guard_t *img_get_entry_bridge_guard(void) { return &s_bridge_guard; }
const entry_path_t *img_get_entry_path(void) { return &s_entry_path; }
uint8_t img_get_entry_status(void) { return s_entry_status; }
uint8_t img_get_roundabout_candidate(void) { return s_ring_candidate; }
uint8_t img_get_roundabout_evidence(void) { return s_ring_evidence; }
uint8_t img_get_roundabout_corner_row(void) { return s_ring_corner_row; }
int16_t img_get_roundabout_entry_error(void) { return s_ring_entry_error; }
uint8_t img_get_roundabout_preview_valid(void) { return s_ring_preview_valid; }
uint8_t img_get_near_valid(void) { return s_valid && s_near_valid; }
uint8_t img_diag_near_rows(void) { return s_diag_near_rows; }
uint8_t img_diag_island_bottom(void) { return s_ring_maxy; }
void img_set_roundabout_guidance(int side) {
    int guide = side < 0 ? -1 : (side > 0 ? 1 : 0);
    if (guide != s_ring_guide) {
        ep_reset();
        ip_bridge_reset();
        s_entry_locked = s_entry_status = 0;
        s_entry_path.count = 0;
    }
    s_ring_guide = guide;
}
uint8_t img_get_valid(void)       { return s_valid; }
int16_t img_get_error(void)       { return s_error; }
int16_t img_get_preview_error(void) { return s_preview_error; }
uint8_t img_get_preview_valid(void) { return s_preview_valid; }
uint8_t img_get_stripe_active(void) { return s_stripe_recovery != 0; }
uint8_t img_get_stripe_rows(void) { return s_stripe_rows; }
uint8_t img_get_usable_rows(void) { return s_usable_rows; }
uint8_t img_get_both_rows(void)   { return s_both_rows; }

void auxiliary_process(uint8_t *_src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols,
                       unsigned char _threshold_val,
                       uint8_t *_left_line, uint8_t *_mid_line, uint8_t *_right_line)
{
    static uint32_t band_hist[256];
    uint8_t  band_thr[IP_BANDS];
    uint8_t  band_bright_supported[IP_BANDS] = {0};
    uint8_t  row_thr[IP_H];
    uint8_t  bin[IP_W];
    int      band_rows = (IP_H + IP_BANDS - 1) / IP_BANDS;
    int      seed, y, usable = 0, both = 0;
    long     err_sum = 0;
    int      err_n = 0;
    long     preview_sum = 0;
    int      preview_n = 0;
    int      near_rows = 0;
    uint8_t measured_pair[IP_H] = {0};
    int measured_l[IP_H] = {0}, measured_r[IP_H] = {0};
    s_road_recovered = 0;
    int      branch_hw_delta = 0, have_branch_hw_delta = 0;

    (void)_threshold_val;
    if (_src_rows != IP_H || _src_cols != IP_W) return;
    if (!_src_pixel_mat || !_left_line || !_mid_line || !_right_line) return;

    /* ---- 1. 按行带求阈值 ---- */
    for (int b = 0; b < IP_BANDS; ++b) {
        uint32_t total = 0;
        int y0 = b * band_rows;
        int y1 = (y0 + band_rows < IP_H) ? (y0 + band_rows) : IP_H;
        if (y1 > IP_TRACK_BOTTOM + 1) y1 = IP_TRACK_BOTTOM + 1;
        memset(band_hist, 0, sizeof band_hist);
        for (y = y0; y < y1; ++y) {
            const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
            for (int x = 0; x < IP_W; ++x) ++band_hist[src[x]];
        }
        for (int i = 0; i < 256; ++i) total += band_hist[i];
        if (total < (uint32_t)IP_BAND_MIN_ROWS * IP_W)
            band_thr[b] = b ? band_thr[b - 1] : 128;
        else
            band_thr[b] = ip_band_threshold(band_hist, total, &band_bright_supported[b]);
        /* A saturated all-white band cannot estimate a separating threshold.
         * Reuse the preceding measured band instead of classifying white==254
         * as black. Global contrast and measured near edges still gate validity. */
        if (b && total) {
            int lo=0,hi=255;
            while(lo<255 && !band_hist[lo])++lo;
            while(hi>lo && !band_hist[hi])--hi;
            if(hi-lo<IP_MIN_DR && lo>band_thr[b-1])band_thr[b]=band_thr[b-1];
        }
        s_scene_ok = 0;
    }
    /* A band without a compact road-bright population cannot establish a
     * lower white-road level by itself. Borrow agreeing adjacent road levels.
     * A genuinely shaded band with its own supported bright road population
     * retains its local threshold. Use original estimates to avoid cascades. */
    {
        uint8_t measured[IP_BANDS];
        memcpy(measured, band_thr, sizeof measured);
        for (int b = 1; b < IP_BANDS - 1; ++b) {
            int reference = measured[b - 1] < measured[b + 1] ? measured[b - 1] : measured[b + 1];
            if (band_bright_supported[b] || !band_bright_supported[b - 1] ||
                !band_bright_supported[b + 1]) continue;
            if (ip_abs(measured[b - 1] - measured[b + 1]) <= 16 &&
                reference - measured[b] >= 24)
                band_thr[b] = (uint8_t)reference;
        }
    }
    for (y = 0; y < IP_H; ++y) { row_thr[y] = band_thr[y / band_rows]; s_row_thr[y] = row_thr[y]; }
    {
        int lo = 255, hi = 0;
        for (y = IP_TRACK_TOP; y <= IP_TRACK_BOTTOM; ++y) {
            const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
            for (int x = 0; x < IP_W; ++x) { if (src[x] < lo) lo = src[x]; if (src[x] > hi) hi = src[x]; }
        }
        s_scene_ok = (uint8_t)(hi - lo >= IP_MIN_DR);
    }

    /* ---- 1b. 条纹检测 ---- */
    {
        int chain = 0, longest = 0, previous = -1, rows = 0;
        memset(s_stripe_mask, 0, sizeof s_stripe_mask);
        for (y = IP_TRACK_TOP; y <= IP_TRACK_BOTTOM; ++y) {
            int center;
            const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
            if (s_scene_ok && ip_stripe_pattern(src, row_thr[y], &center)) {
                s_stripe_mask[y] = 1; ++rows;
                chain = previous >= 0 && ip_abs(center-previous) <= 6 ? chain+1 : 1;
                previous = center;
                if (chain > longest) longest = chain;
            } else { chain = 0; previous = -1; }
        }
        s_stripe_rows = longest >= 4 ? (uint8_t)rows : 0;
        if (!s_stripe_rows) memset(s_stripe_mask, 0, sizeof s_stripe_mask);
        if (s_stripe_rows) s_stripe_recovery = 3;
        else if (s_stripe_recovery) --s_stripe_recovery;
    }

    ip_island(_src_pixel_mat, row_thr);
    ip_approach_bridge(_src_pixel_mat, row_thr);

    /* Pre-measure branch dual-edge half-width delta on the guided island rows
     * so single-edge branch rows (where outer edge leaves FOV) use the true
     * branch width rather than jumping to full-track half-width. */
    if ((s_ring_guide || s_ring_candidate) && s_ring_miny && s_ring_maxy >= s_ring_miny) {
        int g = s_ring_guide ? s_ring_guide : ((s_ring_mask[s_ring_miny] == 2) ? 1 : -1);
        for (y = s_ring_maxy; y >= s_ring_miny; --y) {
            if (!(s_ring_mask[y] & (g < 0 ? 1 : 2))) continue;
            const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
            uint8_t thr = row_thr[y];
            int l, r;
            if (g < 0) {
                r = s_ring_l[y]; l = r;
                while (l > 0 && src[l - 1] > thr) --l;
            } else {
                l = s_ring_r[y]; r = l;
                while (r < IP_W - 1 && src[r + 1] > thr) ++r;
            }
            if (l > 2 && r < IP_W - 3 && r - l + 1 >= IP_RUN_MIN) {
                branch_hw_delta = ip_clamp((r - l + 1) / 2 - ip_half(y), -20, 0);
                have_branch_hw_delta = 1;
                break; /* Closest dual-edge branch row to the vehicle */
            }
        }
    }

    /* ---- 2. 从底部向上逐行找边界 ---- */
    for (y = 0; y < IP_H; ++y) { _left_line[y] = 0; _right_line[y] = MT9V03X_W - 1; _mid_line[y] = 0; }

    for (y = IP_TRACK_BOTTOM; y >= IP_TRACK_TOP; --y) {
        const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
        uint8_t thr = row_thr[y];
        int l, r, width;
        int ref = (y < IP_H - 1 && s_last_center[y + 1] >= 0) ? s_last_center[y + 1] : IP_W / 2;

        if (s_stripe_rows || s_ring_candidate || s_ring_guide || s_approach_side[y]) continue;

        for (int x = 0; x < IP_W; ++x) bin[x] = (uint8_t)(src[x] > thr);
        if (!ip_row_run(bin, src, thr, -1, ip_half(y), ref, &l, &r)) continue;
        width = r - l + 1;
        if (l <= 2 || r >= IP_W - 3) continue;
        if (width < 30 || width > IP_W - 20) continue;
        if (!ip_width_matches(y, width)) continue;
        {
            int old = s_half[y] ? s_half[y] : ip_half_default(y);
            s_half[y] = (uint8_t)((old * (IP_HALF_ALPHA - 1) + width / 2) / IP_HALF_ALPHA);
            ++both;
        }
    }
    if (both >= IP_BOTH_SIDE_MIN) s_half_ready = 1;
    {
        long diff_sum = 0;
        int  diff_n = 0;
        for (y = IP_H - 40; y < IP_H; ++y) {
            int m = ip_half(y), w = s_half[y];
            if (!s_half_bias_ready) m = ip_half_default(y);
            if (w >= 20) { diff_sum += ((int)w) - m; ++diff_n; }
        }
        if (!s_stripe_rows && !s_ring_candidate && !s_ring_guide && diff_n >= 8) {
            int bias = (int)(diff_sum / diff_n);
            bias = (bias < -20) ? -20 : ((bias > 20) ? 20 : bias);
            if (s_half_bias_ready) s_half_bias = (int16_t)((s_half_bias * 3 + bias) / 4);
            else { s_half_bias = (int16_t)bias; s_half_bias_ready = 1; }
        }
    }

    /* 第二遍：正式逐行追踪 */
    seed = IP_W / 2;
    for (y = IP_TRACK_BOTTOM; y >= IP_TRACK_TOP; --y) {
        const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
        uint8_t thr = row_thr[y];
        int l, r, width, center, half;
        int ref;
        int in_guided_branch = 0;
        int raw_l, raw_r;

        if (s_stripe_mask[y]) {
            if (!ip_stripe_edges(src, thr, y, &l, &r)) continue;
            center = (l+r)/2;
            if (s_last_center[y] >= 0 && ip_abs(center-s_last_center[y]) > IP_JUMP_MAX)
                continue;
            _left_line[y] = (uint8_t)l; _right_line[y] = (uint8_t)r;
            _mid_line[y] = (uint8_t)center;
            ++usable; seed = center;
            if (y >= IP_ERROR_ROW_TOP && y <= IP_ERROR_ROW_BOT) {
                int wgt = IP_ERROR_NEAR + (y-IP_ERROR_ROW_TOP);
                err_sum += (long)center*wgt; err_n += wgt; ++near_rows;
            }
            continue;
        }

        for (int x = 0; x < IP_W; ++x) bin[x] = (uint8_t)(src[x] > thr);

        ref = (y < IP_H - 1 && s_last_center[y + 1] >= 0) ? s_last_center[y + 1] : seed;
        {
            int pred_left = ref - ip_half(y);
            if (s_ring_guide && !s_approach_active && (s_ring_mask[y] & (s_ring_guide<0 ? 1:2)) && !s_stripe_recovery) {
                in_guided_branch = 1;
                if(s_ring_guide<0) {
                    r=s_ring_l[y];l=r;while(l>0 && bin[l-1])--l;
                }else{
                    l=s_ring_r[y];r=l;while(r<IP_W-1 && bin[r+1])++r;
                }
            } else if (!ip_row_run(bin, src, thr, pred_left, ip_half(y), ref, &l, &r)) continue;
        }
        if (r - l + 1 < IP_RUN_MIN) continue;

        width = r - l + 1;
        raw_l=l; raw_r=r;
        if (s_approach_side[y] == 2) r = s_approach_edge[y];
        else if (s_approach_side[y] == 1) l = s_approach_edge[y];
        width = r - l + 1;
        {
            int left_ok  = (l > 2);
            int right_ok = (r < IP_W - 3);
            if (!right_ok) {
                int k, cut = -1, best = 24;
                for (k = r - 6; k > l + 8; --k) {
                    int drop = (int)src[k] - (int)src[k + 2];
                    if (drop > best) { best = drop; cut = k; }
                }
                if (cut > 0 && cut < r - 6) { r = cut; right_ok = 1; }
            }
            if (!left_ok) {
                int k, cut = -1, best = 24;
                for (k = l + 6; k < r - 8; ++k) {
                    int drop = (int)src[k] - (int)src[k - 2];
                    if (drop > best) { best = drop; cut = k; }
                }
                if (cut > 0 && cut > l + 6) { l = cut; left_ok = 1; }
            }
            int mid = (l + r) / 2;
            half = ip_half(y);
            if ((in_guided_branch || (s_ring_candidate && s_ring_mask[y])) && have_branch_hw_delta) {
                half = ip_clamp(half + branch_hw_delta, 16, half);
            }
            if (width > half * 2 + half) {
                if(s_ring_guide || s_ring_candidate)continue;
                l = ip_clamp(mid - half, 1, IP_W - 2);
                r = ip_clamp(mid + half, 1, IP_W - 2);
                width = r - l + 1;
                left_ok  = (l > 2);
                right_ok = (r < IP_W - 3);
            }
            if (!left_ok && !right_ok) continue;

            if (left_ok && right_ok) {
                center = mid;
                if (in_guided_branch || (s_ring_candidate && s_ring_mask[y])) {
                    branch_hw_delta = ip_clamp(width / 2 - ip_half(y), -20, 0);
                    have_branch_hw_delta = 1;
                }
            } else if (left_ok) {
                center = l + half;
            } else {
                center = r - half;
            }
        }

        if (center < 2 || center >= IP_W - 2) continue;
        if((s_ring_candidate || s_ring_guide) && !bin[center])continue;
        /* A verified current bridge replaces the old opening-biased center. */
        if (!in_guided_branch && !s_approach_side[y] &&
            s_last_center[y] >= 0 && ip_abs(center - s_last_center[y]) > IP_JUMP_MAX) {
            continue;
        }
        /* Exit evidence uses only measured binary transitions, before gradient
         * substitutes, width clipping, entry ramps or historical interpolation. */
        if (!in_guided_branch && !s_approach_side[y] && raw_l>2 && raw_r<IP_W-3 &&
            ip_width_matches(y,raw_r-raw_l+1)) {
            measured_pair[y]=1; measured_l[y]=raw_l; measured_r[y]=raw_r;
        }
        _left_line[y]  = (uint8_t)l;
        _right_line[y] = (uint8_t)r;
        _mid_line[y]   = (uint8_t)center;
        ++usable;
        seed = center;
        if (y >= 55 && y <= 75) {
            preview_sum += center; ++preview_n;
        }
        if (y >= IP_ERROR_ROW_TOP && y <= IP_ERROR_ROW_BOT) {
            int wgt = IP_ERROR_NEAR + (y - IP_ERROR_ROW_TOP);
            err_sum += (long)center * wgt;
            err_n += wgt;
            ++near_rows;
        }
    }

    /* A normal-road corridor must connect near field to preview continuously.
     * This is conservative recovery evidence, NOT a generic exit/corner detector. */
    if (!s_ring_candidate && !s_ring_miny && !s_stripe_recovery) {
        int chain=0, bottom=0;
        for(y=IP_TRACK_BOTTOM;y>=55;--y) {
            if(!measured_pair[y]) { chain=0; continue; }
            if(chain && (ip_abs(measured_l[y]-measured_l[y+1])>4 ||
                         ip_abs(measured_r[y]-measured_r[y+1])>4))chain=0;
            if(!chain)bottom=y;
            ++chain;
            if(chain>=35 && bottom>=90 && y<=65)s_road_recovered=1;
        }
    }

    /* Save raw measured centers for next-frame jump gating BEFORE applying the
     * entry throat target ramp, so throat pulling never causes a false >IP_JUMP_MAX
     * rejection on the raw corridor in the subsequent frame. */
    for (y = 0; y < IP_H; ++y)
        s_last_center[y] = _mid_line[y] ? (int16_t)_mid_line[y] : -1;

    /* ---- 2b. LAP29 入环目标路径生成（分离于候选识别，基于当前实测白色通道与拐点） ----
     * When roundabout guidance is active (or a candidate island with valid near-field
     * is present), construct the continuous target entry path from IP_TRACK_BOTTOM
     * up to the measured island lower corner (y_low) and arc apex (y_apex).
     * Every generated target point is strictly verified to lie on a current white pixel. */
    s_ring_entry_error = 0;
    if (!s_approach_active && !s_stripe_recovery && near_rows >= 6 && s_ring_miny && s_ring_maxy >= s_ring_miny) {
        int active_side = s_ring_guide;
        if (!active_side && s_ring_candidate) {
            active_side = (s_ring_mask[s_ring_maxy] == 2) ? 1 : -1;
        }
        if (active_side != 0) {
            int y_low = s_ring_maxy;
            int y_apex = s_ring_apex_row ? (int)s_ring_apex_row : (s_ring_miny + s_ring_maxy) / 2;
            int c_low = _mid_line[y_low];
            int c_apex = _mid_line[y_apex];
            /* At a clipped throat, the ordinary midpoint is pulled toward the
             * visible island edge. Reconstruct the branch center from the
             * measured outer edge and the calibrated half width. */
            if (s_ring_guide) {
                int yy;
                for (yy=y_low; yy>=s_ring_miny; --yy) {
                    int l=_left_line[yy], r=_right_line[yy];
                    if (!l && !r) continue;
                    if (s_ring_guide<0 && r>=IP_W-3 && l>2)
                        c_low=ip_clamp(l+ip_half(yy),6,IP_W-7);
                    else if (s_ring_guide>0 && l<=2 && r<IP_W-3)
                        c_low=ip_clamp(r-ip_half(yy),6,IP_W-7);
                    else c_low=(l+r)/2;
                    y_low=yy; break;
                }
                for (yy=y_apex; yy>=s_ring_miny; --yy) {
                    int l=_left_line[yy], r=_right_line[yy];
                    if (!l && !r) continue;
                    if (s_ring_guide<0 && r>=IP_W-3 && l>2)
                        c_apex=ip_clamp(l+ip_half(yy),6,IP_W-7);
                    else if (s_ring_guide>0 && l<=2 && r<IP_W-3)
                        c_apex=ip_clamp(r-ip_half(yy),6,IP_W-7);
                    else c_apex=(l+r)/2;
                    y_apex=yy; break;
                }
            }
            if (!c_low) {
                for (y = y_low; y >= (int)s_ring_miny; --y)
                    if (_mid_line[y]) { c_low = _mid_line[y]; y_low = y; break; }
            }
            if (!c_apex) c_apex = c_low;
            if (c_low > 0 && c_apex > 0) {
                /* Target anchor combines the lower bifurcation center and the arc apex center. */
                int y_anchor = (y_low + y_apex) / 2;
                long entry_sum = 0;
                int  entry_n = 0;

                /* Build the entrance path from the measured inner island edge.
                 * For clockwise entry (guide < 0), the road lies to the left of
                 * the pink inner edge. For the mirrored case it lies to the right.
                 * The target stays close to that edge while remaining in the
                 * current white corridor, which preserves the visible bend. */
                if (s_ring_guide) {
                    int path_top = s_ring_miny;
                    int last_target = -1;
                    for (y = path_top; y <= IP_TRACK_BOTTOM; ++y) {
                        int l = _left_line[y], r = _right_line[y];
                        int branch_ok = (s_ring_mask[y] & (s_ring_guide < 0 ? 1 : 2)) != 0;
                        int target_c, width, inner;
                        const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
                        uint8_t thr = row_thr[y];
                        if (!branch_ok || l <= 2 || r <= l + IP_RUN_MIN) {
                            /* Continue the already measured curve only when the
                             * same point is white in this row. This fills a
                             * wide throat without inventing a line across a gap. */
                            if (last_target >= 0 && src[last_target] > thr &&
                                src[last_target - 2] > thr && src[last_target + 2] > thr) {
                                int tl=last_target, tr=last_target;
                                while (tl > 0 && src[tl - 1] > thr) --tl;
                                while (tr < IP_W - 1 && src[tr + 1] > thr) ++tr;
                                if (tr - tl + 1 >= IP_RUN_MIN) {
                                    _mid_line[y]=(uint8_t)last_target;
                                    _left_line[y]=(uint8_t)tl;
                                    _right_line[y]=(uint8_t)tr;
                                }
                            }
                            continue;
                        }
                        if (s_ring_guide < 0 && r < IP_W - 3) {
                            /* The inner edge may be visible inside the image. */
                            inner = r;
                        } else if (s_ring_guide > 0 && l > 2) {
                            inner = l;
                        } else {
                            inner = s_ring_guide < 0 ? r : l;
                        }
                        width = r - l + 1;
                        target_c = s_ring_guide < 0 ? inner - width * 35 / 100
                                                    : inner + width * 35 / 100;
                        if (last_target >= 0) {
                            if (target_c > last_target + 10) target_c = last_target + 10;
                            if (target_c < last_target - 10) target_c = last_target - 10;
                        }
                        target_c = ip_clamp(target_c, 6, IP_W - 7);
                        /* A target is accepted only when its neighborhood is
                         * current white track. Search toward the road interior
                         * for a nearby white point if the first estimate touches
                         * the island or an exposure hole. */
                        if (!(src[target_c] > thr && src[target_c - 2] > thr && src[target_c + 2] > thr)) {
                            int step = s_ring_guide < 0 ? -1 : 1;
                            int found_target = 0;
                            for (int k = 1; k <= 12; ++k) {
                                int candidate = ip_clamp(target_c + step * k, 6, IP_W - 7);
                                if (src[candidate] > thr && src[candidate - 2] > thr && src[candidate + 2] > thr) {
                                    target_c = candidate; found_target = 1; break;
                                }
                            }
                            if (!found_target) continue;
                        }
                        {
                            int tl = target_c, tr = target_c;
                            while (tl > 0 && src[tl - 1] > thr) --tl;
                            while (tr < IP_W - 1 && src[tr + 1] > thr) ++tr;
                            if (tr - tl + 1 < IP_RUN_MIN) continue;
                            _mid_line[y] = (uint8_t)target_c;
                            _left_line[y] = (uint8_t)tl;
                            _right_line[y] = (uint8_t)tr;
                            last_target = target_c;
                        }
                    }
                }

                /* Compute the path-planned entry error across the near + lower-branch zone. */
                for (y = (y_anchor < IP_ERROR_ROW_TOP ? y_anchor : IP_ERROR_ROW_TOP);
                     y <= IP_ERROR_ROW_BOT; ++y) {
                    int m = _mid_line[y];
                    if (!m) continue;
                    int wgt = IP_ERROR_NEAR + (y >= IP_ERROR_ROW_TOP ? (y - IP_ERROR_ROW_TOP) : 4);
                    entry_sum += (long)m * wgt;
                    entry_n += wgt;
                }
                if (entry_n > 0) {
                    int e_entry = (int)(entry_sum / entry_n) - (IP_W / 2);
                    s_ring_entry_error = (int16_t)ip_clamp(e_entry, -IP_W / 2, IP_W / 2);
                }

                /* When s_ring_guide is active, update err_sum/err_n from the refined
                 * near-field target centerline (y = 80..105). */
                if (s_ring_guide) {
                    err_sum = 0; err_n = 0;
                    for (y = IP_ERROR_ROW_TOP; y <= IP_ERROR_ROW_BOT; ++y) {
                        if (!_mid_line[y]) continue;
                        int wgt = IP_ERROR_NEAR + (y - IP_ERROR_ROW_TOP);
                        err_sum += (long)_mid_line[y] * wgt;
                        err_n += wgt;
                    }
                }
            }
        }
    }

    /* ---- 3. 本帧统计与有效性 ---- */
    s_usable_rows = (uint8_t)(usable > 255 ? 255 : usable);
    s_both_rows   = (uint8_t)(both > 255 ? 255 : both);

    s_valid = (uint8_t)(s_scene_ok && usable >= IP_VALID_ROWS_MIN);
    s_diag_near_rows = (uint8_t)near_rows;
    s_near_valid = (uint8_t)(near_rows >= 6);
    if ((s_stripe_rows || s_ring_candidate || s_ring_guide) && !s_near_valid) s_valid = 0;
    s_preview_valid = (uint8_t)(s_valid && preview_n >= 10 && !s_stripe_recovery);
    /* Guided preview must use the selected inner-edge corridor itself. The old
     * code validated branch rows but averaged unrelated straight-road rows.
     * Use a short measured window at the far end of the visible branch; never
     * extrapolate a missing island or authorize driving without near edges. */
    {
        int total_branch_rows = 0;
        long branch_sum = 0;
        if (s_ring_guide && !s_approach_active) {
            int end = ip_clamp((int)s_ring_miny+19,IP_TRACK_TOP,IP_TRACK_BOTTOM);
            for (y = s_ring_miny; s_ring_miny && y <= end; ++y)
                if (_mid_line[y] && (s_ring_mask[y] & (s_ring_guide < 0 ? 1 : 2))) {
                    ++total_branch_rows;branch_sum+=_mid_line[y];
                }
            preview_n=total_branch_rows;preview_sum=branch_sum;
            s_preview_valid=(uint8_t)(s_valid && s_near_valid && total_branch_rows>=10 && !s_stripe_recovery);
        }
        s_ring_preview_valid = (uint8_t)(!s_approach_active && s_ring_guide && s_preview_valid && total_branch_rows>=10);
    }
    if (s_preview_valid)
        s_preview_error = (int16_t)ip_clamp((int)(preview_sum/preview_n)-IP_W/2,-IP_W/2,IP_W/2);
    else s_preview_error = 0;
    if (s_valid) {
        int e = err_n ? (int)(err_sum / err_n) : (IP_W / 2);
        e -= IP_W / 2;
        if (s_have_error) {
            int max_step = s_ring_guide ? 28 : 16;
            if (e >  s_error + max_step) e = s_error + max_step;
            if (e <  s_error - max_step) e = s_error - max_step;
        }
        if (e > -3 && e < 3) e = 0;
        s_error = (int16_t)ip_clamp(e, -IP_W / 2, IP_W / 2);
        s_have_error = 1;
    }

    /* ---- 3c. 把中线吸附到真正属于赛道(白)的像素上 ---- */
    for (y = IP_TRACK_TOP; y < IP_H; ++y) {
        const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
        int m = _mid_line[y], k, chosen = -1;
        if (s_stripe_mask[y]) continue;
        if (!m) continue;
        if (src[m] > s_row_thr[y]) continue;
        for (k = 1; k <= 3 && chosen < 0; ++k) {
            if (m - k >= 1 && src[m - k] > s_row_thr[y]) chosen = m - k;
            else if (m + k < IP_W - 1 && src[m + k] > s_row_thr[y]) chosen = m + k;
        }
        if (chosen >= 0) { _mid_line[y] = (uint8_t)chosen; m = chosen; }
        else { _mid_line[y] = 0; }
        if (_mid_line[y]) {
            int half = ip_half(y);
            if (_left_line[y])  _left_line[y]  = _mid_line[y] > half ? (uint8_t)(_mid_line[y] - half) : 0;
            if (_right_line[y]) _right_line[y] = _mid_line[y] + half < IP_W ? (uint8_t)(_mid_line[y] + half) : IP_W - 1;
        }
    }

    /* ---- 3b. 空洞填补 ---- */
    for (y = IP_H - 1; y >= 0; --y) {
        int up, down;
        if (_mid_line[y]) continue;
        up = down = -1;
        for (int k = y + 1; k < IP_H; ++k) if (_mid_line[k]) { up = k; break; }
        for (int k = y - 1; k >= 0; --k)    if (_mid_line[k]) { down = k; break; }
        if (up >= 0 && down >= 0) {
            int a = _mid_line[up], b = _mid_line[down];
            _mid_line[y] = (uint8_t)((a * (y - down) + b * (up - y)) / (up - down));
            _left_line[y]  = _mid_line[y] > ip_half(y) ? (uint8_t)(_mid_line[y] - ip_half(y)) : 0;
            _right_line[y] = _mid_line[y] + ip_half(y) < IP_W ? (uint8_t)(_mid_line[y] + ip_half(y)) : IP_W - 1;
        } else if (s_last_center[y] >= 0) {
            _mid_line[y] = (uint8_t)s_last_center[y];
            _left_line[y]  = _mid_line[y] > ip_half(y) ? (uint8_t)(_mid_line[y] - ip_half(y)) : 0;
            _right_line[y] = _mid_line[y] + ip_half(y) < IP_W ? (uint8_t)(_mid_line[y] + ip_half(y)) : IP_W - 1;
        }
    }

    /* ---- 4. 补洞后若某行此前无测量中线，补充记录供下一帧参考 ---- */
    for (y = 0; y < IP_H; ++y)
        if (s_last_center[y] < 0 && _mid_line[y])
            s_last_center[y] = (int16_t)_mid_line[y];
    /* Entrance branch owns the control path after its first visible side
     * opening. Every target comes from the current binary image. */
    if (s_entry_mode && s_ring_guide && !s_stripe_recovery) {
        int built = 0;
        s_entry_path.count = 0;
        if (s_scene_ok) {
            built = ep_build(_src_pixel_mat, row_thr, s_ring_guide, IP_W / 2, &s_entry_path);
            /* A complete center-to-endpoint path is not always observable at
             * the throat. Keep the measured-branch controller alive when the
             * current frame contains a shorter, white corridor. This is a
             * stage-specific fallback, not historical line continuation. */
            if (!built && s_ring_guide && s_near_valid) {
                entry_path_t measured_path;
                if (ep_build_measured_corridor(_src_pixel_mat, row_thr,
                                               s_ring_guide, IP_W / 2,
                                               &measured_path)) {
                    s_entry_path = measured_path;
                    built = 1;
                }
            }
        } else {
            memset(&s_entry_path, 0, sizeof s_entry_path);
            s_entry_path.failure = 5;
            /* A brief loss of image contrast does not change the learned width.
             * Motion continuation is separately bounded by the bridge guard. */
        }
        if (s_entry_path.failure == 6) s_entry_locked = 1;
        if (built) {
            memset(s_approach_side, 0, sizeof s_approach_side);
            s_approach_active = s_approach_width_side = 0;
            s_entry_locked = 1;
            s_entry_status = s_entry_path.boundary_source==3 ? 4 : (s_entry_path.inferred ? 2 : 1);
            s_valid = s_near_valid = 1; /* Current boundary + connected near-white path. */
            s_error = s_ring_entry_error = s_entry_path.error;
            s_preview_error = s_entry_path.preview;
            s_preview_valid = s_ring_preview_valid = 1;
            ip_bridge_observe(row_thr);
            /* The legacy row arrays contain ONLY the ordered path's points.
             * They cannot store a horizontal turn's complete shape. The ordered
             * API and LCD drawing below carry all points and paired boundaries. */
            memset(_mid_line, 0, IP_H);
            memset(_left_line, 0, IP_H);
            memset(_right_line, IP_W - 1, IP_H);
            for (int i = 0; i < s_entry_path.count; ++i) {
                const entry_point_t *point = &s_entry_path.points[i];
                if (!_mid_line[point->y]) _mid_line[point->y] = point->x;
            }
        } else if (s_entry_locked) {
            int bridged=(s_entry_path.failure==1 || s_entry_path.failure==5) &&
                        ip_bridge_apply(_src_pixel_mat,row_thr);
            if (!bridged) {
                ip_bridge_reset();
                s_entry_status = 3;
                s_ring_entry_error = s_preview_error = 0;
                s_valid = s_near_valid = s_preview_valid = s_ring_preview_valid = 0;
            }
            memset(_mid_line,0,IP_H);
            memset(_left_line,0,IP_H);
            memset(_right_line,IP_W-1,IP_H);
        } else {
            s_entry_status = 0; /* Preparatory approach before a side opening. */
            if (s_approach_active) {
                s_ring_entry_error = s_error;
                s_ring_preview_valid = 0;
            }
        }
    } else {
        ip_bridge_reset();
        s_entry_status = 0;
        s_entry_path.count = 0;
    }

}
