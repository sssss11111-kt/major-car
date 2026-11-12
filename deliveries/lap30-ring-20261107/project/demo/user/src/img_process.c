#define RING_ENTRY_IMPLEMENTATION
#include "img_process.h"
#undef RING_ENTRY_IMPLEMENTATION
#include "string.h"
#include "steering_control.h"

static int ip_abs(int v) { return v < 0 ? -v : v; }

/* ---------- 可调常量 ---------- */
#define IP_W              (MT9V03X_W)
#define IP_H              (MT9V03X_H)
#define IP_BANDS          8          /* 纵向分带数，每带 IP_H/IP_BANDS 行 */
#define IP_BAND_MIN_ROWS  8          /* 少于此行数的带直接沿用相邻带阈值 */
#define IP_LOW_MARGIN_PCT 22         /* 阈值下限 = 带内最暗 + (最亮-最暗)*22/100 */
#define IP_MIN_DR          10        /* 一帧内最亮-最暗低于这个值，视为无有效场景 */

static uint8_t s_scene_ok, s_scene_contrast;
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
static uint8_t  s_single_rows;
static uint8_t  s_near_run;
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
static uint8_t s_entry_mode, s_entry_locked, s_entry_status, s_orbit_ready;
static entry_path_t s_entry_path;
#define IP_ENTRY_BLEND_STEP 0.20f
#define IP_ENTRY_INSIDE_FRAMES 3U
static float s_entry_weight;
static float s_align_turn_bias;
static uint32_t s_entry_weight_tick,s_entry_control_tick;
static uint8_t s_entry_inside,s_entry_control_seen;
static int16_t s_entry_control_error,s_entry_control_preview;
static uint8_t s_exit_tip_row, s_exit_tip_x;
static uint8_t s_entry_ready, s_exit_ready, s_exit_search;
static uint8_t s_entry_half_width[IP_H], s_entry_island_mask[IP_H];
static uint8_t s_entry_lost_frames, s_entry_reacquire;
static entry_bridge_guard_t s_bridge_guard;
static float s_odometry_distance, s_odometry_yaw, s_bridge_curvature;
static uint32_t s_odometry_tick;
static uint8_t s_odometry_valid, s_motion_ready, s_bridge_stable;

static void ip_bridge_reset(void) {
    memset(&s_bridge_guard,0,sizeof s_bridge_guard);
    s_bridge_curvature=0;
    s_bridge_stable=s_motion_ready=0;
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

/* Before acquiring the island, a side opening can merge into the white run.
 * Measure the intact stem at the bottom, then follow the bank that stays on
 * its approach tangent. All repaired rows still lie in the current run. */
static int ip_approach_opening(const int *left, const int *right)
{
    enum { STEM_SPAN = 5, OPENING_EXCESS = 8, OPENING_ROWS = 4 };
    const int bottom = IP_TRACK_BOTTOM, stem_top = bottom - STEM_SPAN;
    int width_offset = 0;
    for (int y = stem_top; y <= bottom; ++y) {
        int width = right[y] - left[y];
        int l = left[bottom] + (left[stem_top] - left[bottom]) * (bottom - y) / STEM_SPAN;
        int r = right[bottom] + (right[stem_top] - right[bottom]) * (bottom - y) / STEM_SPAN;
        if (left[y] <= 2 || right[y] >= IP_W - 3 || !ip_width_matches(y, width + 1) ||
            ip_abs(left[y] - l) > 2 || ip_abs(right[y] - r) > 2) return 0;
        width_offset += width - 2 * ip_half_default(y);
    }
    width_offset /= STEM_SPAN + 1;
    for (int side = 1; side <= 2; ++side) {
        const int *bank = side == 1 ? right : left;
        int direction = side == 1 ? -1 : 1;
        int start = -1, end = -1, wide_rows = 0;
        uint8_t repaired[IP_H];
        for (int y = stem_top - 1; y >= 55; --y) {
            int predicted = bank[bottom] + (bank[stem_top] - bank[bottom]) * (bottom - y) / STEM_SPAN;
            int edge = bank[y] + direction * (2 * ip_half_default(y) + width_offset);
            if (left[y] < 0 || bank[y] <= 2 || bank[y] >= IP_W - 3 ||
                ip_abs(bank[y] - predicted) > 3) break;
            if (edge < left[y] - 2 || edge > right[y] + 2 ||
                (edge - IP_W / 2) * direction < 0) break;
            int excess = direction * ((side == 1 ? left[y] : right[y]) - edge);
            if (excess <= 2) {
                if (start >= 0) break;
                continue;
            }
            if (start < 0) start = y;
            end = y;
            if (excess >= OPENING_EXCESS) ++wide_rows;
            repaired[y] = (uint8_t)edge;
        }
        if (start < IP_ERROR_ROW_TOP || wide_rows < OPENING_ROWS) continue;
        for (int y = end; y <= start; ++y) {
            s_approach_edge[y] = repaired[y];
            s_approach_side[y] = (uint8_t)side;
        }
        return 1;
    }
    return 0;
}

static void ip_approach_bridge(const uint8_t *raw, const uint8_t *thr)
{
    int left[IP_H], right[IP_H];
    memset(s_approach_side, 0, sizeof s_approach_side);
    s_approach_active = 0;
    if (s_approach_width_side && s_approach_width_age < 255) ++s_approach_width_age;
    if (s_entry_locked || (s_entry_mode > 1 && s_entry_mode != 5) || s_stripe_recovery || !s_scene_ok) {
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
    if (!s_ring_guide) s_approach_active = (uint8_t)ip_approach_opening(left, right);
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

/* Reuse the island scratch buffer after segmentation. Four-connected WHITE
 * pixels seeded under the car are the only road component row targets may use. */
static void ip_road_add(const uint8_t *raw,const uint8_t *thr,int x,int y,int *tail) {
    if(x<0 || x>=IP_W || y<IP_TRACK_TOP || y>IP_TRACK_BOTTOM)return;
    uint8_t bit=(uint8_t)(1U<<(x&7));
    if((s_exterior[y][x>>3]&bit) || raw[y*IP_W+x]<=thr[y])return;
    s_exterior[y][x>>3]|=bit;
    s_exterior_queue[(*tail)++]=(uint16_t)(y*IP_W+x);
}
static void ip_road_component(const uint8_t *raw,const uint8_t *thr) {
    int head=0,tail=0;memset(s_exterior,0,sizeof s_exterior);
    ip_road_add(raw,thr,IP_W/2,IP_TRACK_BOTTOM,&tail);
    while(head<tail) {
        int p=s_exterior_queue[head++],x=p%IP_W,y=p/IP_W;
        ip_road_add(raw,thr,x-1,y,&tail);ip_road_add(raw,thr,x+1,y,&tail);
        ip_road_add(raw,thr,x,y-1,&tail);ip_road_add(raw,thr,x,y+1,&tail);
    }
}
static int ip_on_road(int x,int y) {
    return x>=0 && x<IP_W && y>=IP_TRACK_TOP && y<=IP_TRACK_BOTTOM &&
           (s_exterior[y][x>>3]&(1U<<(x&7)));
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
            /* Once the island is acquired, its near bank can become a narrow
             * bottom-clipped arc while both entrance banks remain visible. */
            if(h<12 || w>120 || tail*3<h*w)continue;
            if(s_ring_guide) {
                if(w<4 || tail<32)continue;
            } else if(h<20 || w<20 || tail<320)continue;
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
                      ((miny>=IP_TRACK_TOP+2 && wide-top_width>=ip_clamp(wide/2,2,10)) ||
                       (maxy<=IP_TRACK_BOTTOM-2 && wide-bottom_width>=ip_clamp(wide/2,2,10)))) {
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


/* Every ring candidate starts at the same camera anchor and uses fixed
 * path-distance windows. Horizontal turns never switch to a tail percentage.
 * These pixel distances remain provisional until camera/vehicle calibration. */
static int ip_path_row(const entry_path_t *path,int row,float *x) {
    for(int i=1;i<path->count;++i) {
        const entry_point_t *a=&path->points[i-1],*b=&path->points[i];
        if(b->y>=a->y)return 0;
        if(row<=a->y && row>=b->y) {
            *x=a->x+(float)(b->x-a->x)*(a->y-row)/(a->y-b->y);return 1;
        }
    }
    return 0;
}
static int ip_path_controls(entry_path_t *path,int *preview_valid) {
    float distance=0,near_sum=0,far_sum=0;int near_n=0,far_n=0;
    *preview_valid=0;path->preview=0;
    if(path->count<2 || path->points[0].x!=94 || path->points[0].y!=EP_BOTTOM)return 0;
    int sample=1;
    for(int i=1;i<path->count && sample<=50;++i) {
        float dx=path->points[i].x-path->points[i-1].x;
        float dy=path->points[i].y-path->points[i-1].y;
        float length=sqrtf(dx*dx+dy*dy);
        if(length<0.001f)continue;
        while(sample<=50 && sample<=distance+length) {
            float x=path->points[i-1].x+dx*(sample-distance)/length;
            if(sample>=5 && sample<=25){near_sum+=x;++near_n;}
            if(sample>=30){far_sum+=x;++far_n;}
            ++sample;
        }
        distance+=length;
    }
    if(near_n!=21)return 0;
    path->error=(int16_t)lroundf(near_sum/near_n-94);
    *preview_valid=far_n==21;
    if(*preview_valid)path->preview=(int16_t)lroundf(far_sum/far_n-94);
    return 1;
}
/* Fit a current ordinary corridor to the entrance's forward cubic parameter.
 * There is no cached image/path: only blend progress survives a new frame. */
static int ip_entry_transition(const uint8_t *raw,const uint8_t *thr,
                              const uint8_t *ordinary,float weight,
                              const entry_path_t *entry,entry_path_t *out) {
    float end=1.0f,curve[8],normal[4],mat[3][4]={{0}};
    if(entry->count<12)return 0;
    /* Split the current cubic at y=55, keeping its forward control horizon. */
    if(entry->curve[7]<55) {
        float lo=0,hi=1;
        for(int k=0;k<16;++k){float x,y,dx,dy,ddx,ddy;
            end=(lo+hi)*0.5f;
            ep_curve_sample(entry->curve,end,&x,&y,&dx,&dy,&ddx,&ddy);
            if(y>55)lo=end;else hi=end;
        }
        end=(lo+hi)*0.5f;
    }
    float q[8];memcpy(q,entry->curve,sizeof q);
    curve[0]=q[0];curve[1]=q[1];
    for(int level=1;level<=3;++level){
        for(int j=0;j<4-level;++j)for(int c=0;c<2;++c)
            q[j*2+c]=(1-end)*q[j*2+c]+end*q[(j+1)*2+c];
        curve[level*2]=q[0];curve[level*2+1]=q[1];
    }
    int bottom=(int)(curve[1]+0.5f);
    if(bottom<90 || bottom>105 || !ordinary[bottom] || curve[7]>84)return 0;
    normal[0]=ordinary[bottom];
    for(int i=1;i<=32;++i) {
        float t=i/32.0f,u=1-t,x,y,dx,dy,ddx,ddy;
        ep_curve_sample(curve,t,&x,&y,&dx,&dy,&ddx,&ddy);
        y=bottom+(55-bottom)*t;
        int row=(int)(y+0.5f);
        if(row<55 || row>105 || !ordinary[row])return 0;
        float b[3]={3*u*u*t,3*u*t*t,t*t*t};
        float target=ordinary[row]-u*u*u*normal[0];
        for(int j=0;j<3;++j){for(int k=0;k<3;++k)mat[j][k]+=b[j]*b[k];mat[j][3]+=b[j]*target;}
    }
    for(int i=0;i<3;++i){
        int pivot=i;for(int j=i+1;j<3;++j)if(fabsf(mat[j][i])>fabsf(mat[pivot][i]))pivot=j;
        if(fabsf(mat[pivot][i])<0.0001f)return 0;
        for(int k=i;k<4;++k){float v=mat[i][k];mat[i][k]=mat[pivot][k];mat[pivot][k]=v;}
        float scale=mat[i][i];for(int k=i;k<4;++k)mat[i][k]/=scale;
        for(int j=0;j<3;++j)if(j!=i){float v=mat[j][i];for(int k=i;k<4;++k)mat[j][k]-=v*mat[i][k];}
    }
    for(int i=1;i<4;++i)normal[i]=mat[i-1][3];
    /* Fitting must not silently invent a different ordinary route. */
    float normal_curve[8];memcpy(normal_curve,curve,sizeof curve);
    for(int i=0;i<4;++i){normal_curve[2*i]=normal[i];normal_curve[2*i+1]=bottom+(55-bottom)*i/3.0f;}
    for(int i=0;i<=32;++i){float x,y,dx,dy,ddx,ddy;
        ep_curve_sample(normal_curve,i/32.0f,&x,&y,&dx,&dy,&ddx,&ddy);
        int row=(int)(y+0.5f);
        if(!ordinary[row] || fabsf(x-ordinary[row])>8.0f)return 0;
    }
    for(int i=0;i<8;++i)curve[i]=normal_curve[i]+weight*(curve[i]-normal_curve[i]);
    *out=*entry;
    if(!ep_sample_curve(raw,thr,curve,out))return 0;
    memcpy(out->curve,curve,sizeof curve);
    /* Sampled centres AND the intervening curve need current white clearance. */
    for(int i=0;i<=128;++i){float x,y,dx,dy,ddx,ddy;
        ep_curve_sample(curve,i/128.0f,&x,&y,&dx,&dy,&ddx,&ddy);
        int xx=(int)(x+0.5f),yy=(int)(y+0.5f);
        if(!ep_white(raw,thr,xx-2,yy) || !ep_white(raw,thr,xx+2,yy))return 0;
    }
    ep_measure_edges(raw,thr,out);
    out->goal_x=out->points[out->count-1].x;out->goal_y=out->points[out->count-1].y;
    int pv=0;return (!ep_context.fresh_island || ep_entry_clearance_ok(out,s_ring_guide)) && ip_path_controls(out,&pv) && pv;
}

/* Publish precisely the validated curve used for control and telemetry. */
static void ip_publish_curve(const entry_path_t *path,uint8_t *left,uint8_t *mid,uint8_t *right) {
    s_entry_path=*path;
    int pv=0;
    if(!ip_path_controls(&s_entry_path,&pv))return;
    s_error=s_ring_entry_error=s_entry_path.error;
    s_preview_error=s_entry_path.preview;s_preview_valid=(uint8_t)pv;
    s_ring_preview_valid=0;s_valid=s_near_valid=1;s_have_error=1;
    memset(left,0,IP_H);memset(mid,0,IP_H);memset(right,0,IP_H);
    for(int i=0;i<s_entry_path.count;++i) {
        const entry_point_t *p=&s_entry_path.points[i];
        mid[p->y]=p->x;
        if(p->measured&1)left[p->y]=p->left_x;
        if(p->measured&2)right[p->y]=p->right_x;
    }
}
/* The first persistent dark-to-light edge above current near white space is
 * the outer bank. Three-pixel averages reject single-pixel texture; adjacent
 * columns must track one contour, rather than another distant white road. */
static int ip_turn_bank(const uint8_t *raw,const uint8_t *thr,int x) {
    for(int y=72;y>=EP_TOP+4;--y) {
        int above=0,below=0;
        for(int d=1;d<=3;++d){above+=raw[(y-d)*IP_W+x];below+=raw[(y+d)*IP_W+x];}
        if(below-above<90 || below<3*(int)thr[y])continue;
        int dark=0,gap=0,max_gap=0;
        for(int yy=y+3;yy<=y+15 && yy<=100;++yy) {
            if(!ep_white(raw,thr,x,yy)){++dark;if(++gap>max_gap)max_gap=gap;}else gap=0;
        }
        if(max_gap<=2 && dark<=3)return y;
    }
    return -1;
}
/* The outer ridge must belong to a broad current dark region extending
 * inward from the selected edge. A short high-contrast mark on white floor
 * cannot provide a lateral bank. Local grey contrast remains usable when the
 * row threshold leaks into the background. Scratch storage is no longer used
 * by the row tracker at this point in the frame. */
static int ip_outer_region(const uint8_t *raw,int guide,int edge) {
    int x=guide<0 ? 182:5,above=0,below=0;
    for(int d=1;d<=3;++d){above+=raw[(edge-d)*IP_W+x];below+=raw[(edge+d)*IP_W+x];}
    int threshold=(above+below)/6,head=0,tail=0,reach=x,top=IP_TRACK_BOTTOM,bottom=IP_TRACK_TOP;
    int seed=(edge-3)*IP_W+x;
    if(raw[seed]>=threshold)return 0;
    memset(s_exterior,0,sizeof s_exterior);
    s_exterior_queue[tail++]=(uint16_t)seed;s_exterior[edge-3][x>>3]|=(uint8_t)(1U<<(x&7));
    while(head<tail) {
        int pos=s_exterior_queue[head++],xx=pos%IP_W,yy=pos/IP_W;
        if(yy<top)top=yy;if(yy>bottom)bottom=yy;
        if((guide<0 && xx<reach) || (guide>0 && xx>reach))reach=xx;
        const int nx[4]={xx-1,xx+1,xx,xx},ny[4]={yy,yy,yy-1,yy+1};
        for(int k=0;k<4;++k) {
            int a=nx[k],b=ny[k];
            if(a<0 || a>=IP_W || b<IP_TRACK_TOP || b>IP_TRACK_BOTTOM)continue;
            uint8_t bit=(uint8_t)(1U<<(a&7));
            if((s_exterior[b][a>>3]&bit) || raw[b*IP_W+a]>=threshold)continue;
            s_exterior[b][a>>3]|=bit;s_exterior_queue[tail++]=(uint16_t)(b*IP_W+a);
        }
    }
    return top==IP_TRACK_TOP && bottom-top>=12 && tail>=256 &&
           (guide<0 ? reach<=110:reach>=77);
}
/* Current island clearance retains the original scaled tip margin whenever
 * a coherent fresh island is visible; no held contour can veto a new image. */
static int ip_curve_clearance(const entry_path_t *path,int guide) {
    return !ep_context.fresh_island || ep_entry_clearance_ok(path,guide);
}
/* Pull the ALIGN curve toward the observed inner bank. Moving only the
 * forward control points toward that bank prevents any outward displacement. */
static int ip_bias_align_curve(const uint8_t *raw, const uint8_t *thr, int guide,
                              entry_path_t *path) {
    if (!ep_context.fresh_island || path->count < 2 || s_ring_corner_row <= 90) return 0;
    /* Delay the inward offset until the current corner nears the car. */
    float progress = ip_clamp(s_ring_corner_row - 90, 0, 15) / 15.0f;
    progress = progress * progress * (3.0f - 2.0f * progress);
    float signed_progress = guide < 0 ? progress : -progress;
    int support = 0, gap_sum = 0;
    for (int i = 0; i < path->count; ++i) {
        int y = path->points[i].y;
        if (!ep_context.island_mask[y]) continue;
        int bank = guide < 0 ? ep_context.island_left[y] : ep_context.island_right[y];
        int gap = guide < 0 ? bank-path->points[i].x : path->points[i].x-bank;
        if (gap <= 0) continue;
        gap_sum += gap;
        ++support;
    }
    if (support < 4) return 0;
    int shift = gap_sum/(4*support);
    if (shift > 14) shift = 14;
    static entry_path_t candidate;
    for (; shift >= 2; shift -= 2) {
        float curve[8];
        memcpy(curve, path->curve, sizeof curve);
        float bias = shift * signed_progress;
        /* Start the inward bend within the near steering window. */
        curve[2] += 0.65f*bias;
        curve[4] += 1.5f*bias;
        curve[6] += bias;
        candidate = *path;
        if (!ep_sample_curve(raw, thr, curve, &candidate)) continue;
        int safe = 1;
        for (int i = 0; i <= 128; ++i) {
            float x,y,dx,dy,ddx,ddy;
            ep_curve_sample(curve,i/128.0f,&x,&y,&dx,&dy,&ddx,&ddy);
            int px = (int)(x+0.5f), py = (int)(y+0.5f);
            float speed2 = dx*dx+dy*dy, cross = dx*ddy-dy*ddx;
            if (!ep_white(raw,thr,px-2,py) || !ep_white(raw,thr,px+2,py) ||
                cross*cross > 0.0225f*speed2*speed2*speed2) { safe = 0; break; }
        }
        if (!safe) continue;
        memcpy(candidate.curve,curve,sizeof curve);
        ep_measure_edges(raw,thr,&candidate);
        int preview = 0;
        if (!ip_curve_clearance(&candidate,guide) ||
            !ip_path_controls(&candidate,&preview) || !preview) continue;
        candidate.goal_x = candidate.points[candidate.count-1].x;
        candidate.goal_y = candidate.points[candidate.count-1].y;
        ep_control_points(&candidate);
        *path = candidate;
        s_align_turn_bias = signed_progress;
        return 1;
    }
    return 0;
}
/* A lateral opening is measured across several columns. In ALIGN it must
 * have two current banks and have reached the near preview window. In ENTER
 * or EXIT a current outer contour may support a cropped inner bank. Width
 * then remains an explicitly inferred, provisional pixel clearance. */
static int ip_turn_curve(const uint8_t *raw,const uint8_t *thr,int guide,
                         int allow_outer,entry_path_t *out,int *paired) {
    int top[5],bottom[5],n=0;
    *paired=0;
    for(int k=0;k<5;++k) {
        int x=guide<0 ? 182-2*k:5+2*k;
        int best_a=-1,best_b=-1;
        for(int y=EP_TOP;y<=EP_BOTTOM;) {
            while(y<=EP_BOTTOM && !ep_white(raw,thr,x,y))++y;
            int a=y;
            while(y<=EP_BOTTOM && ep_white(raw,thr,x,y))++y;
            int b=y-1;
            if(a<=EP_TOP+2 || b>=EP_BOTTOM-2 || b-a+1<20)continue;
            if(best_b>=0)return 0; /* Multiple eligible side branches. */
            best_a=a;best_b=b;
        }
        if(best_b<0)break;
        if(n && (ip_abs(best_a-top[n-1])>4 || ip_abs(best_b-bottom[n-1])>10))break;
        top[n]=best_a;bottom[n]=best_b;++n;
    }
    float goal_y=0,width=0,slope=0;
    if(n==5) {
        for(int k=0;k<5;++k){goal_y+=(top[k]+bottom[k])/10.0f;width+=(bottom[k]-top[k]+1)/5.0f;}
        slope=((top[0]+bottom[0])-(top[4]+bottom[4]))/16.0f;
        *paired=1;
    } else {
        if(!allow_outer)return 0;
        int prev=-1,first=-1,last=-1,observed=0;
        /* Trace one continuous side ridge inward. Its dark component must
         * also extend toward the middle of the current image. */
        for(int k=0;k<=24;++k) {
            int x=guide<0 ? 182-2*k:5+2*k;
            int edge=ip_turn_bank(raw,thr,x);
            if(edge<0)break;
            if(prev>=0 && ip_abs(edge-prev)>4)return 0;
            if(first<0)first=edge;
            prev=last=edge;++observed;
        }
        if(observed<17 || first<28 || first>66 || ip_abs(last-first)>20 ||
           !ip_outer_region(raw,guide,first))return 0;
        /* Crop boundaries do not measure road width. A bounded provisional
         * 48px normal width is tagged inferred; it is not a calibration. */
        width=48.0f;goal_y=first+width*0.5f;
        slope=(first-last)/(2.0f*(observed-1));n=observed;
    }
    if(goal_y<40 || goal_y>84 || width<20 || width>80)return 0;
    entry_context_t saved=ep_context;
    ep_context.active=0; /* branch association above uses this frame's columns */
    memset(out,0,sizeof *out);
    out->points[0].x=94;out->points[0].y=EP_BOTTOM;
    out->points[1].x=(uint8_t)(guide<0 ? 182:5);out->points[1].y=(uint8_t)(goal_y+0.5f);
    out->count=2;out->goal_x=out->points[1].x;out->goal_y=out->points[1].y;
    out->estimated_width=width;out->boundary_source=(uint8_t)(*paired ? 1:2);
    int ok=ep_fit_curve(raw,thr,guide,slope,out);
    ep_context=saved;
    if(!ok){out->count=0;return 0;}
    out->support=(uint8_t)n;out->inferred=(uint8_t)!*paired;
    ep_measure_edges(raw,thr,out);
    int pv=0;
    return ip_curve_clearance(out,guide) && ip_path_controls(out,&pv) && pv;
}

/* Track the OUTER-bank convex tip at the unique fixed exit. Work in a
 * mirrored coordinate so clockwise/anticlockwise share exactly one detector.
 * A dark component must touch >=6 actual near-bank observations; a speckle,
 * image edge, monotone bend or detached distant stripe is not an exit. */
static int ip_exit_tip(const uint8_t *raw,const uint8_t *thr,
                       const int16_t *bank,int guide,int *tip_x,int *tip_y)
{
    int best_votes=0,best_x=0,best_y=0;
    memset(s_exterior,0,sizeof s_exterior);
    for(int seed_y=IP_TRACK_BOTTOM;seed_y>=85;--seed_y) {
        int edge=bank[seed_y];
        if(edge<=2 || edge>=IP_W-3)continue;
        int seed_x=edge+guide;
        if(raw[seed_y*IP_W+seed_x]>thr[seed_y])continue;
        if(s_exterior[seed_y][seed_x>>3]&(1U<<(seed_x&7)))continue;
        int head=0,tail=0,outline[IP_H],votes=0;
        for(int y=0;y<IP_H;++y)outline[y]=-1;
        ip_exterior_add(raw,thr,seed_x,seed_y,&tail);
        while(head<tail) {
            int idx=s_exterior_queue[head++],x=idx%IP_W,y=idx/IP_W;
            int mirrored=guide<0 ? x:IP_W-1-x;
            if(mirrored>outline[y])outline[y]=mirrored;
            if(y>=85 && bank[y]>2 && bank[y]<IP_W-3 && x==bank[y]+guide)++votes;
            for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
                int xx=x+dx,yy=y+dy;
                if(xx>=0 && xx<IP_W && yy>=IP_TRACK_TOP && yy<=IP_TRACK_BOTTOM)
                    ip_exterior_add(raw,thr,xx,yy,&tail);
            }
        }
        if(votes<6 || votes<=best_votes)continue;
        int y_tip=IP_TRACK_TOP;
        for(int y=IP_TRACK_TOP+1;y<=IP_TRACK_BOTTOM;++y)
            if(outline[y]>outline[y_tip])y_tip=y;
        int x_tip=outline[y_tip],upper=x_tip,lower=x_tip,up_count=0,down_count=0;
        if(x_tip<8 || x_tip>=IP_W-3 || y_tip<32 || y_tip>92)continue;
        for(int d=6;d<=12;++d) {
            if(y_tip-d>=IP_TRACK_TOP && outline[y_tip-d]>=0) {
                if(outline[y_tip-d]<upper)upper=outline[y_tip-d];++up_count;
            }
            if(y_tip+d<=IP_TRACK_BOTTOM && outline[y_tip+d]>=0) {
                if(outline[y_tip+d]<lower)lower=outline[y_tip+d];++down_count;
            }
        }
        if(up_count<4 || down_count<4 || x_tip-upper<10 || x_tip-lower<10)continue;
        best_votes=votes;best_x=guide<0 ? x_tip:IP_W-1-x_tip;best_y=y_tip;
    }
    if(!best_votes)return 0;
    *tip_x=best_x;*tip_y=best_y;return 1;
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
    memset(&ep_context, 0, sizeof ep_context);
    s_entry_lost_frames = s_entry_reacquire = 0;
    memset(s_half, 0, sizeof s_half);
    memset(s_last_center, 0xff, sizeof s_last_center);
    s_half_ready = 0;
    s_half_bias = 0;
    s_half_bias_ready = 0;
    s_have_error = 0;
    s_scene_ok = s_scene_contrast = 0;
    s_valid = 0;
    s_error = 0;
    s_preview_error = 0;
    s_preview_valid = 0;
    s_usable_rows = 0;
    s_both_rows = 0;
    s_single_rows = 0;
    s_stripe_rows = s_stripe_recovery = 0;
    s_ring_candidate = s_near_valid = s_ring_preview_valid = 0;
    s_road_recovered = 0;
    s_ring_evidence = s_ring_corner_row = s_ring_apex_row = 0;
    s_ring_miny = s_ring_maxy = 0;
    s_ring_entry_error = 0;
    s_ring_guide = 0;
    s_entry_mode = s_entry_locked = s_entry_status = s_orbit_ready = 0;
    s_align_turn_bias = 0;
    s_entry_weight=0;s_entry_weight_tick=s_entry_control_tick=0;s_entry_inside=s_entry_control_seen=0;
    s_exit_tip_row=s_exit_tip_x=0;
    s_entry_ready=s_exit_ready=s_exit_search=0;
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
void img_set_exit_search(uint8_t enabled) { s_exit_search=enabled; }
/* Side is measured at acquisition; ambiguous centered islands are not a
 * direction command. Cross-island arbitration remains in roundabout_scan. */
int img_get_roundabout_side(void) {
    if(!s_ring_candidate || s_ring_hold_age || !s_ring_miny)return 0;
    int row=(s_ring_miny+s_ring_maxy)/2;
    if(s_ring_mask[row]==1)return 1;
    if(s_ring_mask[row]==2)return -1;
    int center=((int)s_ring_l[row]+s_ring_r[row])/2;
    return center>IP_W/2+8 ? 1:(center<IP_W/2-8 ? -1:0);
}
uint8_t img_get_orbit_ready(void) { return s_orbit_ready; }
uint8_t img_get_entry_ready(void) { return s_entry_ready; }
uint8_t img_get_entry_path_safe(void) {
    return s_valid && s_near_valid && s_entry_path.count>=12 &&
        ip_abs(s_entry_path.error)<=42 && s_entry_status==1 && s_entry_path.support>=5 &&
        ip_curve_clearance(&s_entry_path,s_ring_guide);
}
uint8_t img_get_exit_ready(void) { return s_exit_ready; }
int16_t img_get_exit_error(void) { return s_preview_error; }
uint8_t img_get_exit_tip_row(void) { return s_exit_tip_row; }
uint8_t img_get_exit_tip_x(void) { return s_exit_tip_x; }
void img_set_roundabout_entry(uint8_t enabled) {
    if (enabled != s_entry_mode) {
        s_align_turn_bias = 0;
        if(enabled==1){s_entry_weight=0;s_entry_weight_tick=s_odometry_tick;s_entry_inside=s_entry_control_seen=0;}
        ip_bridge_reset();
        s_exit_tip_row=s_exit_tip_x=0;
        s_entry_lost_frames=s_entry_reacquire=0;
        if(enabled==3)s_entry_locked=1;
    }
    if (!enabled) {
        memset(&ep_context, 0, sizeof ep_context);
        s_entry_lost_frames = s_entry_reacquire = 0;
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
float img_get_align_turn_bias(void) { return s_align_turn_bias; }
uint8_t img_get_roundabout_candidate(void) { return s_ring_candidate; }
uint8_t img_get_roundabout_evidence(void) { return s_ring_evidence; }
const uint8_t *img_get_row_thresholds(void) { return s_row_thr; }
uint8_t img_get_scene_contrast(void) { return s_scene_contrast; }
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
uint8_t img_get_single_rows(void) { return s_single_rows; }
uint8_t img_get_near_run(void)    { return s_near_run; }

void auxiliary_process(uint8_t *_src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols,
                       unsigned char _threshold_val,
                       uint8_t *_left_line, uint8_t *_mid_line, uint8_t *_right_line)
{
    s_align_turn_bias = 0;
    static uint32_t band_hist[256];
    uint8_t  band_thr[IP_BANDS];
    uint8_t  band_bright_supported[IP_BANDS] = {0};
    uint8_t  row_thr[IP_H];
    uint8_t  bin[IP_W];
    int      band_rows = (IP_H + IP_BANDS - 1) / IP_BANDS;
    int      seed, y, usable = 0, both = 0, single_rows = 0;
    long     err_sum = 0;
    int      err_n = 0;
    long     preview_sum = 0;
    int      preview_n = 0;
    int      near_rows = 0;
    uint8_t measured_pair[IP_H] = {0};
    uint8_t measured_center[IP_H] = {0};
    int16_t outer_bank[IP_H];
    for(int row=0;row<IP_H;++row)outer_bank[row]=-1;
    int measured_l[IP_H] = {0}, measured_r[IP_H] = {0};
    s_road_recovered = 0;
    s_entry_ready=s_exit_ready=s_orbit_ready=0;
    s_exit_tip_row=s_exit_tip_x=0;
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
            uint32_t cumulative = 0;
            int p05 = lo, p95 = hi, got_low = 0;
            for (int value = lo; value <= hi; ++value) {
                cumulative += band_hist[value];
                if (!got_low && cumulative*100U >= total*5U) { p05 = value; got_low = 1; }
                if (cumulative*100U >= total*95U) { p95 = value; break; }
            }
            /* A few dark speckles cannot turn a uniformly bright road band
             * into a measured boundary. Check the supported intensity spread. */
            if (p95-p05 < 16 && p05 > band_thr[b-1]) band_thr[b] = band_thr[b-1];
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
        s_scene_contrast = (uint8_t)(hi - lo);
        s_scene_ok = (uint8_t)(s_scene_contrast >= IP_MIN_DR);
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
    if(s_entry_mode==1 || s_entry_mode==2 || s_entry_mode==3) {
        memset(s_approach_side,0,sizeof s_approach_side);
        s_approach_active=s_approach_width_side=0;
    } else ip_approach_bridge(_src_pixel_mat, row_thr);
    /* Retain island topology for exit recognition; it must not own the
     * ordinary row follower during ORBIT. */
    int row_guide=s_entry_mode ? 0:s_ring_guide;
    int row_island=s_entry_mode ? 0:s_ring_candidate;

    if(s_entry_mode)ip_road_component(_src_pixel_mat,row_thr);

    /* Pre-measure branch dual-edge half-width delta on the guided island rows
     * so single-edge branch rows (where outer edge leaves FOV) use the true
     * branch width rather than jumping to full-track half-width. */
    if ((row_guide || row_island) && s_ring_miny && s_ring_maxy >= s_ring_miny) {
        int g = row_guide ? row_guide : ((s_ring_mask[s_ring_miny] == 2) ? 1 : -1);
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
    for (y = 0; y < IP_H; ++y) { _left_line[y] = 0; _right_line[y] = 0; _mid_line[y] = 0; }

    for (y = IP_TRACK_BOTTOM; y >= IP_TRACK_TOP; --y) {
        const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
        uint8_t thr = row_thr[y];
        int l, r, width;
        int ref = (y < IP_H - 1 && s_last_center[y + 1] >= 0) ? s_last_center[y + 1] : IP_W / 2;

        if (s_stripe_rows || row_island || row_guide || s_approach_side[y]) continue;

        for (int x = 0; x < IP_W; ++x) bin[x] = (uint8_t)(src[x] > thr);
        if (!ip_row_run(bin, src, thr, -1, ip_half(y), ref, &l, &r)) continue;
        if(s_entry_mode && (!ip_on_road((l+r)/2,y) || !ip_on_road(l,y) || !ip_on_road(r,y)))continue;
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
        if (!s_stripe_rows && !row_island && !row_guide && diff_n >= 8) {
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
        int single_edge = 0;

        if (s_stripe_mask[y]) {
            if (!ip_stripe_edges(src, thr, y, &l, &r)) continue;
            center = (l+r)/2;
            if (s_last_center[y] >= 0 && ip_abs(center-s_last_center[y]) > IP_JUMP_MAX)
                continue;
            _left_line[y] = (uint8_t)l; _right_line[y] = (uint8_t)r;
            _mid_line[y] = (uint8_t)center;
            ++usable; seed = center;
            measured_center[y] = 1;
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
            if (s_entry_mode != 5 && s_entry_mode != 3 && row_guide && !s_approach_active && (s_ring_mask[y] & (row_guide<0 ? 1:2)) && !s_stripe_recovery) {
                in_guided_branch = 1;
                if(row_guide<0) {
                    r=s_ring_l[y];l=r;while(l>0 && bin[l-1])--l;
                }else{
                    l=s_ring_r[y];r=l;while(r<IP_W-1 && bin[r+1])++r;
                }
            } else if (!ip_row_run(bin, src, thr, pred_left, ip_half(y), ref, &l, &r)) continue;
        }
        if (r - l + 1 < IP_RUN_MIN) continue;

        width = r - l + 1;
        raw_l=l; raw_r=r;
        outer_bank[y]=(int16_t)(s_ring_guide<0 ? raw_l:raw_r);
        /* Count actual binary banks before gradient substitutions or synthetic
         * approach edges. A crop edge is not an observed track boundary. */
        single_edge=((raw_l>2) != (raw_r<IP_W-3)) && !s_approach_side[y];
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
            if ((in_guided_branch || (row_island && s_ring_mask[y])) && have_branch_hw_delta) {
                half = ip_clamp(half + branch_hw_delta, 16, half);
            }
            if (width > half * 2 + half) {
                if(row_guide || row_island)continue;
                l = ip_clamp(mid - half, 1, IP_W - 2);
                r = ip_clamp(mid + half, 1, IP_W - 2);
                width = r - l + 1;
                left_ok  = (l > 2);
                right_ok = (r < IP_W - 3);
            }
            /* One bank outside the field of view is ordinary at a ring throat,
             * an exit merge and any wide opening: the road is simply wider than
             * the camera. The visible bank plus this row's calibrated half width
             * still locates the centre, so keep the row as a measured
             * single-edge observation instead of discarding the whole near
             * corridor. It stays distinguishable from a pair (left_ok &&
             * right_ok) and never teaches the width model. */
            if (!left_ok && !right_ok) {
                int left_clipped  = raw_l <= 2;
                int right_clipped = raw_r >= IP_W - 3;
                if (left_clipped && right_clipped) continue; /* wider than the FOV */
                if (left_clipped) {
                    center = raw_r - half;
                    if (center >= raw_r - 2) continue;
                } else {
                    center = raw_l + half;
                    if (center <= raw_l + 2) continue;
                }
                if (center < 2 || center >= IP_W - 2) continue;
                l = center - half; if (l < 0) l = 0;
                r = center + half; if (r > IP_W - 1) r = IP_W - 1;
                if (!bin[center]) continue;
            } else if (left_ok && right_ok) {
                center = mid;
                if (in_guided_branch || (row_island && s_ring_mask[y])) {
                    branch_hw_delta = ip_clamp(width / 2 - ip_half(y), -20, 0);
                    have_branch_hw_delta = 1;
                }
            } else if (left_ok) {
                /* Only the right bank is missing: keep the visible left bank
                 * and use the calibrated half width as an ordinary bend. */
                center = l + half;
            } else {
                /* Only the left bank is missing: mirror the same one-bank
                 * fallback around the visible right bank. */
                center = r - half;
            }
        }

        if (center < 2 || center >= IP_W - 2) continue;
        if(s_entry_mode && !ip_on_road(center,y))continue;
        if((row_island || row_guide) && !bin[center])continue;
        /* A verified current bridge replaces the old opening-biased center. */
        if (!in_guided_branch && !s_approach_side[y] &&
            s_last_center[y] >= 0 && ip_abs(center - s_last_center[y]) > IP_JUMP_MAX) {
            continue;
        }
        /* Exit evidence uses only measured binary transitions, before gradient
         * substitutes, width clipping or short-gap interpolation. */
        if (!in_guided_branch && !s_approach_side[y] && raw_l>2 && raw_r<IP_W-3 &&
            ip_width_matches(y,raw_r-raw_l+1)) {
            measured_pair[y]=1; measured_l[y]=raw_l; measured_r[y]=raw_r;
        }
        /* Zero marks an unobserved bank. The crop edge is never a track edge. */
        _left_line[y]  = (uint8_t)(l>2 && s_approach_side[y]!=1 ? l : 0);
        _right_line[y] = (uint8_t)(r<IP_W-3 && s_approach_side[y]!=2 ? r : 0);
        _mid_line[y]   = (uint8_t)center;
        ++usable;
        if (single_edge) ++single_rows;
        measured_center[y] = 1;
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
    if (!row_island && (s_entry_mode==2 || s_entry_mode==3 || !s_ring_miny) && !s_stripe_recovery) {
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

    /* Cache current row targets before gap interpolation or ordered planning.
     * Generated targets must not become next-frame boundary evidence. */
    for (y = 0; y < IP_H; ++y)
        s_last_center[y] = _mid_line[y] ? (int16_t)_mid_line[y] : -1;

    s_ring_entry_error = 0;

    /* ---- 3. 本帧统计与有效性 ----
     * Validity answers "can I steer from this frame", which is a question about
     * the near control corridor, not about how many rows happened to resolve.
     * At a ring throat the road fills the lower image (frame 125 of run02: rows
     * 88..93 are saturated 254 across the full width) so whole-image row count
     * collapses while the near corridor is still measured on the visible bank.
     * Counting rows over the full ROI therefore reported "no vision" and
     * commanded a stop with steering data still available. Require instead a
     * continuous, current near-field corridor, and keep the row total only as a
     * secondary check for ordinary far-field-only images. */
    s_usable_rows = (uint8_t)(usable > 255 ? 255 : usable);
    s_both_rows   = (uint8_t)(both > 255 ? 255 : both);
    s_single_rows = (uint8_t)(single_rows > 255 ? 255 : single_rows);
    s_diag_near_rows = (uint8_t)near_rows;
    {
        /* Longest unbroken run of near rows, measured from the bottom up, using
         * only rows carrying a current centre. Interpolated rows are filled in
         * after this point, so they can never satisfy the run. */
        int run = 0, best_run = 0;
        for (y = IP_ERROR_ROW_BOT; y >= IP_ERROR_ROW_TOP; --y) {
            if (!measured_center[y]) { run = 0; continue; }
            if (++run > best_run) best_run = run;
        }
        s_near_run = (uint8_t)best_run;
        int near_ok = best_run >= 8;
        int rows_ok = usable >= IP_VALID_ROWS_MIN;
        /* A wide throat makes the far field legitimately unparsable; the near
         * run then carries the frame. On an ordinary road both still hold. */
        s_valid = (uint8_t)(s_scene_ok && (rows_ok || (near_ok && usable >= IP_VALID_ROWS_MIN / 2)));
    }
    /* Near validity remains tied to current near rows; single-bank mode
     * changes the path choice, not the scene-quality gate. */
    s_near_valid = (uint8_t)(near_rows >= 6);
    if ((s_stripe_rows || row_island || row_guide) && !s_near_valid) s_valid = 0;
    /* Ordinary preview must belong to the current near corridor. Averaging
     * every far row mixed in a different branch after an edge disappeared. */
    if(!row_guide) {
        int previous=-1, previous_y=0, gap=0;
        preview_sum=0;preview_n=0;
        for(int row=IP_TRACK_BOTTOM;row>=55;--row) {
            if(!measured_center[row]) {
                if(++gap>3)break;
                continue;
            }
            int center=_mid_line[row];
            if(previous>=0) {
                if(ip_abs(center-previous)>16)break;
                int connected=1;
                for(int yy=row;yy<=previous_y;++yy) {
                    int xx=center+(previous-center)*(yy-row)/(previous_y-row);
                    if(!ep_white(_src_pixel_mat,row_thr,xx,yy)){connected=0;break;}
                }
                if(!connected)break;
            }
            gap=0;previous=center;previous_y=row;
            if(row<=75){preview_sum+=center;++preview_n;}
        }
    }
    s_preview_valid = (uint8_t)(s_valid && preview_n >= 10 && !s_stripe_recovery);
    /* Guided preview must use the selected inner-edge corridor itself. The old
     * code validated branch rows but averaged unrelated straight-road rows.
     * Use a short measured window at the far end of the visible branch; never
     * extrapolate a missing island or authorize driving without near edges. */
    {
        int total_branch_rows = 0;
        long branch_sum = 0;
        if (row_guide && !s_approach_active) {
            int end = ip_clamp((int)s_ring_miny+19,IP_TRACK_TOP,IP_TRACK_BOTTOM);
            for (y = s_ring_miny; s_ring_miny && y <= end; ++y)
                if (_mid_line[y] && (s_ring_mask[y] & (row_guide < 0 ? 1 : 2))) {
                    ++total_branch_rows;branch_sum+=_mid_line[y];
                }
            preview_n=total_branch_rows;preview_sum=branch_sum;
            s_preview_valid=(uint8_t)(s_valid && s_near_valid && total_branch_rows>=10 && !s_stripe_recovery);
        }
        s_ring_preview_valid = (uint8_t)(!s_approach_active && row_guide && s_preview_valid && total_branch_rows>=10);
    }
    if (s_preview_valid)
        s_preview_error = (int16_t)ip_clamp((int)(preview_sum/preview_n)-IP_W/2,-IP_W/2,IP_W/2);
    else s_preview_error = 0;
    if (s_valid) {
        int e = err_n ? (int)(err_sum / err_n) : (IP_W / 2);
        e -= IP_W / 2;
        if (s_have_error) {
            int max_step = row_guide ? 28 : 16;
            if (e >  s_error + max_step) e = s_error + max_step;
            if (e <  s_error - max_step) e = s_error - max_step;
        }
        if (e > -3 && e < 3) e = 0;
        s_error = (int16_t)ip_clamp(e, -IP_W / 2, IP_W / 2);
        s_have_error = 1;
    }

    /* Fill only short, currently white gaps between measured centres.
     * Missing banks remain unknown and interpolation never becomes history. */
    for (y = IP_TRACK_TOP+1; y < IP_TRACK_BOTTOM; ++y) {
        if (_mid_line[y] || !_mid_line[y-1]) continue;
        int end=y;
        while(end<=IP_TRACK_BOTTOM && !_mid_line[end])++end;
        if(end>IP_TRACK_BOTTOM || end-y>3)continue;
        int first=y-1, a=_mid_line[first], b=_mid_line[end], valid=1;
        for(int row=y;row<end;++row) {
            int x=a+(b-a)*(row-first)/(end-first);
            if(!ep_white(_src_pixel_mat,row_thr,x,row))valid=0;
        }
        if(!valid)continue;
        for(int row=y;row<end;++row)
            _mid_line[row]=(uint8_t)(a+(b-a)*(row-first)/(end-first));
        y=end;
    }
    /* Entrance branch owns the control path after its first visible side
     * opening. Every target comes from the current binary image. */
    if (s_entry_mode && s_ring_guide && !s_stripe_recovery) {
        int raw_valid=s_valid && s_near_valid;
        for (int row = 0; row < IP_H; ++row) s_entry_half_width[row] = (uint8_t)ip_half(row);
        entry_context_t context = {0};
        context.active = 1;
        context.ring_phase = s_entry_mode;
        /* Guided split rows can include disconnected background marks. Use
         * one coherent current contour, never the union of unrelated rows. */
        int chain = 0, best_chain = 0, best_end = 0;
        memset(s_entry_island_mask,0,sizeof s_entry_island_mask);
        for (int row = EP_TOP; row <= EP_BOTTOM; ++row) {
            if (!(s_ring_mask[row] & (s_ring_guide < 0 ? 1 : 2))) { chain = 0; continue; }
            if (chain && (ip_abs((int)s_ring_l[row]-s_ring_l[row-1]) > 8 ||
                          ip_abs((int)s_ring_r[row]-s_ring_r[row-1]) > 8)) chain = 0;
            ++chain;
            if (chain > best_chain) { best_chain = chain; best_end = row; }
        }
        /* A current contiguous island contour selects the entrance branch. */
        context.fresh_island = (uint8_t)(s_ring_miny && best_chain>=12 && !s_ring_hold_age);
        if (context.fresh_island)
            for (int row = best_end-best_chain+1; row <= best_end; ++row)
                s_entry_island_mask[row] = s_ring_mask[row];
        context.island_left = s_ring_l; context.island_right = s_ring_r;
        context.island_mask = s_entry_island_mask; context.half_width = s_entry_half_width;
        context.tick = s_odometry_tick; context.distance_m = s_odometry_distance;
        context.odometry_valid = s_odometry_valid;
        s_orbit_ready=(uint8_t)(s_valid && s_near_valid && s_near_run>=8 &&
            context.fresh_island && best_end>=IP_TRACK_BOTTOM-2 &&
            best_end-best_chain+1<=75);
        ep_set_context(&context);
        s_entry_path.count = 0;
        static entry_path_t ordinary,turn,transition;
        entry_context_t saved=ep_context;
        if(s_entry_mode==2 || s_entry_mode==3)ep_context.fresh_island=0;
        int ordinary_ok=s_scene_ok && ep_build_measured_corridor(_src_pixel_mat,row_thr,s_ring_guide,94,&ordinary);
        ep_context=saved;
        int ordinary_pv=0;
        if(ordinary_ok)ordinary_ok=ip_path_controls(&ordinary,&ordinary_pv);
        if(ordinary_ok && (s_entry_mode==1 || s_entry_mode==5))
            ordinary_ok=ip_curve_clearance(&ordinary,s_ring_guide);
        /* Exit recognition retains raw measured banks before any curve output. */
        if((s_entry_mode==2 || s_entry_mode==3) && s_exit_search && raw_valid) {
            int tx,ty;
            if(ip_exit_tip(_src_pixel_mat,row_thr,outer_bank,s_ring_guide,&tx,&ty)) {
                s_exit_tip_row=(uint8_t)ty;s_exit_tip_x=(uint8_t)tx;s_exit_ready=1;
            }
        }
        if(ordinary_ok && raw_valid)ip_publish_curve(&ordinary,_left_line,_mid_line,_right_line);
        if(s_entry_mode==2) {
            /* Reject a partial/reversed orbit route instead of exposing the raw
             * scan rows that the directional curve fitter just discarded. */
            if(!ordinary_ok || !ordinary_pv || !raw_valid) {
                s_entry_path.count=0;s_valid=s_near_valid=s_preview_valid=0;
                s_preview_error=0;
                memset(_left_line,0,IP_H);memset(_mid_line,0,IP_H);memset(_right_line,0,IP_H);
            }
            s_orbit_ready=(uint8_t)(s_valid && s_near_valid && s_preview_valid && !s_exit_ready);
            s_entry_status=s_entry_locked=0;ip_bridge_reset();return;
        }
        if(s_entry_mode==3) {
            int paired=0;
            if(s_scene_ok && !s_road_recovered && ip_turn_curve(_src_pixel_mat,row_thr,s_ring_guide,1,&turn,&paired)) {
                ip_publish_curve(&turn,_left_line,_mid_line,_right_line);s_entry_status=1;
            } else s_entry_status=0;
            s_entry_locked=0;ip_bridge_reset();return;
        }
        int paired=0;
        int turn_ok=s_scene_ok && ip_turn_curve(_src_pixel_mat,row_thr,s_ring_guide,s_entry_mode==1,&turn,&paired);
        uint8_t ordinary_mid[IP_H]={0};
        if(ordinary_ok)for(int row=55;row<=105;++row){float x;if(ip_path_row(&ordinary,row,&x))ordinary_mid[row]=(uint8_t)(x+0.5f);}
        if(s_entry_mode==5) {
            /* A lateral passage must reach y49 on two actual observations.
             * Its near-side island touching row105 alone is not entry. */
            s_entry_ready=(uint8_t)(turn_ok && paired && turn.goal_y>=49 && ordinary_ok && raw_valid &&
                ip_entry_transition(_src_pixel_mat,row_thr,ordinary_mid,0,&turn,&transition) &&
                ip_abs(transition.error-s_error)<=4 && ip_abs(transition.preview-s_preview_error)<=12);
            if(ordinary_ok && raw_valid && ip_bias_align_curve(_src_pixel_mat,row_thr,s_ring_guide,&ordinary))
                ip_publish_curve(&ordinary,_left_line,_mid_line,_right_line);
            s_entry_status=s_entry_locked=s_orbit_ready=0;
            ip_bridge_reset();return;
        }
        int fresh=s_odometry_tick!=s_entry_weight_tick;
        int sign=s_ring_guide<0 ? 1:-1;
        int follows_side=ordinary_ok && (ordinary.goal_x-94)*sign>=16;
        if(follows_side)for(int i=1;i<ordinary.count;++i)
            if((ordinary.points[i].x-ordinary.points[i-1].x)*sign< -2)follows_side=0;
        int current_bend=follows_side && ordinary_pv && raw_valid && ordinary.preview*sign>=8;
        /* The same 200 ms freshness limit as steering: an old target may not
         * permanently block reacquisition after a camera interruption. */
        int continuous=!s_entry_control_seen || s_odometry_tick-s_entry_control_tick>STEERING_MAX_UPDATE_TICKS ||
            (ip_abs(ordinary.error-s_entry_control_error)<=4 &&
             ip_abs(ordinary.preview-s_entry_control_preview)<=12);
        int joins=current_bend && continuous && (!turn_ok ||
            (turn_ok && ip_abs(ordinary.error-turn.error)<=4 &&
                        ip_abs(ordinary.preview-turn.preview)<=12));
        if(fresh){if(joins){if(s_entry_inside<3)++s_entry_inside;}else s_entry_inside=0;}
        float next=s_entry_weight;
        if(fresh && next<1){next+=IP_ENTRY_BLEND_STEP;if(next>1)next=1;}
        int built=0;
        if(joins){transition=ordinary;built=1;}
        else if(turn_ok && ordinary_ok && next<0.99f)
            built=ip_entry_transition(_src_pixel_mat,row_thr,ordinary_mid,next,&turn,&transition);
        else if(turn_ok){transition=turn;built=1;}
        /* A short loss of side-bank evidence does not erase a measured road.
         * It cannot confirm ORBIT without current directional bend evidence. */
        if(!built && ordinary_ok && raw_valid && continuous){transition=ordinary;built=1;}
        /* After two lost images, require two distinct current observations.
         * The last VALID target has its own timestamp; failed processing may
         * never refresh it and permanently block reacquisition. */
        if(built && s_entry_lost_frames>=2) {
            if(fresh && s_entry_reacquire<2)++s_entry_reacquire;
            if(s_entry_reacquire<2)built=0;
            else s_entry_lost_frames=s_entry_reacquire=0;
        } else if(built) s_entry_lost_frames=s_entry_reacquire=0;
        else if(fresh) {
            s_entry_reacquire=0;
            if(s_entry_lost_frames<255)++s_entry_lost_frames;
        }
        s_entry_weight_tick=s_odometry_tick;s_orbit_ready=0;
        if(built) {
            s_entry_weight=next;
            ip_publish_curve(&transition,_left_line,_mid_line,_right_line);
            s_orbit_ready=(uint8_t)(s_entry_inside>=3);
            s_entry_status=1;s_entry_locked=0;s_entry_control_seen=1;
            s_entry_control_error=s_error;s_entry_control_preview=s_preview_error;s_entry_control_tick=s_odometry_tick;
        } else {
            s_entry_inside=0;s_entry_status=3;s_entry_locked=1;s_entry_path.count=0;
            s_valid=s_near_valid=s_preview_valid=0;s_preview_error=0;
            memset(_left_line,0,IP_H);memset(_mid_line,0,IP_H);memset(_right_line,0,IP_H);
        }
        ip_bridge_reset();
    } else {
        ip_bridge_reset();
        s_entry_status = 0;
        s_entry_path.count = 0;
    }

}
