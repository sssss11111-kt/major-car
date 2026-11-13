#ifndef CROSS_STRAIGHT_H
#define CROSS_STRAIGHT_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Straight-through crossing, using current visible approach or departure banks. */
#define CS_TOP 25
#define CS_BOTTOM 105
#define CS_MIN_SUPPORT 10
#define CS_OPEN_ROWS 4
#define CS_RECOVER_FRAMES 3
#define CS_HOLD_TICKS 100u

typedef struct {
    uint8_t active, valid, opening, recovered, source, support;
    uint8_t clear_frames, gap_top, gap_bottom, hold_frames;
    float left[2], right[2]; /* x = intercept + slope * y */
    uint8_t measured[120], center[120];
    uint8_t measured_left[120], measured_right[120];
    uint32_t support_tick, bend_tick;
    int8_t bend_side;
    uint8_t bend_frames, bend_released, outer_count, bend_follow, exit_frames, exit_white_rows, exit_white_seen;
    uint8_t outer_x[96], outer_y[96];
    uint8_t bend_left[120], bend_right[120];
    int16_t near_error, preview_error;
} cross_straight_t;

const cross_straight_t *img_get_cross_straight(void);

static inline int cs_run(const uint8_t *raw, const uint8_t *thresholds, int y,
                  int seed, int *left, int *right)
{
    const uint8_t *row = raw + y * 188;
    if (seed < 3 || seed > 184 || row[seed] <= thresholds[y]) return 0;
    int l = seed, r = seed;
    while (l > 0 && row[l - 1] > thresholds[y]) --l;
    while (r < 187 && row[r + 1] > thresholds[y]) ++r;
    *left = l; *right = r;
    return r - l >= 30;
}

static inline void cs_fit(const int *values, int first, int last, float out[2])
{
    float n = (float)(last - first + 1), sy = 0, sx = 0, syy = 0, syx = 0;
    for (int y = first; y <= last; ++y) {
        sy += y; sx += values[y]; syy += y * y; syx += y * values[y];
    }
    out[1] = (n * syx - sy * sx) / (n * syy - sy * sy);
    out[0] = (sx - out[1] * sy) / n;
}

static inline int cs_anchor(const int *left, const int *right, const uint8_t *paired,
                     int *first, int *last, float fit_left[2], float fit_right[2])
{
    int chain = 0, best = 0;
    for (int y = CS_TOP; y <= CS_BOTTOM; ++y) {
        if (!paired[y]) { chain = 0; continue; }
        if (chain && (abs(left[y] - left[y - 1]) > 2 ||
                      abs(right[y] - right[y - 1]) > 2)) chain = 0;
        ++chain;
        /* Prefer a long coherent segment. Near support wins a same-length tie. */
        if (chain >= CS_MIN_SUPPORT && chain >= best) {
            float l[2], r[2];
            int start = y - chain + 1, good = 1;
            cs_fit(left, start, y, l); cs_fit(right, start, y, r);
            if (fabsf(l[1]) > 2 || fabsf(r[1]) > 2 || r[1] - l[1] < -0.3f ||
                r[1] - l[1] > 2.5f) continue;
            for (int row = start; row <= y; ++row)
                if (fabsf(left[row] - l[0] - l[1] * row) > 3 ||
                    fabsf(right[row] - r[0] - r[1] * row) > 3) good = 0;
            if (!good) continue;
            best = chain; *first = start; *last = y;
            memcpy(fit_left, l, sizeof l); memcpy(fit_right, r, sizeof r);
        }
    }
    return best;
}

/* Trace the upper outer bank in column order. A column can still measure
 * the bank once it has turned too far for a row scan to represent it. */
static inline int cs_outer_bank(cross_straight_t *s, const uint8_t *raw,
                               const int *left, const int *right, int side)
{
    const int *bank = side > 0 ? left : right;
    int y = bank[64] >= 8 && bank[64] <= 179 ? 64 : 55;
    int seed = bank[y], x = seed, count = 0, supported = 0, far = 0;
    if (x < 8 || x > 179) return 0;
    for (int step = 0; step < 96 && x >= 4 && x <= 183; ++step, x += side * 2) {
        int best = -1, score = 0;
        for (int row = y - 7; row <= y + 2; ++row) {
            if (row < 14 || row > 70) continue;
            int gradient = 0;
            for (int d = 2; d <= 4; ++d)
                gradient += raw[(row+d)*188+x] - raw[(row-d)*188+x];
            int value = gradient - 2 * abs(row-y);
            if (gradient >= 23 && value > score) { best = row; score = value; }
        }
        if (best < 0) {
            if (++far > 2) break;
            continue;
        }
        far = 0;
        y = best;
        s->outer_x[count] = (uint8_t)x;
        s->outer_y[count++] = (uint8_t)y;
        if (y <= 34 && abs(x - seed) >= 45) ++supported;
    }
    if (count < 30 || supported < 10 || (count && abs(s->outer_x[count-1] - seed) < 90) || y > 33) return 0;
    return count;
}

static inline void cs_update(cross_straight_t *s, const uint8_t *raw,
                      const uint8_t *thresholds, uint32_t tick, int enabled)
{
    int left[120] = {0}, right[120] = {0};
    uint8_t paired[120] = {0};
    cross_straight_t previous = *s;
    uint8_t was_active = previous.active, clear = previous.clear_frames;
    memset(s, 0, sizeof *s);
    if (!enabled) return;
    s->active = was_active;
    for (int y = CS_TOP; y <= CS_BOTTOM; ++y) {
        if (cs_run(raw, thresholds, y, 94, &left[y], &right[y]) &&
            left[y] > 3 && right[y] < 184 && right[y] - left[y] >= 40)
            paired[y] = 1;
    }
    /* Measure a connected white span outside both projected banks.
       The central stem supplies the reference width for the opening. */
    float exit_left[2], exit_right[2];
    int exit_first = 0, exit_last = 0;
    int exit_support = cs_anchor(left, right, paired, &exit_first, &exit_last,
                                 exit_left, exit_right);
    int white_chain = 0, near_white = 0, bridge_chain = 0, bridge_ready = 0;
    if (exit_support) {
        for (int y = 35; y <= CS_BOTTOM; ++y) {
            float l = exit_left[0] + exit_left[1]*y;
            float r = exit_right[0] + exit_right[1]*y;
            int a, b;
            int connected = r-l >= 40 &&
                cs_run(raw, thresholds, y, (int)((l+r)*.5f), &a, &b);
            /* A partially visible side opening can justify bridging before
               it is wide enough to count as exit evidence. */
            if (connected && a <= l-10 && b >= r+10) {
                if (++bridge_chain >= CS_OPEN_ROWS) bridge_ready = 1;
            } else bridge_chain = 0;
            if (connected && l >= 16 && r <= 171 && a <= l-16 && b >= r+16) {
                ++white_chain;
                if (white_chain > s->exit_white_rows)
                    s->exit_white_rows = (uint8_t)white_chain;
                if (y >= 83 && white_chain >= 8) near_white = 1;
            } else white_chain = 0;
        }
    }
    s->exit_white_seen = previous.exit_white_seen;
    /* A continuous side turn has a far bank bending in the same direction
       across successive images. Width expansion alone also occurs at bends. */
    int side = 0;
    if (was_active) {
        if (left[55] > left[64] && right[55] > 110 &&
            left[55] > 8 && right[55] - left[55] >= 44) side = 1;
        else if (right[55] && right[55] < right[64] && left[55] < 77 &&
                 right[55] < 179 && right[55] - left[55] >= 44) side = -1;
        if (side) {
            s->outer_count = (uint8_t)cs_outer_bank(s, raw, left, right, side);
            if (!s->outer_count) side = 0;
        }
    }
    s->bend_side = (int8_t)side;
    if (side) {
        s->bend_frames = 1;
        s->bend_tick = tick;
        if (side == previous.bend_side) {
            s->bend_frames = previous.bend_frames;
            if ((uint32_t)(tick-previous.bend_tick) > CS_HOLD_TICKS)
                s->bend_frames = 1;
            else if (tick != previous.bend_tick && s->bend_frames < 3)
                ++s->bend_frames;
        }
    }
    if (was_active && (s->bend_frames >= 3 || previous.bend_follow)) {
        int near_pairs = 0;
        for (int y = 91; y <= CS_BOTTOM; ++y) near_pairs += paired[y];
        if (!previous.bend_follow) s->exit_white_seen = 0;
        if (near_white) s->exit_white_seen = 1;
        int clear_exit = s->exit_white_seen && !near_white && near_pairs >= 13;
        s->exit_frames = clear_exit ? previous.exit_frames : 0;
        if (clear_exit && (!previous.bend_follow || tick != previous.bend_tick) &&
            s->exit_frames < 3) ++s->exit_frames;
        s->bend_tick = tick;
        s->bend_follow = side || (was_active && s->exit_frames < 3);
        s->bend_side = side ? (int8_t)side : previous.bend_side;
        for (int y = CS_TOP; y <= CS_BOTTOM; ++y) {
            if (left[y] > 3 && left[y] < 94) s->bend_left[y] = (uint8_t)left[y];
            if (right[y] > 94 && right[y] < 184) s->bend_right[y] = (uint8_t)right[y];
        }
        /* Once the opening has passed the near field, its disappearance
           with a currently measured bend is enough to hand off this frame. */
        int exit_into_bend = s->exit_white_seen && !bridge_ready && side;
        s->active = was_active && !exit_into_bend && s->exit_frames < 3;
        s->recovered = was_active && !s->active;
        s->bend_released = s->recovered;
        s->support_tick = tick;
        if (s->recovered) {
            s->bend_follow = 0;
            s->bend_side = 0;
            s->bend_frames = 0;
            s->outer_count = 0;
            s->exit_white_seen = 0;
            return;
        }
        if (!bridge_ready) return;
        s->outer_count = 0;
        /* A visible straight stem still guides the car through the white
           opening when the earlier curved boundary has left the image. */
    }
    int last = exit_last;
    int support = exit_support;
    if (support) {
        memcpy(s->left, exit_left, sizeof s->left);
        memcpy(s->right, exit_right, sizeof s->right);
    }
    if (!support) {
        if (!was_active) return;
        memcpy(s->left, previous.left, sizeof s->left);
        memcpy(s->right, previous.right, sizeof s->right);
        s->support_tick = previous.support_tick;
        s->hold_frames = previous.hold_frames < 255 ? (uint8_t)(previous.hold_frames + 1) : 255;
        s->source = 3;
        if (s->hold_frames > 3 || (uint32_t)(tick-s->support_tick)>CS_HOLD_TICKS) return;
    } else {
        s->support = (uint8_t)support;
        s->source = last >= 90 ? 1 : 2;
        s->support_tick = tick;
    }
    int chain = 0, longest = 0;
    for (int y = CS_TOP; y <= CS_BOTTOM; ++y) {
        float l = s->left[0] + s->left[1] * y;
        float r = s->right[0] + s->right[1] * y;
        int x = (int)((l + r) * 0.5f + 0.5f), a, b;
        if (!cs_run(raw, thresholds, y, x, &a, &b)) { chain = 0; continue; }
        int opening = a < l - 12 && b > r + 12 && r - l >= 40;
        if (opening) {
            if (!chain) s->gap_top = (uint8_t)y;
            ++chain;
            if (chain > longest) { longest = chain; s->gap_bottom = (uint8_t)y; }
        } else chain = 0;
        if (a > 3 && (fabsf(a-l) <= 4 || (s->source == 2 && paired[y] && a < x-6 && fabsf(a-l) <= 28))) {
            s->measured[y] |= 1; s->measured_left[y]=(uint8_t)a;
        }
        if (b < 184 && (fabsf(b-r) <= 4 || (s->source == 2 && paired[y] && b > x+6 && fabsf(b-r) <= 28))) {
            s->measured[y] |= 2; s->measured_right[y]=(uint8_t)b;
        }
    }
    s->opening = (uint8_t)(longest >= CS_OPEN_ROWS);
    if (!was_active && !s->opening) return;
    /* The line is inferred across missing banks, with white road checked at
     * every center target and a three-pixel corridor around it. */
    for (int y = CS_BOTTOM; y >= CS_TOP; --y) {
        float x = (s->left[0] + s->right[0] + (s->left[1] + s->right[1]) * y) * 0.5f;
        int center = (int)(x + 0.5f);
        int white = center >= 6 && center <= 181;
        for (int offset = -3; offset <= 3; ++offset)
            if (white && raw[y * 188 + center + offset] <= thresholds[y]) white = 0;
        if (!white) {
            if (y >= 65) return;
            break;
        }
        s->center[y] = (uint8_t)center;
    }
    s->active = 1;
    s->valid = 1;
    int near_recovered = 0;
    for (int y = 80; y <= CS_BOTTOM; ++y) near_recovered += s->measured[y] != 0;
    s->clear_frames = !s->bend_follow && !s->opening && near_recovered >= 20 ? (uint8_t)(clear + 1) : 0;
    if (s->clear_frames >= CS_RECOVER_FRAMES) {
        s->active = 0; s->recovered = 1;
    }
}

#endif
