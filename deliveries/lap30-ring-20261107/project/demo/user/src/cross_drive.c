#include "cross_drive.h"

#define CROSS_BOTTOM 105
#define CROSS_SCAN_TOP 40
#define CROSS_NEAR 85
#define CROSS_PREVIEW 65
#define CROSS_MARGIN 4
#define CROSS_MIN_WIDTH 16
#define CROSS_MAX_ROW_STEP 4
#define CROSS_MAX_START_ERROR 16
#define CROSS_EXIT_NEAR 85
#define CROSS_EXIT_FRAMES 3
#define CROSS_TIMEOUT_TICKS 800u
#define CROSS_APPROACH_TICKS 4u
#define CROSS_APPROACH_FRAMES 2
#define CROSS_APPROACH_NEAR 90

typedef struct { int left, right; } interval_t;
typedef struct {
    int split_seen, near_merge, preferred_side;
    int16_t near_error, preview_error;
} corridor_t;

static int min(int a, int b) { return a < b ? a : b; }
static int max(int a, int b) { return a > b ? a : b; }
static int abs_diff(int a, int b) { return max(a, b) - min(a, b); }
static int clamp(int x, interval_t r) { return min(max(x, r.left), r.right); }
static int center(interval_t r) { return (r.left + r.right) / 2; }
static int overlaps(interval_t a, interval_t b) { return a.left <= b.right && b.left <= a.right; }

static int runs_at(const uint8_t *raw, int y, uint8_t threshold, interval_t runs[2])
{
    const uint8_t *row = raw + y * CAR_W;
    int count = 0;
    for (int x = 0; x < CAR_W;) {
        if (row[x] <= threshold) { ++x; continue; }
        int left = x;
        while (x < CAR_W && row[x] > threshold) ++x;
        if (x - left < CROSS_MIN_WIDTH) continue;
        if (count == 2) return -1;
        runs[count++] = (interval_t){left + CROSS_MARGIN, x - 1 - CROSS_MARGIN};
    }
    return count;
}

static int choose_side(const interval_t runs[2], interval_t reach)
{
    int x = center(reach);
    int left_distance = abs_diff(x, clamp(x, runs[0]));
    int right_distance = abs_diff(x, clamp(x, runs[1]));
    if (left_distance != right_distance) return left_distance < right_distance ? -1 : 1;
    return runs[0].right - runs[0].left >= runs[1].right - runs[1].left ? -1 : 1;
}

static int trace_corridor(const cross_drive_t *s, int side, const uint8_t *raw,
                          const uint8_t *thr, corridor_t *result)
{
    interval_t reach[CROSS_BOTTOM - CROSS_SCAN_TOP + 1];
    interval_t prev_runs[2] = {{0, 0}, {0, 0}};
    interval_t previous = {CAR_W / 2 - CROSS_MAX_START_ERROR, CAR_W / 2 + CROSS_MAX_START_ERROR};
    int prev_count = 0, farthest = CROSS_BOTTOM + 1, goal = CAR_W / 2, merge_rows = 0;
    for (int y = CROSS_BOTTOM; y >= CROSS_SCAN_TOP; --y) {
        interval_t runs[2];
        int count = runs_at(raw, y, thr[y], runs);
        if (count < 0) return 0;
        if (count == 0) break;
        if (count == 2 && merge_rows >= CROSS_EXIT_FRAMES) return 0;
        if (count == 2) merge_rows = 0;
        if (count == 2 && !result->preferred_side)
            result->preferred_side = choose_side(runs, previous);
        interval_t run = runs[count == 2 && side > 0 ? 1 : 0];
        if (count == 2 && runs[1].left - runs[0].right < 2 * CROSS_MARGIN + 7) break;
        interval_t next = {max(run.left, previous.left - CROSS_MAX_ROW_STEP),
                           min(run.right, previous.right + CROSS_MAX_ROW_STEP)};
        if (next.left > next.right || !overlaps(previous, next)) break;
        reach[y - CROSS_SCAN_TOP] = next;
        farthest = y;
        if (y >= CROSS_PREVIEW) goal = center(run);
        if (count == 2) result->split_seen = 1;
        if (prev_count == 2 && count == 1 && y >= CROSS_EXIT_NEAR &&
            overlaps(run, prev_runs[0]) && overlaps(run, prev_runs[1])) merge_rows = 1;
        else if (count == 1 && merge_rows) ++merge_rows;
        for (int i = 0; i < count; ++i) prev_runs[i] = runs[i];
        prev_count = count;
        previous = next;
    }
    if (farthest > CROSS_PREVIEW || (!s->saw_split && !result->split_seen)) return 0;
    result->near_merge = merge_rows >= CROSS_EXIT_FRAMES;
    /* Backtrack from the preview target through the connected reachable intervals. */
    int x = clamp(goal, reach[CROSS_PREVIEW - CROSS_SCAN_TOP]);
    result->preview_error = (int16_t)(x - CAR_W / 2);
    for (int y = CROSS_PREVIEW + 1; y <= CROSS_NEAR; ++y)
        x = clamp(x, reach[y - CROSS_SCAN_TOP]);
    if (s->saw_split && !result->split_seen && s->phase == CROSS_DRIVE_FOLLOW) {
        /* An occluded fork is not an exit. Keep measuring the same corridor. */
        interval_t near = reach[CROSS_NEAR - CROSS_SCAN_TOP];
        int prior = CAR_W / 2 + s->last_near_error;
        if (abs_diff(prior, clamp(prior, near)) > CROSS_MAX_START_ERROR) return 0;
    }
    result->near_error = (int16_t)(x - CAR_W / 2);
    return 1;
}

static int plan_corridor(cross_drive_t *s, const uint8_t *raw, const uint8_t *thresholds,
                         corridor_t *selected)
{
    if (s->side) return trace_corridor(s, s->side, raw, thresholds, selected);

    corridor_t left = {0}, right = {0};
    int left_valid = trace_corridor(s, -1, raw, thresholds, &left);
    int right_valid = trace_corridor(s, 1, raw, thresholds, &right);
    if (!left_valid && !right_valid) return 0;
    if (left_valid && (!right_valid || left.preferred_side < 0)) {
        s->side = -1;
        *selected = left;
    } else {
        s->side = 1;
        *selected = right;
    }
    return 1;
}

static int measured_stem(const uint8_t *raw, const uint8_t *thresholds, int16_t *error)
{
    /* Short retry is allowed only on a single, bounded road visibly containing
     * the vehicle axis on every near row. Do not bridge any missing pixels. */
    int sum = 0;
    for (int y = CROSS_BOTTOM; y >= CROSS_APPROACH_NEAR; --y) {
        interval_t runs[2];
        if (runs_at(raw, y, thresholds[y], runs) != 1 ||
            runs[0].left == CROSS_MARGIN || runs[0].right == CAR_W - 1 - CROSS_MARGIN ||
            clamp(CAR_W / 2, runs[0]) != CAR_W / 2 ||
            abs_diff(center(runs[0]), CAR_W / 2) > CROSS_MAX_START_ERROR) return 0;
        sum += center(runs[0]) - CAR_W / 2;
    }
    *error = (int16_t)(sum / (CROSS_BOTTOM - CROSS_APPROACH_NEAR + 1));
    return 1;
}

cross_drive_output_t cross_drive_step(cross_drive_t *s, const special_track_output_t *scan,
                                     const uint8_t *raw, const uint8_t *thresholds, uint32_t tick,
                                     int running, int roundabout_busy, int road_recovered)
{
    cross_drive_output_t out = {0};
    if (s->phase == CROSS_DRIVE_IDLE) {
        if (roundabout_busy || scan->type != SPECIAL_CROSS_ISLAND) return out;
        out.active = 1;
        if (!running) return out;
        s->phase = CROSS_DRIVE_APPROACH;
        s->started_tick = tick;
    }
    out.active = 1;
    if (!running || s->phase == CROSS_DRIVE_LOST) return out;
    corridor_t selected = {0};
    if (roundabout_busy || tick - s->started_tick >= CROSS_TIMEOUT_TICKS ||
        (s->phase == CROSS_DRIVE_APPROACH && tick - s->started_tick >= CROSS_APPROACH_TICKS) ||
        (scan->diagnostic_flags & (SPECIAL_DIAG_LOW_CONTRAST | SPECIAL_DIAG_OVERFLOW |
                                   SPECIAL_DIAG_CONFLICT | SPECIAL_DIAG_CONFIG)) ||
        scan->frame_type == SPECIAL_ROUNDABOUT) {
        s->phase = CROSS_DRIVE_LOST;
        return out;
    }
    if ((s->phase == CROSS_DRIVE_APPROACH && scan->type != SPECIAL_CROSS_ISLAND) ||
        !plan_corridor(s, raw, thresholds, &selected)) {
        if (s->phase == CROSS_DRIVE_APPROACH && s->approach_frames < CROSS_APPROACH_FRAMES &&
            measured_stem(raw, thresholds, &out.near_error)) {
            ++s->approach_frames;
            out.valid = out.approach_only = 1;
            out.preview_error = out.near_error;
            return out;
        }
        s->phase = CROSS_DRIVE_LOST;
        return out;
    }
    if (s->phase == CROSS_DRIVE_APPROACH) s->phase = CROSS_DRIVE_FOLLOW;
    out.near_error = selected.near_error;
    out.preview_error = selected.preview_error;
    s->saw_split |= (uint8_t)selected.split_seen;
    s->last_near_error = out.near_error;
    if (selected.near_merge) s->phase = CROSS_DRIVE_EXIT;
    if (s->phase == CROSS_DRIVE_EXIT && !selected.split_seen && road_recovered &&
        abs_diff(out.near_error, out.preview_error) <= CROSS_MAX_START_ERROR) {
        if (++s->clear_frames >= CROSS_EXIT_FRAMES) {
            *s = (cross_drive_t){0};
            return (cross_drive_output_t){0};
        }
    } else {
        s->clear_frames = 0;
    }
    out.valid = 1;
    return out;
}
