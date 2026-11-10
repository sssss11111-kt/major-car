/* Multi-interval scan of the normalized camera image. GPL-3.0-or-later. */
#include "special_track_scan.h"
#include <string.h>

typedef struct {
    const special_run_t *left, *right;
    int side;
} row_gap_t;

static int magnitude(int v) { return v < 0 ? -v : v; }
static unsigned capped(unsigned value, unsigned limit) { return value < limit ? value : limit; }

static void scan_row(const uint8_t *row, unsigned min_width, special_scan_row_t *out)
{
    int x = 0;
    while (x < CAR_W) {
        while (x < CAR_W && row[x] != 255) ++x;
        int left = x;
        while (x < CAR_W && row[x] == 255) ++x;
        int width = x - left;
        if (width < (int)min_width) continue;
        if (out->run_count == SPECIAL_MAX_RUNS_PER_ROW) {
            out->overflow = 1;
            return;
        }
        special_run_t *run = &out->runs[out->run_count++];
        run->left = (int16_t)left;
        run->right = (int16_t)(x - 1);
        run->center = (int16_t)((left + x - 1) / 2);
        run->width = (uint16_t)width;
    }
}

static row_gap_t find_gap(const special_scan_row_t *row, const special_track_config_t *cfg,
                          uint8_t *ambiguous)
{
    row_gap_t gap = {0};
    for (unsigned i = 1; i < row->run_count; ++i) {
        const special_run_t *left = &row->runs[i - 1], *right = &row->runs[i];
        if (left->width < cfg->min_channel_width || right->width < cfg->min_channel_width ||
            right->left - left->right - 1 < cfg->min_gap_width) continue;
        if (gap.left) {
            *ambiguous = 1;
            return (row_gap_t){0};
        }
        int offset = left->right + right->left - (CAR_W - 1);
        gap.left = left;
        gap.right = right;
        if (magnitude(offset) > 2 * cfg->center_tolerance) gap.side = offset < 0 ? -1 : 1;
    }
    return gap;
}

static int gap_linked(const row_gap_t *a, const row_gap_t *b, unsigned shift)
{
    return a->side == b->side &&
           a->left->right + 1 < b->right->left && b->left->right + 1 < a->right->left &&
           a->left->left <= b->left->right && b->left->left <= a->left->right &&
           a->right->left <= b->right->right && b->right->left <= a->right->right &&
           magnitude(a->left->right - b->left->right) <= (int)shift &&
           magnitude(a->right->left - b->right->left) <= (int)shift;
}

static int joins(const special_scan_row_t *row, const row_gap_t *gap)
{
    return row->run_count == 1 && row->runs[0].left <= gap->left->center &&
           row->runs[0].right >= gap->right->center;
}

static int bend(int a, int b, int c, int dy_ab, int dy_bc, unsigned step)
{
    int change = (c - b) * dy_ab - (b - a) * dy_bc;
    int threshold = (int)step * (dy_ab < dy_bc ? dy_ab : dy_bc);
    return magnitude(change) >= threshold ? (change > 0 ? 1 : -1) : 0;
}

static int paired_bend(const special_run_t *a, const special_run_t *b, const special_run_t *c,
                       const special_scan_result_t *scan, unsigned i,
                       const special_track_config_t *cfg)
{
    if (a->left == 0 || b->left == 0 || c->left == 0 ||
        a->right == CAR_W - 1 || b->right == CAR_W - 1 || c->right == CAR_W - 1)
        return 0;
    int dy_ab = scan->rows[i - 1].y - scan->rows[i - 2].y;
    int dy_bc = scan->rows[i].y - scan->rows[i - 1].y;
    /* Reject disconnected runs before comparing how their two boundaries bend. */
    if (a->right < b->left || b->right < a->left ||
        b->right < c->left || c->right < b->left) return 0;
    int left = bend(a->left, b->left, c->left, dy_ab, dy_bc, cfg->min_curve_step);
    int right = bend(a->right, b->right, c->right, dy_ab, dy_bc, cfg->min_curve_step);
    return left == right ? left : 0;
}

static void count_ordinary_bends(special_scan_result_t *scan, const special_track_config_t *cfg)
{
    int previous = 0;
    unsigned streak = 0;
    for (unsigned i = 2; i < scan->row_count; ++i) {
        const special_scan_row_t *a = &scan->rows[i - 2], *b = &scan->rows[i - 1];
        const special_scan_row_t *c = &scan->rows[i];
        int sign = 0;
        if (a->run_count == 1 && b->run_count == 1 && c->run_count == 1)
            sign = paired_bend(&a->runs[0], &b->runs[0], &c->runs[0], scan, i, cfg);
        streak = sign == 0 ? 0 : (sign == previous ? streak + 1 : 1);
        previous = sign;
        if (streak > scan->curved_boundary_rows) scan->curved_boundary_rows = (uint8_t)streak;
    }
}

static unsigned arc_rows(const row_gap_t *gaps, unsigned start, unsigned end,
                         const special_scan_result_t *scan, const special_track_config_t *cfg)
{
    unsigned best = 0, streak = 0;
    int previous = 0;
    for (unsigned i = start + 2; i < end; ++i) {
        /* For a left island use the right channel and vice versa. */
        int side = gaps[i].side;
        const special_run_t *a = side < 0 ? gaps[i - 2].right : gaps[i - 2].left;
        const special_run_t *b = side < 0 ? gaps[i - 1].right : gaps[i - 1].left;
        const special_run_t *c = side < 0 ? gaps[i].right : gaps[i].left;
        int sign = paired_bend(a, b, c, scan, i, cfg);
        if (magnitude(a->center - b->center) > cfg->max_row_shift ||
            magnitude(b->center - c->center) > cfg->max_row_shift) sign = 0;
        streak = sign == 0 ? 0 : (sign == previous ? streak + 1 : 1);
        previous = sign;
        if (streak > best) best = streak;
    }
    return best;
}

static void summarize_gaps(const row_gap_t *gaps, special_scan_result_t *scan,
                           const special_track_config_t *cfg)
{
    for (unsigned start = 0; start < scan->row_count;) {
        if (!gaps[start].left) { ++start; continue; }
        unsigned end = start + 1;
        while (end < scan->row_count && gaps[end].left &&
               gap_linked(&gaps[end - 1], &gaps[end], cfg->max_row_shift)) ++end;
        unsigned span = end - start;
        int side = gaps[start].side;
        int near_join = end < scan->row_count && joins(&scan->rows[end], &gaps[end - 1]);
        if (span > scan->linked_gap_rows) scan->linked_gap_rows = (uint8_t)span;
        if (side == 0) {
            if (span > scan->central_dark_rows) scan->central_dark_rows = (uint8_t)span;
            if (near_join && start > 0 && joins(&scan->rows[start - 1], &gaps[start]) &&
                span > scan->cross_rows) scan->cross_rows = (uint8_t)span;
        } else {
            unsigned arc = arc_rows(gaps, start, end, scan, cfg);
            if (arc > scan->curved_boundary_rows) scan->curved_boundary_rows = (uint8_t)arc;
            if (near_join && span >= cfg->min_ring_rows && arc >= cfg->min_arc_rows) {
                if (scan->island_side && scan->island_side != side) scan->ring_conflict = 1;
                if (span > scan->ring_rows) {
                    scan->ring_rows = (uint8_t)span;
                    scan->island_side = (int8_t)side;
                }
            }
        }
        start = end;
    }
}

/* A bank entering or leaving the image edge can trigger a scan. Its unseen
 * side contributes no topology, so the classifier cannot use it as a split. */
static void count_clipped_bank(special_scan_result_t *scan, const special_track_config_t *cfg,
                               int right, int reverse)
{
    unsigned streak = 0;
    int saw_clip = 0, previous_outer = -1;
    for (unsigned j = 0; j < scan->row_count; ++j) {
        unsigned i = reverse ? scan->row_count - 1 - j : j;
        const special_scan_row_t *row = &scan->rows[i];
        if (row->run_count != 1 || row->runs[0].width < cfg->min_channel_width) {
            saw_clip = 0;
            streak = 0;
            previous_outer = -1;
            continue;
        }
        const special_run_t *run = &row->runs[0];
        int outer = right != 0 ? run->left : CAR_W - 1 - run->right;
        int inset = right != 0 ? CAR_W - 1 - run->right : run->left;
        if (outer == 0 || (previous_outer >= 0 && magnitude(outer - previous_outer) > cfg->max_row_shift)) {
            saw_clip = 0;
            streak = 0;
        }
        previous_outer = outer;
        if (outer == 0) continue;
        if (inset == 0) {
            saw_clip = 1;
            streak = 0;
        } else if (saw_clip && inset >= cfg->min_gap_width) {
            if (++streak > scan->clipped_bank_rows) scan->clipped_bank_rows = (uint8_t)streak;
        } else {
            streak = 0;
        }
    }
}

void special_track_scan(const uint8_t *binary, const special_track_config_t *cfg,
                        const vision_result_t *normal, unsigned count, special_scan_result_t *out)
{
    row_gap_t gaps[SPECIAL_SCAN_ROWS] = {{0}};
    memset(out, 0, sizeof *out);
    out->row_count = (uint8_t)count;
    if (normal->scanned_rows > normal->paired_rows)
        out->normal_edge_loss_rows = (uint8_t)capped(normal->scanned_rows - normal->paired_rows, CAR_H);
    for (unsigned i = 0; i < count; ++i) {
        special_scan_row_t *row = &out->rows[i];
        row->y = (uint8_t)(cfg->scan_top + (cfg->scan_bottom - cfg->scan_top) * i / (count - 1));
        scan_row(binary + row->y * CAR_W, cfg->min_run_width, row);
        out->overflow |= row->overflow;
        if (row->run_count > 1) ++out->multi_run_rows;
        if (row->run_count > 0 && row->runs[0].width < CAR_W) ++out->usable_rows;
        gaps[i] = find_gap(row, cfg, &out->ambiguous);
        if (i == 0 || row->run_count != 1 || out->rows[i - 1].run_count != 1) continue;
        const special_run_t *previous = &out->rows[i - 1].runs[0];
        const special_run_t *current = &row->runs[0];
        if ((previous->left == 0) != (current->left == 0) ||
            (previous->right == CAR_W - 1) != (current->right == CAR_W - 1) ||
            magnitude(previous->width - current->width) > 2 * cfg->max_row_shift)
            ++out->edge_breaks;
    }
    count_ordinary_bends(out, cfg);
    summarize_gaps(gaps, out, cfg);
    count_clipped_bank(out, cfg, 1, 0);
    count_clipped_bank(out, cfg, 1, 1);
    count_clipped_bank(out, cfg, 0, 0);
    count_clipped_bank(out, cfg, 0, 1);
}

uint8_t special_monitor_score(const special_scan_result_t *scan, const vision_result_t *normal)
{
    if (scan->overflow) return 100;
    unsigned split = capped(14u * scan->multi_run_rows, 35);
    unsigned dark = scan->linked_gap_rows >= 2 ? capped(10u * scan->linked_gap_rows, 25) : 0;
    unsigned bend_score = capped(18u * scan->curved_boundary_rows, 36);
    unsigned edge = capped(12u * scan->edge_breaks + 8u * scan->clipped_bank_rows, 30);
    if (normal->scanned_rows && 4u * scan->normal_edge_loss_rows >= normal->scanned_rows)
        edge += 16;
    return (uint8_t)capped(split + dark + bend_score + edge, 100);
}
