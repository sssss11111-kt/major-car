#include "zebra_run.h"
#include <stdlib.h>

static int stripe_pattern(const uint8_t *src, int image_width, int thr, int *pattern_center)
{
    int x = 0, count = 0, first = 0, last = 0, previous_end = -1;
    int min_dark = image_width, max_dark = 0, min_gap = image_width, max_gap = 0;
    while (x < image_width) {
        int start, end, width, gap;
        while (x < image_width && src[x] > thr) ++x;
        start = x;
        while (x < image_width && src[x] <= thr) ++x;
        end = x - 1;
        width = end - start + 1;
        if (start <= 2 || end >= image_width-3 || width < 2 || width > 18) {
            if (count >= 4) break;
            count = 0; previous_end = -1;
            min_dark = min_gap = image_width; max_dark = max_gap = 0;
            continue;
        }
        gap = previous_end >= 0 ? start-previous_end-1 : 0;
        if (previous_end >= 0 && (gap < 2 || gap > 26)) {
            if (count >= 4) break;
            count = 0; min_dark = min_gap = image_width; max_dark = max_gap = 0;
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


uint16_t zebra_detect_bottom(const uint8_t *gray, int width, int height,
                             int stride, const uint8_t *row_threshold,
                             int top, int bottom)
{
    int chain = 0, previous = -1;
    uint16_t result = 0;
    if (!gray || !row_threshold || width < 40 || stride < width ||
        top < 1 || bottom < top || bottom >= height || bottom > UINT16_MAX)
        return 0;

    for (int y = top; y <= bottom; ++y) {
        int center;
        if (stripe_pattern(gray + (size_t)y * (size_t)stride, width,
                           row_threshold[y], &center)) {
            chain = previous >= 0 && abs(center - previous) <= 6 ? chain + 1 : 1;
            previous = center;
            if (chain >= 4) result = (uint16_t)y;
        } else {
            chain = 0;
            previous = -1;
        }
    }
    return result;
}

void zebra_run_start(zebra_run_t *run, uint32_t frame)
{
    *run = (zebra_run_t){.last_frame = frame, .phase = ZEBRA_LEAVE};
}

void zebra_run_step(volatile zebra_run_t *run, uint32_t tick,
                                  uint32_t frame, uint16_t bottom, int stop_row,
                                  int left_counts, int right_counts, float meters_per_count)
{
    const int new_frame = frame != run->last_frame;
    run->last_frame = frame;

    switch (run->phase) {
    case ZEBRA_LEAVE:
        run->count_sum += left_counts + right_counts;
        if (new_frame) {
            if (bottom) run->clear_frames = 0;
            else if (run->clear_frames < ZEBRA_CLEAR_FRAMES) ++run->clear_frames;
        }
        if (run->clear_frames >= ZEBRA_CLEAR_FRAMES &&
            run->count_sum * (0.5f * meters_per_count) >= ZEBRA_REARM_METERS)
            run->phase = ZEBRA_LAP;
        break;
    case ZEBRA_LAP:
        if (!new_frame) break;
        if (!bottom) run->confirm_frames = 0;
        else if (++run->confirm_frames >= ZEBRA_CONFIRM_FRAMES) {
            run->phase = ZEBRA_FORWARD;
            run->phase_tick = tick;
            run->count_sum = 0;
        }
        break;
    case ZEBRA_FORWARD:
        run->count_sum += left_counts + right_counts;
        if (run->count_sum * (0.5f * meters_per_count) >= ZEBRA_FORWARD_METERS) {
            run->phase = ZEBRA_SETTLE;
            run->phase_tick = tick;
            run->still_ticks = 0;
        } else if (tick - run->phase_tick >= 1500U) {
            run->phase = ZEBRA_ABORT;
        }
        break;
    case ZEBRA_SETTLE:
        if (left_counts == 0 && right_counts == 0) ++run->still_ticks;
        else run->still_ticks = 0;
        if (run->still_ticks >= ZEBRA_SETTLED_TICKS) {
            run->phase = ZEBRA_REVERSE;
            run->phase_tick = tick;
            run->count_sum = 0;
        } else if (tick - run->phase_tick >= 300U) {
            run->phase = ZEBRA_ABORT;
        }
        break;
    case ZEBRA_REVERSE:
        run->count_sum += left_counts + right_counts;
        if (new_frame && bottom >= stop_row) {
            run->phase = ZEBRA_DONE;
        } else if (tick - run->phase_tick >= 2500U ||
                   run->count_sum * (0.5f * meters_per_count) >= ZEBRA_REVERSE_LIMIT_METERS) {
            run->phase = ZEBRA_ABORT;
        }
        break;
    default:
        break;
    }
}
