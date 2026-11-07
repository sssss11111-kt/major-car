/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "obstacle_detect.h"
#include <string.h>

/* Experimental replay parameters. See data/obstacle/README.md for evidence.
 * Row 105 is the existing LAP28 track limit before the vehicle body. */
enum {
    ROI_TOP = 36, ROI_BOTTOM = 100, TRACK_BOTTOM = 105,
    ANCHOR_OFFSET = 7, MARGIN = 11, CONTRAST = 30, EDGE_STEP = 4,
    EDGE_CONTRAST = 23, ROAD_TOLERANCE = 18, QUEUE_LIMIT = 2048,
    MAX_ROW_SPAN = 12, MAX_BOUNDARY_SHIFT = 24,
    MASK_STRIDE = (OBS_WIDTH + 7) / 8, CONFIRM_HITS = 3
};

static uint8_t mask[ROI_BOTTOM - ROI_TOP + 1][MASK_STRIDE];
static uint16_t queue[QUEUE_LIMIT];
static uint8_t reference[OBS_HEIGHT];
static uint16_t histogram[256];

static int min_int(int a, int b) { return a < b ? a : b; }
static int max_int(int a, int b) { return a > b ? a : b; }

static uint8_t *mask_byte(int x, int y)
{
    return &mask[y - ROI_TOP][x / 8];
}

static uint8_t mask_bit(int x)
{
    return (uint8_t)(1u << (x % 8));
}

static int pop_pixel(int x, int y)
{
    uint8_t *byte = mask_byte(x, y);
    uint8_t bit = mask_bit(x);
    if (!(*byte & bit)) return 0;
    *byte &= (uint8_t)~bit;
    return 1;
}

static int lane_row_valid(obstacle_lane_row_t row)
{
    return row.measured && row.left > 2 && row.right < OBS_WIDTH - 3 &&
           row.right - row.left >= 35 && row.right - row.left <= 170;
}

static obstacle_lane_row_t lane_at(const obstacle_lane_row_t *lane, int y)
{
    if (lane_row_valid(lane[y])) return lane[y];
    int before = y - 1, after = y + 1;
    while (before >= 0 && y - before < MAX_ROW_SPAN && !lane_row_valid(lane[before])) --before;
    while (after <= TRACK_BOTTOM && after - y < MAX_ROW_SPAN && !lane_row_valid(lane[after])) ++after;
    if (before < 0 || after > TRACK_BOTTOM || after - before > MAX_ROW_SPAN ||
        !lane_row_valid(lane[before]) || !lane_row_valid(lane[after]))
        return (obstacle_lane_row_t){0};
    int left_delta = lane[after].left - lane[before].left;
    int right_delta = lane[after].right - lane[before].right;
    if (left_delta < -MAX_BOUNDARY_SHIFT || left_delta > MAX_BOUNDARY_SHIFT ||
        right_delta < -MAX_BOUNDARY_SHIFT || right_delta > MAX_BOUNDARY_SHIFT)
        return (obstacle_lane_row_t){0};
    return (obstacle_lane_row_t){
        (uint8_t)(lane[before].left + left_delta * (y - before) / (after - before)),
        (uint8_t)(lane[before].right + right_delta * (y - before) / (after - before)), 2};
}

static uint8_t build_mask(const uint8_t *gray, const obstacle_lane_row_t *lane)
{
    uint8_t interpolated = 0;
    memset(mask, 0, sizeof mask);
    for (int y = ROI_TOP; y <= ROI_BOTTOM; ++y) {
        obstacle_lane_row_t anchor = lane_at(lane, min_int(y + ANCHOR_OFFSET, TRACK_BOTTOM));
        if (!anchor.measured) continue;
        if (anchor.measured == 2) ++interpolated;
        int left = anchor.left, right = anchor.right;
        /* The lower, same-frame measured corridor is a proxy, with an
         * inward margin for perspective and the projection uncertainty. */
        left += MARGIN;
        right -= MARGIN;
        const uint8_t *row = gray + y * OBS_WIDTH;
        memset(histogram, 0, sizeof histogram);
        for (int x = left; x <= right; ++x) ++histogram[row[x]];
        int ref = 255, upper_count = (right - left + 1) - 3 * (right - left + 1) / 4;
        while ((upper_count -= histogram[ref]) > 0) --ref;
        reference[y] = (uint8_t)ref;
        for (int x = left; x <= right; ++x) {
            if (ref - row[x] >= CONTRAST) *mask_byte(x, y) |= mask_bit(x);
        }
    }
    return interpolated;
}

static int component(int x, int y, obstacle_candidate_t *candidate)
{
    unsigned count = 1;
    int left = x, right = x, top = y, bottom = y;
    queue[0] = (uint16_t)(y * OBS_WIDTH + x);
    for (unsigned head = 0; head < count; ++head) {
        x = queue[head] % OBS_WIDTH;
        y = queue[head] / OBS_WIDTH;
        left = min_int(left, x);
        right = max_int(right, x);
        top = min_int(top, y);
        bottom = max_int(bottom, y);
        for (int yy = max_int(y - 1, ROI_TOP); yy <= min_int(y + 1, ROI_BOTTOM); ++yy) {
            for (int xx = max_int(x - 1, 0); xx <= min_int(x + 1, OBS_WIDTH - 1); ++xx) {
                if (!pop_pixel(xx, yy)) continue;
                if (count == QUEUE_LIMIT) return 0;
                queue[count++] = (uint16_t)(yy * OBS_WIDTH + xx);
            }
        }
    }
    *candidate = (obstacle_candidate_t){.x_min = (uint8_t)left, .x_max = (uint8_t)right,
        .y_min = (uint8_t)top, .y_max = (uint8_t)bottom, .area = (uint16_t)count};
    return 1;
}

static int score(obstacle_candidate_t *box, const uint8_t *gray,
                 const uint8_t *binary, const uint8_t *thresholds)
{
    int width = box->x_max - box->x_min + 1;
    int height = box->y_max - box->y_min + 1;
    /* Perspective bands bracket the observed front face widths (19–41 px).
     * Minimum area rises below row 64. All bounds remain experimental. */
    if (width < 10 || width > 56 || height < 3 || height > 28 || width < height ||
        box->area < (box->y_max <= 64 ? 28 : 48)) return 0;
    box->fill_permille = (uint16_t)((uint32_t)box->area * 1000 / (width * height));
    if (box->fill_permille < 440) return 0;

    uint8_t edge[OBS_WIDTH] = {0};
    unsigned sum = 0, contrast = 0, dark = 0, support = 0;
    for (unsigned i = 0; i < box->area; ++i) {
        unsigned index = queue[i];
        int y = index / OBS_WIDTH;
        int value = gray[index];
        sum += value;
        contrast += reference[y] - value;
        if (binary ? binary[index] == 0 : (thresholds && value <= thresholds[y])) ++dark;
        if (y >= box->y_max - 4) {
            int below = gray[index + EDGE_STEP * OBS_WIDTH];
            if (below - value >= EDGE_CONTRAST && below >= reference[y] - ROAD_TOLERANCE)
                edge[index % OBS_WIDTH] = 1;
        }
    }
    for (int x = box->x_min; x <= box->x_max; ++x) support += edge[x];
    if ((int)(2 * support) < width) return 0;
    box->edge_columns = (uint8_t)support;
    box->mean_gray = (uint8_t)(sum / box->area);
    box->contrast = (uint8_t)(contrast / box->area);
    box->binary_dark_permille = (uint16_t)((uint32_t)dark * 1000 / box->area);
    return 1;
}

static int matches(obstacle_candidate_t a, obstacle_candidate_t b)
{
    int overlap_w = min_int(a.x_max, b.x_max) - max_int(a.x_min, b.x_min) + 1;
    int overlap_h = min_int(a.y_max, b.y_max) - max_int(a.y_min, b.y_min) + 1;
    if (overlap_w <= 0 || overlap_h <= 0) return 0;
    int common = overlap_w * overlap_h;
    int area_a = (a.x_max - a.x_min + 1) * (a.y_max - a.y_min + 1);
    int area_b = (b.x_max - b.x_min + 1) * (b.y_max - b.y_min + 1);
    return 4 * common >= area_a + area_b - common; /* Experimental IoU >= 0.25. */
}

obstacle_result_t obstacle_detect_frame(const uint8_t gray[OBS_PIXELS],
    const uint8_t *binary, const uint8_t *thresholds,
    const obstacle_lane_row_t lane[OBS_HEIGHT], uint32_t frame, obstacle_track_t *track)
{
    obstacle_result_t result = {0};
    if (!gray || !lane || !track) {
        if (track) track->hits = 0;
        return result;
    }
    result.interpolated_rows = build_mask(gray, lane);
    for (int y = ROI_TOP; y <= ROI_BOTTOM; ++y) {
        for (int x = 0; x < OBS_WIDTH; ++x) {
            if (!pop_pixel(x, y)) continue;
            obstacle_candidate_t box;
            if (!component(x, y, &box)) {
                track->hits = 0;
                return (obstacle_result_t){.overflow = 1};
            }
            if (!score(&box, gray, binary, thresholds)) continue;
            ++result.candidate_count;
            if (!result.found || box.edge_columns > result.best.edge_columns ||
                (box.edge_columns == result.best.edge_columns && box.area > result.best.area)) {
                result.found = 1;
                result.best = box;
            }
        }
    }
    if (!result.found) {
        track->hits = 0;
        return result;
    }
    if (track->hits && frame - track->last_frame == 1u && matches(track->last, result.best)) {
        if (track->hits < UINT8_MAX) ++track->hits;
    } else {
        track->hits = 1;
    }
    track->last_frame = frame;
    track->last = result.best;
    result.hits = track->hits;
    result.confirmed = track->hits >= CONFIRM_HITS;
    return result;
}
