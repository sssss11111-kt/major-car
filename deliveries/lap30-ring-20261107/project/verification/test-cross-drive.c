#include "cross_drive.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t raw[CAR_H * CAR_W], threshold[CAR_H];

static void scene(int island_left, int island_right, int top, int bottom)
{
    memset(raw, 20, sizeof raw);
    memset(threshold, 100, sizeof threshold);
    for (int y = 0; y < CAR_H; ++y) {
        for (int x = 22; x <= 165; ++x) {
            if (y < top || y > bottom || x < island_left || x > island_right)
                raw[y * CAR_W + x] = 220;
        }
    }
}

static void test_side(int side)
{
    cross_drive_t s = {0};
    special_track_output_t scan = {.type = SPECIAL_CROSS_ISLAND};
    int offset = -side * 8;
    scene(83 + offset, 104 + offset, 56, 87);
    cross_drive_output_t out = cross_drive_step(&s, &scan, raw, threshold, 20, 0, 0, 0);
    assert(out.active && !out.valid && s.phase == CROSS_DRIVE_IDLE);
    out = cross_drive_step(&s, &scan, raw, threshold, 22, 1, 0, 0);
    assert(out.active && out.valid && s.side == side && out.preview_error * side > 8);
    scan.type = SPECIAL_UNKNOWN;
    scene(83 - offset, 104 - offset, 56, 87);
    out = cross_drive_step(&s, &scan, raw, threshold, 24, 1, 0, 0);
    assert(out.valid && s.side == side && out.preview_error * side > 8);

    scene(0, 0, 120, 119);
    for (uint32_t tick = 26; tick < 32; tick += 2) {
        out = cross_drive_step(&s, &scan, raw, threshold, tick, 1, 0, 1);
        assert(out.valid && s.phase == CROSS_DRIVE_FOLLOW);
    }

    scene(83 - side * 28, 104 - side * 28, 86, 105);
    out = cross_drive_step(&s, &scan, raw, threshold, 32, 1, 0, 0);
    assert(out.valid && s.phase == CROSS_DRIVE_EXIT);
    scene(0, 0, 120, 119);
    for (uint32_t tick = 34; tick < 38; tick += 2) {
        out = cross_drive_step(&s, &scan, raw, threshold, tick, 1, 0, 1);
        assert(out.active && out.valid && s.phase == CROSS_DRIVE_EXIT);
    }
    out = cross_drive_step(&s, &scan, raw, threshold, 38, 1, 0, 1);
    assert(!out.active && s.phase == CROSS_DRIVE_IDLE);
}

static void test_recovered_frame(void)
{
    const uint32_t gaps[] = {4, 13}; /* 40 ms and 130 ms between real images. */
    special_track_output_t scan = {.type = SPECIAL_CROSS_ISLAND};
    for (unsigned i = 0; i < sizeof gaps / sizeof gaps[0]; ++i) {
        cross_drive_t s = {0};
        scene(83, 104, 56, 87);
        memset(raw + 70 * CAR_W, 20, CAR_W);
        cross_drive_output_t out = cross_drive_step(&s, &scan, raw, threshold, 10, 1, 0, 0);
        assert(out.valid && out.approach_only && s.phase == CROSS_DRIVE_APPROACH);
        scene(83, 104, 56, 87);
        out = cross_drive_step(&s, &scan, raw, threshold, 10 + gaps[i], 1, 0, 0);
        assert(out.valid && !out.approach_only && s.phase == CROSS_DRIVE_FOLLOW);
    }
}

static void test_retry_limits(void)
{
    special_track_output_t scan = {.type = SPECIAL_CROSS_ISLAND};
    cross_drive_t s = {0};
    scene(83, 104, 56, 87);
    memset(raw + 70 * CAR_W, 20, CAR_W);
    assert(cross_drive_step(&s, &scan, raw, threshold, 10, 1, 0, 0).approach_only);
    assert(cross_drive_step(&s, &scan, raw, threshold, 12, 1, 0, 0).approach_only);
    assert(!cross_drive_step(&s, &scan, raw, threshold, 14, 1, 0, 0).valid);
    assert(s.phase == CROSS_DRIVE_LOST);
    scene(83, 104, 56, 87);
    assert(!cross_drive_step(&s, &scan, raw, threshold, 16, 1, 0, 0).valid);

    /* Two frames do not permit a third retry even inside the time budget. */
    s = (cross_drive_t){0};
    memset(raw + 70 * CAR_W, 20, CAR_W);
    assert(cross_drive_step(&s, &scan, raw, threshold, 10, 1, 0, 0).approach_only);
    assert(cross_drive_step(&s, &scan, raw, threshold, 11, 1, 0, 0).approach_only);
    assert(!cross_drive_step(&s, &scan, raw, threshold, 12, 1, 0, 0).valid);

    /* A blocked near road never qualifies for measured approach. */
    s = (cross_drive_t){0};
    scene(83, 104, 56, 87);
    memset(raw + 96 * CAR_W, 20, CAR_W);
    assert(!cross_drive_step(&s, &scan, raw, threshold, 10, 1, 0, 0).valid);
    assert(s.phase == CROSS_DRIVE_LOST);

    /* A complete path cannot bypass the overall lifetime or diagnostics. */
    s = (cross_drive_t){.phase = CROSS_DRIVE_APPROACH, .started_tick = 10};
    scene(83, 104, 56, 87);
    assert(!cross_drive_step(&s, &scan, raw, threshold, 810, 1, 0, 0).valid);
    const uint8_t faults[] = {SPECIAL_DIAG_LOW_CONTRAST, SPECIAL_DIAG_OVERFLOW,
                              SPECIAL_DIAG_CONFLICT, SPECIAL_DIAG_CONFIG};
    for (unsigned i = 0; i < sizeof faults / sizeof faults[0]; ++i) {
        s = (cross_drive_t){.phase = CROSS_DRIVE_APPROACH, .started_tick = 10};
        scan.diagnostic_flags = faults[i];
        assert(!cross_drive_step(&s, &scan, raw, threshold, 14, 1, 0, 0).valid);
        assert(s.phase == CROSS_DRIVE_LOST);
    }
}

static void test_preview_horizon(void)
{
    const int rows[] = {50, 75, 65};
    special_track_output_t scan = {.type = SPECIAL_CROSS_ISLAND};
    for (int extra_split = 0; extra_split <= 1; ++extra_split) {
        for (unsigned i = 0; i < sizeof rows / sizeof rows[0]; ++i) {
            cross_drive_t s = {0};
            if (extra_split) scene(55, 76, 86, 105); /* Near merge is visible. */
            else scene(83, 104, 56, 87);
            cross_drive_output_t baseline = cross_drive_step(&s, &scan, raw, threshold, 10, 1, 0, 0);
            assert(baseline.valid);
            cross_drive_phase_t phase = s.phase;
            uint8_t *row = raw + rows[i] * CAR_W;
            if (extra_split) {
                memset(row + 83, 20, 22);
            } else {
                memset(row, 20, CAR_W);
                memset(row + 22, 220, 20);
                memset(row + 65, 220, 30);
                memset(row + 125, 220, 41);
            }
            cross_drive_output_t out = cross_drive_step(&s, &scan, raw, threshold, 12, 1, 0, 0);
            if (rows[i] < 65) {
                assert(out.valid && s.phase == phase);
                assert(out.near_error == baseline.near_error && out.preview_error == baseline.preview_error);
            } else {
                assert(!out.valid && s.phase == CROSS_DRIVE_LOST);
            }
        }
    }
}

int main(int argc, char **argv)
{
    /* Named cases allow each regression to be observed against the old owner. */
    if (argc > 1) {
        if (!strcmp(argv[1], "recovery")) test_recovered_frame();
        else if (!strcmp(argv[1], "horizon")) test_preview_horizon();
        else return 2;
        return 0;
    }
    test_recovered_frame();
    test_retry_limits();
    test_preview_horizon();
    test_side(-1);
    test_side(1);
    cross_drive_t s = {0};
    special_track_output_t scan = {.type = SPECIAL_CROSS_ISLAND};
    scene(83, 104, 56, 87);
    assert(!cross_drive_step(&s, &scan, raw, threshold, 0, 1, 1, 0).active);
    assert(cross_drive_step(&s, &scan, raw, threshold, 2, 1, 0, 0).valid);
    memset(raw + 91 * CAR_W, 20, CAR_W);
    cross_drive_output_t out = cross_drive_step(&s, &scan, raw, threshold, 4, 1, 0, 0);
    assert(out.active && !out.valid && s.phase == CROSS_DRIVE_LOST);
    scene(83, 104, 56, 87);
    assert(!cross_drive_step(&s, &scan, raw, threshold, 6, 1, 0, 0).valid);
    puts("PASS: cross-island recovery, bounded retries, preview horizon, left/right latch, measured merge, exit and loss lock");
    return 0;
}
