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

int main(void)
{
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
    puts("PASS: cross-island left/right latch, measured merge, three-frame exit and loss lock");
    return 0;
}
