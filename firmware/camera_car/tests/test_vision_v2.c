/* Behaviour tests for ordinary curves and failure rejection.
 * Synthetic geometry is ground truth, not evidence of real-car performance.
 * GPL-3.0-or-later.
 */
#include "car_core.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t gray[CAR_W * CAR_H], binary[CAR_W * CAR_H];
static int truth_left[CAR_H], truth_right[CAR_H];
static vision_config_t cfg;
static vision_state_t state;
static vision_result_t result;
static unsigned checks, failures, cases;

static int absolute(int x) { return x < 0 ? -x : x; }
static void check(int ok, const char *name, const char *contract) {
    checks++;
    if (!ok) {
        failures++;
        fprintf(stderr, "FAIL %s: %s\n", name, contract);
    }
}
static void start(void) {
    vision_config_default(&cfg);
    cfg.first_row = 40;
    cfg.last_row = 112;
    cfg.row_step = 6;
    vision_reset(&state);
}
static void process(void) {
    vision_process(&state, &cfg, gray, binary, &result);
}
static void report(const char *name, unsigned before) {
    cases++;
    printf("v2 %-34s %s valid=%u paired=%u/%u rows=%u error=%.2f\n",
           name, before == failures ? "PASS" : "FAIL", result.valid,
           result.paired_rows, result.scanned_rows, result.valid_rows,
           (double)result.error);
}
/* Mirror exactly around x=93.5, including pixel rounding.
 * Offset 40 gives only a few current-frame paired anchors, while every true
 * centre remains inside the image. Widths do not change through the sequence.
 */
static void road(int direction, int offset, int perspective, int shadow) {
    for (int y = 0; y < CAR_H; y++) {
        int bend = offset ? (112 - y) * 55 / 100 : 0;
        int center = 94 + offset + bend;
        int half = perspective ? 20 + y / 4 : 44;
        if (direction < 0) center = CAR_W - 1 - center;
        truth_left[y] = center - half;
        truth_right[y] = center + half;
        for (int x = 0; x < CAR_W; x++) {
            int inside = x >= truth_left[y] && x <= truth_right[y];
            gray[y * CAR_W + x] = (uint8_t)(
                shadow && y <= 100 ? (inside ? 45 : 15) :
                (inside ? 210 : 30));
        }
    }
}
static void geometry(const char *name) {
    unsigned counted_pairs = 0, counted_rows = 0;
    for (int y = cfg.last_row; y >= cfg.first_row; y -= cfg.row_step) {
        unsigned flags = result.edge_flags[y];
        if (flags) counted_rows++;
        if (flags == 3) counted_pairs++;
        check(flags <= 3, name, "only observed-edge bits");
        if (flags & 1)
            check(truth_left[y] >= 2 &&
                  absolute(result.left[y] - truth_left[y]) <= 1,
                  name, "left flag describes a real visible edge");
        if (flags & 2)
            check(truth_right[y] <= CAR_W - 3 &&
                  absolute(result.right[y] - truth_right[y]) <= 1,
                  name, "right flag describes a real visible edge");
        if (flags) {
            float truth = (truth_left[y] + truth_right[y]) * 0.5f;
            check(fabsf(result.center[y] - truth) <= 2.0f,
                  name, "observed or inferred centre within two pixels");
        }
    }
    check(counted_pairs == result.paired_rows, name, "paired count is honest");
    check(counted_rows == result.valid_rows, name, "valid row count is honest");
}
static void clipped_curve(int direction, int perspective) {
    const char *name = perspective ?
        (direction > 0 ? "right_clipped_perspective" : "left_clipped_perspective") :
        (direction > 0 ? "right_clipped_fixed_width" : "left_clipped_fixed_width");
    unsigned before = failures;
    start();
    road(direction, 0, perspective, 0);
    for (int i = 0; i < 3; i++) process();
    check(result.valid, name, "straight calibration is usable");
    for (int offset = 10; offset <= 40; offset += 10) {
        road(direction, offset, perspective, 0);
        process();
    }
    check(result.paired_rows >= 2 &&
          result.paired_rows * 2 < result.scanned_rows,
          name, "fixture actually has minority paired observations");
    check(result.valid, name, "continuous clipped curve is usable");
    check(result.valid_rows * 3 >= result.scanned_rows * 2,
          name, "most rows remain supported by real edges");
    check(direction * result.error > 35, name, "steering has correct curve direction");
    geometry(name);
    /* Current near-field geometry must support a sustained bend, not just an
     * eight-frame grace period after the straight. */
    for (int frame = 0; frame < 30; frame++) {
        process();
        check(result.valid, name, "current anchors support a sustained 30-frame bend");
        check(direction * result.error > 35, name, "sustained bend keeps steering direction");
        geometry(name);
    }
    report(name, before);
}
static void row_illumination(void) {
    const char *name = "clear_road_under_strong_shadow";
    unsigned before = failures;
    start(); road(1, 0, 0, 1); process();
    check(result.valid && fabsf(result.error) < 1.0f,
          name, "locally separated road remains usable");
    check(result.paired_rows >= 10, name, "shadow rows retain both real edges");
    geometry(name); report(name, before);
}
static void uniform_and_weak(void) {
    const char *name = "uniform_and_local_low_contrast";
    unsigned before = failures;
    start();
    for (int value = 0; value <= 255; value += 255) {
        memset(gray, value, sizeof gray); process();
        check(!result.valid && !result.valid_rows, name, "uniform image is invalid");
    }
    /* Global range is large; each row has only 8 grey levels of separation. */
    for (int y = 0; y < CAR_H; y++)
        for (int x = 0; x < CAR_W; x++)
            gray[y * CAR_W + x] = (uint8_t)(20 + y +
                (x >= 50 && x <= 138 ? 8 : 0));
    process();
    check(!result.valid, name, "global range cannot authorize locally weak edges");
    report(name, before);
}
static void smooth_gradients(void) {
    const char *name = "smooth_gradients_are_not_roads";
    unsigned before = failures;
    start();
    for (int y = 0; y < CAR_H; y++)
        for (int x = 0; x < CAR_W; x++) gray[y * CAR_W + x] = (uint8_t)(30 + x);
    process(); check(!result.valid, name, "monotone lighting ramp is invalid");
    /* Otsu would invent two boundaries in this smooth illumination dome. */
    for (int y = 0; y < CAR_H; y++)
        for (int x = 0; x < CAR_W; x++)
            gray[y * CAR_W + x] = (uint8_t)(30 + 2 * (94 - absolute(x - 94)));
    process(); check(!result.valid, name, "smooth illumination dome has no sharp road edges");
    report(name, before);
}
static void anchors_without_path(void) {
    const char *name = "two_anchors_without_path";
    unsigned before = failures;
    start(); road(1, 0, 0, 0);
    for (int i = 0; i < 3; i++) process();
    for (int y = 0; y < CAR_H; y++)
        if (y != 112 && y != 106) memset(gray + y * CAR_W, 210, CAR_W);
    process();
    check(!result.valid, name, "two good rows do not authorize blank remainder");
    check(result.valid_rows <= 2, name, "blank rows do not acquire ghost edges");
    report(name, before);
}
static void all_single_edge(void) {
    const char *name = "whole_frame_single_edge";
    unsigned before = failures;
    for (int warm = 0; warm <= 1; warm++) {
        for (int direction = -1; direction <= 1; direction += 2) {
            start();
            if (warm) {
                road(direction, 0, 0, 0);
                for (int i = 0; i < 3; i++) process();
            }
            for (int y = 0; y < CAR_H; y++)
                for (int x = 0; x < CAR_W; x++)
                    gray[y * CAR_W + x] = (uint8_t)(
                        (direction > 0 ? x >= 50 : x <= 137) ? 210 : 30);
            process();
            check(!result.valid && result.paired_rows == 0, name,
                  "cold and warm full single-edge views remain unanchored");
        }
    }
    report(name, before);
}
static void expired_geometry(void) {
    const char *name = "expired_width_cannot_bridge_curve";
    unsigned before = failures;
    start(); road(1, 0, 0, 0);
    for (int i = 0; i < 3; i++) process();
    memset(gray, 210, sizeof gray);
    for (int i = 0; i < 10; i++) {
        process();
        check(!result.valid && result.valid_rows == 0, name, "lost frames have no ghost path");
    }
    /* Offset 43 leaves only two paired rows, so there is not enough current
     * width evidence to replace the expired historical model. */
    road(1, 43, 0, 0); process();
    check(!result.valid, name, "expired per-row widths cannot authorize majority inference");
    report(name, before);
}
static void parallel_band(void) {
    const char *name = "outside_band_does_not_steal_path";
    unsigned before = failures;
    start(); road(1, 0, 0, 0); process();
    for (int frame = 0; frame < 4; frame++) {
        road(1, 0, 0, 0);
        for (int y = 0; y < CAR_H; y++)
            for (int x = 4; x <= 28; x++) gray[y * CAR_W + x] = 230;
        process();
        check(result.valid && fabsf(result.error) < 1.0f, name,
              "near-centre connected road wins over outside parallel band");
        geometry(name);
    }
    report(name, before);
}
static void reacquire_width(void) {
    const char *name = "reacquire_after_width_change";
    unsigned before = failures;
    start(); road(1, 0, 0, 0); process();
    for(int y=0;y<CAR_H;y++) for(int x=0;x<CAR_W;x++)
        gray[y*CAR_W+x]=(uint8_t)(x>=28 && x<=160?210:30);
    process(); /* One inconsistent frame may be rejected. */
    process();
    check(result.valid && fabsf(result.error)<1, name,
          "invalid history cannot block clear double-edge reacquisition");
    report(name,before);
}
int main(void) {
    clipped_curve(1, 0); clipped_curve(-1, 0);
    clipped_curve(1, 1); clipped_curve(-1, 1);
    row_illumination(); uniform_and_weak(); smooth_gradients();
    anchors_without_path(); all_single_edge(); expired_geometry(); parallel_band();
    reacquire_width();
    printf("Vision v2: %u cases, %u checks, %u failures\n", cases, checks, failures);
    return failures ? 1 : 0;
}
