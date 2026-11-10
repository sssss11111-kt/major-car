#include "special_track.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    STRAIGHT, LEFT_BEND, RIGHT_BEND, S_BEND, EDGE_LOSS, SHADOW, REFLECTION,
    NOISE, CROSS, RING_LEFT, RING_RIGHT, OVERFLOW, WHITE, BLACK, LOW_CONTRAST, CLIPPED_BANK
} fixture_t;

static uint8_t gray[CAR_W * CAR_H], binary[CAR_W * CAR_H];
static vision_state_t vision;
static vision_result_t normal;
static special_track_state_t track;
static special_track_config_t config;
static special_track_output_t output;

static void render(fixture_t scene, int dark_road, int advance)
{
    for (int y = 0; y < CAR_H; ++y) {
        int d = y - 106;
        int shift = 0;
        if (scene == LEFT_BEND) shift = -d * d / 100;
        if (scene == RIGHT_BEND) shift = d * d / 100;
        if (scene == S_BEND) shift = (y - 74) * (d * d / 80) / 26;
        int lo = 28 + shift, hi = 159 + shift;
        if (scene == EDGE_LOSS && y >= 71 && y <= 82) lo = 0;
        if (scene == CLIPPED_BANK) hi = y < 49 || y > 91 ? CAR_W - 1 : 146;
        for (int x = 0; x < CAR_W; ++x) {
            int road = x >= lo && x <= hi;
            if (scene == CROSS && y >= 56 + advance && y <= 87 + advance)
                road = road && (x < 83 || x > 104);
            if ((scene == RING_LEFT || scene == RING_RIGHT) &&
                y >= 55 + advance && y <= 91 + advance) {
                int z = y - 73 - advance;
                int arc = z * z / 20;
                int col = scene == RING_LEFT ? x : CAR_W - 1 - x;
                road = col >= 25 + arc && col <= 135 + arc &&
                       (col < 46 + arc || col > 63 + arc);
            }
            if (scene == SHADOW && y >= 60 && y <= 64) road = 0;
            if (scene == REFLECTION && y >= 47 && y <= 91 && x >= 170 && x <= 181)
                road = 1;
            if (scene == NOISE && (x * 83 + y * 107) % 97 == 0) road = !road;
            if (scene == OVERFLOW) road = x % 25 < 12;
            if (scene == WHITE) road = 1;
            if (scene == BLACK) road = 0;
            gray[y * CAR_W + x] = (uint8_t)(scene == LOW_CONTRAST ? 92 + road :
                                           (road != dark_road ? 210 : 30));
        }
    }
}

static void frame(fixture_t scene, uint32_t now, int dark_road, int advance)
{
    vision_config_t vision_config;
    vision_config_default(&vision_config);
    vision_config.white_road = (uint8_t)!dark_road;
    render(scene, dark_road, advance);
    vision_process(&vision, &vision_config, gray, binary, &normal);
    special_track_update(&track, &config, binary, &normal, now, &output);
    assert(!output.route_ready && !output.target_valid);
}

static void reset(void)
{
    vision_reset(&vision);
    special_track_init(&track);
    special_track_config_default(&config);
}

static void detect(fixture_t scene, special_track_type_t type, int dark_road, uint32_t start)
{
    frame(STRAIGHT, start, dark_road, 0);
    assert(output.state == SPECIAL_IDLE && !output.full_scan_attempted);
    for (unsigned i = 1; i <= 3; ++i) {
        frame(scene, start + i * 20, dark_road, (int)i - 1);
        assert(output.full_scan_valid && output.frame_type == type);
        if (i < 3) assert(output.state == SPECIAL_CANDIDATE && output.type == SPECIAL_UNKNOWN);
    }
    assert(output.type == type);
    assert(output.state == (type == SPECIAL_CROSS_ISLAND ? SPECIAL_ACTIVE_CROSS :
                                                           SPECIAL_ACTIVE_ROUNDABOUT));
}

static void diagnostics(const char *dir, const char *name, const char *verdict)
{
    if (!dir) return;
    const char *suffix[] = {"gray.pgm", "binary.pgm", "overlay.ppm"};
    for (unsigned kind = 0; kind < 3; ++kind) {
        char path[1024];
        snprintf(path, sizeof path, "%s/%s-%s", dir, name, suffix[kind]);
        FILE *file = fopen(path, "wb");
        assert(file);
        if (kind != 2) {
            fprintf(file, "P5\n%d %d\n255\n", CAR_W, CAR_H);
            assert(fwrite(kind == 0 ? gray : binary, 1, sizeof gray, file) == sizeof gray);
        } else {
            fprintf(file, "P6\n%d %d\n255\n", CAR_W, CAR_H);
            for (int y = 0; y < CAR_H; ++y) for (int x = 0; x < CAR_W; ++x) {
                uint8_t base = gray[y * CAR_W + x] / 2;
                uint8_t rgb[] = {base, base, base};
                for (unsigned i = 0; i < track.scan.row_count; ++i) {
                    const special_scan_row_t *row = &track.scan.rows[i];
                    if (y != row->y) continue;
                    for (unsigned j = 0; j < row->run_count; ++j) {
                        const special_run_t *run = &row->runs[j];
                        if (x >= run->left && x <= run->right) rgb[1] = 155;
                        if (x == run->left || x == run->right) rgb[1] = 255;
                    }
                }
                if (x == normal.left[y] || x == normal.right[y]) rgb[2] = 255;
                if (x == normal.center[y]) rgb[0] = 255;
                assert(fwrite(rgb, 1, sizeof rgb, file) == sizeof rgb);
            }
        }
        assert(fclose(file) == 0);
    }
    char path[1024];
    snprintf(path, sizeof path, "%s/results.tsv", dir);
    FILE *file = fopen(path, "a");
    assert(file);
    fprintf(file, "%s\t%s\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\n", name, verdict,
            output.type, output.state, output.candidate_score, output.confidence,
            track.scan.row_count, track.scan.multi_run_rows, track.scan.central_dark_rows,
            track.scan.curved_boundary_rows, output.diagnostic_flags);
    assert(fclose(file) == 0);
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : NULL;
    const char *names[] = {"straight", "left_bend", "right_bend", "s_bend", "edge_loss",
        "shadow", "reflection", "noise"};
    for (unsigned scene = STRAIGHT; scene <= NOISE; ++scene) {
        reset();
        for (unsigned i = 0; i < 7; ++i) frame((fixture_t)scene, i * 20, 0, 0);
        assert(output.type != SPECIAL_CROSS_ISLAND && output.type != SPECIAL_ROUNDABOUT);
        if (scene == STRAIGHT) assert(!output.full_scan_attempted);
        if (scene == LEFT_BEND || scene == RIGHT_BEND) assert(output.full_scan_attempted);
        diagnostics(dir, names[scene], scene == STRAIGHT ? "PASS" : "REJECT");
    }

    reset();
    detect(CROSS, SPECIAL_CROSS_ISLAND, 0, 0);
    diagnostics(dir, "cross", "PASS");
    frame(OVERFLOW, 80, 0, 0);
    assert(output.type == SPECIAL_UNKNOWN && output.state == SPECIAL_ACTIVE_CROSS);
    assert(output.full_scan_attempted && !output.full_scan_valid && track.scan.overflow);
    for (unsigned i = 0; i < track.scan.row_count; ++i)
        assert(track.scan.rows[i].run_count == SPECIAL_MAX_RUNS_PER_ROW);
    diagnostics(dir, "overflow", "UNKNOWN");
    frame(CROSS, 100, 0, 0);
    assert(output.type == SPECIAL_CROSS_ISLAND);
    for (unsigned i = 0; i < 4; ++i) {
        frame(RING_LEFT, 120 + i * 20, 0, 0);
        assert(output.state == SPECIAL_ACTIVE_CROSS && output.type == SPECIAL_UNKNOWN);
    }
    frame(BLACK, 120 + config.loss_timeout_ms, 0, 0);
    assert(output.state == SPECIAL_FAULT && output.type == SPECIAL_UNKNOWN);
    for (unsigned i = 0; i < config.recover_frames; ++i)
        frame(STRAIGHT, 500 + i * 20, 0, 0);
    assert(output.state == SPECIAL_COOLDOWN);

    reset();
    detect(RING_LEFT, SPECIAL_ROUNDABOUT, 0, 0);
    diagnostics(dir, "ring_left", "PASS");
    assert(output.island_side < 0);
    reset();
    detect(RING_RIGHT, SPECIAL_ROUNDABOUT, 1, UINT32_MAX - 120u);
    diagnostics(dir, "ring_right_dark_road", "PASS");
    assert(output.island_side > 0);
    for (unsigned i = 0; i < config.recover_frames; ++i)
        frame(STRAIGHT, UINT32_MAX - 40u + i * 20, 1, 0);
    assert(output.state == SPECIAL_COOLDOWN);
    uint32_t cooled_at = UINT32_MAX - 40u + (config.recover_frames - 1u) * 20u;
    frame(RING_RIGHT, cooled_at + config.cooldown_ms - 1u, 1, 0);
    assert(output.state == SPECIAL_COOLDOWN && !output.full_scan_attempted);
    frame(RING_RIGHT, cooled_at + config.cooldown_ms, 1, 0);
    assert(output.state == SPECIAL_CANDIDATE);

    reset();
    frame(STRAIGHT, 0, 0, 0);
    frame(CROSS, 20, 0, 0);
    frame(OVERFLOW, 40, 0, 0);
    frame(CROSS, 60, 0, 0);
    assert(output.type == SPECIAL_UNKNOWN);
    frame(CROSS, 80, 0, 0);
    assert(output.type == SPECIAL_CROSS_ISLAND);

    reset();
    frame(STRAIGHT, 0, 0, 0);
    for (unsigned i = 1; i <= 6; ++i) {
        frame(i % 2 ? RING_LEFT : RING_RIGHT, i * 20, 0, 0);
        if (i <= 4) assert(output.type == SPECIAL_UNKNOWN);
    }
    /* A fresh UNKNOWN cannot promote previously accumulated votes. */
    frame(OVERFLOW, 140, 0, 0);
    assert(output.type == SPECIAL_UNKNOWN);

    reset();
    config.vote_window = 3;
    config.vote_required = 2;
    frame(CROSS, 0, 0, 0);
    frame(CROSS, config.loss_timeout_ms, 0, 0);
    assert(output.type == SPECIAL_UNKNOWN);
    frame(CROSS, config.loss_timeout_ms + 20, 0, 0);
    assert(output.type == SPECIAL_CROSS_ISLAND);

    reset();
    frame(STRAIGHT, 0, 0, 0);
    for (unsigned i = 1; i <= 7; ++i) {
        frame(CLIPPED_BANK, i * 20, 0, 0);
        assert(output.full_scan_attempted && output.type == SPECIAL_UNKNOWN);
        assert(track.scan.clipped_bank_rows >= 2);
    }
    diagnostics(dir, "clipped_bank", "UNKNOWN");

    reset();
    for (unsigned i = 0; i < 7; ++i) frame(RING_LEFT, i * 20, 0, 0);
    assert(output.type == SPECIAL_UNKNOWN);
    diagnostics(dir, "ring_without_approach", "OBSERVE");

    for (unsigned scene = WHITE; scene <= LOW_CONTRAST; ++scene) {
        reset();
        frame((fixture_t)scene, 0, 0, 0);
        assert(output.type != SPECIAL_CROSS_ISLAND && output.type != SPECIAL_ROUNDABOUT);
    }
    config.vote_window = 255;
    frame(CROSS, 20, 0, 0);
    assert(output.state == SPECIAL_FAULT && (output.diagnostic_flags & SPECIAL_DIAG_CONFIG));

    puts("special track: PASS (multi-row classification, polarity, voting, overflow, loss and wraparound)");
    return 0;
}
