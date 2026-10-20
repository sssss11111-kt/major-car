/* Host-only adapter for the actual firmware vision core. GPL-3.0-or-later. */
#include "car_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t gray[CAR_W * CAR_H], binary[CAR_W * CAR_H];
static vision_state_t state;
static vision_result_t result;

int main(int argc, char **argv) {
    vision_config_t cfg;
    unsigned frame = 0;
    char path[2048];
    FILE *input;
    if (argc < 3 || argc > 5) {
        fprintf(stderr, "usage: vision_replay RAW_SEQUENCE OUTPUT_DIR [THRESHOLD|-1] [DARK_ROAD]\n");
        return 2;
    }
    vision_config_default(&cfg);
    if (argc > 3 && strcmp(argv[3], "-1") != 0) {
        char *end;
        long value = strtol(argv[3], &end, 10);
        if (!*argv[3] || *end || value < 0 || value > 255) return 2;
        cfg.otsu = 0;
        cfg.fixed_threshold = (uint8_t)value;
    }
    if (argc > 4) {
        if (strcmp(argv[4], "0") && strcmp(argv[4], "1")) return 2;
        cfg.white_road = (uint8_t)(strcmp(argv[4], "1") != 0);
    }
    input = fopen(argv[1], "rb");
    if (!input) { perror("raw sequence"); return 2; }
    vision_reset(&state);
    for (;;) {
        size_t got = fread(gray, 1, sizeof gray, input);
        if (!got) break;
        if (got != sizeof gray) {
            fprintf(stderr, "truncated raw frame %u\n", frame);
            fclose(input); return 2;
        }
        vision_process(&state, &cfg, gray, binary, &result);
        int length = snprintf(path, sizeof path, "%s/%06u-binary.pgm", argv[2], frame);
        if (length < 0 || (size_t)length >= sizeof path) { fclose(input); return 2; }
        FILE *out = fopen(path, "wb");
        if (!out) { perror("binary output"); fclose(input); return 2; }
        fprintf(out, "P5\n%d %d\n255\n", CAR_W, CAR_H);
        if (fwrite(binary, 1, sizeof binary, out) != sizeof binary || fclose(out)) {
            fprintf(stderr, "binary write failed\n"); fclose(input); return 2;
        }
        printf("{\"frame\":%u,\"valid\":%u,\"paired_rows\":%u,\"valid_rows\":%u,"
               "\"scanned_rows\":%u,\"inferred_rows\":%u,\"threshold\":%u,"
               "\"contrast\":%u,\"error\":%.6f,\"rows\":[",
               frame, result.valid, result.paired_rows, result.valid_rows,
               result.scanned_rows, result.inferred_rows, result.threshold,
               result.contrast, (double)result.error);
        for (int y = 0; y < CAR_H; y++) {
            if (y) putchar(',');
            printf("{\"y\":%d,\"left\":%d,\"right\":%d,\"center\":%d,"
                   "\"edge_flags\":%u,\"threshold\":%u}",
                   y, result.left[y], result.right[y], result.center[y],
                   result.edge_flags[y], result.row_threshold[y]);
        }
        puts("]}");
        frame++;
    }
    if (ferror(input)) { fclose(input); return 2; }
    fclose(input);
    if (!frame) { fprintf(stderr, "empty raw sequence\n"); return 2; }
    return 0;
}
