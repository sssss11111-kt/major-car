#include "obstacle_vision.h"
#include <string.h>

static obstacle_track_t tracks[3];
static vision_state_t vision;

__declspec(dllexport) void replay_reset_obstacle(void)
{
    memset(tracks, 0, sizeof tracks);
    vision_reset(&vision);
}

__declspec(dllexport) void replay_obstacle(uint8_t *gray, const uint8_t *binary,
    const uint8_t *lines, uint32_t frame, int mode, uint16_t out[13])
{
    obstacle_lane_row_t rows[OBS_HEIGHT] = {0};
    int suppressed = 0;
    static uint8_t work_binary[OBS_PIXELS];
    if (mode == 2) {
        vision_result_t result;
        vision_config_t config;
        vision_config_default(&config);
        vision_process(&vision, &config, gray, work_binary, &result);
        obstacle_lane_from_vision(&result, rows);
        binary = work_binary;
        suppressed = !result.valid;
    } else {
        for (int y = 0; y < OBS_HEIGHT; ++y) {
            uint8_t l = lines[y], m = lines[y + OBS_HEIGHT], r = lines[y + 2 * OBS_HEIGHT];
            rows[y] = (obstacle_lane_row_t){l, r, l < m && m < r};
        }
    }
    obstacle_result_t r = {0};
    if (suppressed) tracks[mode].hits = 0;
    else r = obstacle_detect_frame(gray, binary, 0, rows, frame, &tracks[mode]);
    const uint16_t result[] = {r.found, r.best.x_min, r.best.y_min, r.best.x_max, r.best.y_max,
        r.confirmed, r.hits, r.best.area, r.best.binary_dark_permille, suppressed,
        r.overflow, r.candidate_count, r.interpolated_rows};
    memcpy(out, result, sizeof result);
}

__declspec(dllexport) int detector_unit_tests(void)
{
    static uint8_t gray[OBS_PIXELS], binary[OBS_PIXELS];
    obstacle_lane_row_t rows[OBS_HEIGHT];
    obstacle_track_t track = {0};
    memset(gray, 250, sizeof gray);
    memset(binary, 1, sizeof binary);
    for (int y = 0; y < OBS_HEIGHT; ++y) rows[y] = (obstacle_lane_row_t){30, 160, 1};
    for (int y = 50; y <= 57; ++y) {
        memset(gray + y * OBS_WIDTH + 75, 170, 20);
        memset(gray + y * OBS_WIDTH + 2, 100, 20); /* Exterior confounder. */
        memset(binary + y * OBS_WIDTH + 75, 0, 20);
    }
    for (uint32_t i = UINT32_MAX - 1; ; ++i) {
        obstacle_result_t r = obstacle_detect_frame(gray, binary, 0, rows, i, &track);
        if (!r.found || r.best.x_min != 75 || r.best.x_max != 94 ||
            r.best.y_min != 50 || r.best.y_max != 57 || r.best.area != 160 ||
            r.best.binary_dark_permille != 1000 || r.confirmed != (i == 0)) return 1;
        if (i == 0) break;
    }
    if (obstacle_detect_frame(gray, binary, 0, rows, 2, &track).hits != 1) return 2;
    for (int y = 0; y < OBS_HEIGHT; ++y) rows[y].measured = 0;
    if (obstacle_detect_frame(gray, binary, 0, rows, 3, &track).found || track.hits) return 3;
    for (int y = 0; y < OBS_HEIGHT; ++y) {
        rows[y].measured = 1;
        memset(gray + y * OBS_WIDTH + 45, 160, 50);
    }
    if (!obstacle_detect_frame(gray, binary, 0, rows, 4, &track).overflow || track.hits) return 4;
    return 0;
}
