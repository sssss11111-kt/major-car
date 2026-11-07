#ifndef OBSTACLE_VISION_H
#define OBSTACLE_VISION_H

#include "car_core.h"
#include "obstacle_detect.h"

/* Adapter from the existing vision result. Never use single-edge width
 * predictions as observations. The detector may bridge bounded row gaps. */
static inline void obstacle_lane_from_vision(const vision_result_t *vision,
                                               obstacle_lane_row_t lane[OBS_HEIGHT])
{
    for (int y = 0; y < OBS_HEIGHT; ++y) {
        int left = vision->left[y], right = vision->right[y];
        lane[y] = (obstacle_lane_row_t){0};
        if (vision->edge_flags[y] == 3 && left >= 0 && right < OBS_WIDTH && left < right)
            lane[y] = (obstacle_lane_row_t){(uint8_t)left, (uint8_t)right, 1};
    }
}

#endif
