#ifndef SINGLE_EDGE_H
#define SINGLE_EDGE_H
#include <stdint.h>
#define SE_W 188
#define SE_H 120
#define SE_ROW 90
#define SE_TIMEOUT_TICKS 50U
/* mode: 0 invalid, 1 measured dual, 2 measured left, 3 measured right. */
typedef struct {
    uint8_t mode, center, degraded, model_matches;
    uint32_t since;
} se_result_t;
se_result_t single_edge_update(const uint8_t *binary, const uint8_t *left,
    const uint8_t *mid, const uint8_t *right, uint32_t tick, uint8_t allow_single);
#endif
