#ifndef ENTRY_BRIDGE_H
#define ENTRY_BRIDGE_H
#include <stdint.h>
#include <math.h>
/* Conservative software bridge bounds; field calibration remains required. */
#define ENTRY_BRIDGE_MAX_TICKS 10U
#define ENTRY_BRIDGE_MAX_DISTANCE_M 0.040f
#define ENTRY_BRIDGE_MAX_YAW_ERROR_DEG 3.0f
#define ENTRY_SINGLE_EDGE_TARGET 50.0f
#define ENTRY_BRIDGE_TARGET 35.0f

typedef struct {
    uint32_t visual_tick;
    float distance_m, yaw_deg, curvature;
    uint8_t active, stable;
} entry_bridge_guard_t;

static inline int entry_bridge_allowed(const volatile entry_bridge_guard_t *guard,
                                       float distance_m,float yaw_deg,uint32_t tick) {
    float travel, error;
    if (!guard->stable || (uint32_t)(tick-guard->visual_tick)>=ENTRY_BRIDGE_MAX_TICKS) return 0;
    travel=distance_m-guard->distance_m;
    if (!(travel>=-0.001f && travel<=ENTRY_BRIDGE_MAX_DISTANCE_M)) return 0;
    error=yaw_deg-guard->yaw_deg-guard->curvature*travel*57.2957795f;
    return fabsf(error)<=ENTRY_BRIDGE_MAX_YAW_ERROR_DEG;
}
#endif
