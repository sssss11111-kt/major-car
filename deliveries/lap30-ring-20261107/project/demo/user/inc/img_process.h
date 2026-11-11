#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_
#include "zf_common_headfile.h"

/* Full track width at row 90; tolerance is also in full-width pixels. */
#define IP_WIDTH_CAL_ROW       90
#define IP_WIDTH_CAL_PX        142
#define IP_WIDTH_TOLERANCE_PX  15
/* User-confirmed vehicle body enters the image at row 107 (zero based).
 * Keep one extra row of clearance; track processing ends at row 105. */
#define IP_BODY_MASK_START_ROW 107
#define IP_HALF_PERSPECTIVE_SPAN 32.0f
#include "ring_entry_path.h"
#include "entry_bridge.h"

/* Auxiliary roundabout evidence flags (bitmask exported by img_get_roundabout_evidence). */
#define IP_RING_EV_FULL_ISLAND    0x01U /* Enclosed 8-connected hole between two corridors */
#define IP_RING_EV_SIDE_ISLAND    0x02U /* Side-clipped tapered dark island */
#define IP_RING_EV_CORNER_LOW     0x04U /* Lower bifurcation corner / step detected */
#define IP_RING_EV_CORNER_APEX    0x08U /* Inner island arc apex inflection detected */
#define IP_RING_EV_OUTER_CONT     0x10U /* Outer guide boundary continuous & smooth */
#define IP_RING_EV_WIDTH_ANOMALY  0x20U /* Throat expansion + branch constriction */

void get_hist_gram(unsigned char* _img,
                   const unsigned short _rows,
                   const unsigned short _cols,
                   short* _hist_gram);
unsigned char get_threshold_otsu(const short* _hist_gram);
void binaryzation_process(unsigned char* _img,
                          const unsigned short _rows,
                          const unsigned short _cols,
                          const unsigned int _threshold_value);
void auxiliary_process(uint8_t* _src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols,
                       unsigned char _threshold_val,
                       uint8_t* _left_line, uint8_t* _mid_line, uint8_t* _right_line);

/* 新增：清空帧间状态（上电、换场地、摄像头重新初始化后调用一次） */
void img_process_reset(void);
/* 新增：本帧巡线结果，main.c 用它替代"只看第90行"的有效性判断 */
uint8_t  img_get_valid(void);        /* 1 = 本帧可信 */
int16_t  img_get_error(void);        /* 像素：正 = 赛道中心在画面中心右侧 */
uint8_t  img_get_usable_rows(void);  /* 本帧可用行数（遥测用） */
uint8_t  img_get_both_rows(void);    /* 本帧双边行数（遥测用） */
uint8_t  img_get_single_rows(void);  /* 本帧单边重建行数（遥测用） */
uint8_t  img_get_near_run(void);     /* 近场连续实测行数：有效性判据用 */
int16_t  img_get_preview_error(void); /* Measured rows 55..75, before gap filling. */
uint8_t  img_get_preview_valid(void);
/* Flat stripe marker, including three clean-frame recovery. */
uint8_t img_get_stripe_active(void);
uint8_t img_get_stripe_rows(void);

/* Current-frame topology + auxiliary geometry; negative guide selects left of island (clockwise).
 * Candidate recognition and entry target path generation are separated. */
uint8_t img_get_roundabout_candidate(void);
uint8_t img_get_roundabout_evidence(void);
const uint8_t *img_get_row_thresholds(void);
uint8_t img_get_scene_contrast(void);
uint8_t img_get_roundabout_corner_row(void);
int16_t img_get_roundabout_entry_error(void);
uint8_t img_get_near_valid(void);
/* Current raw dual-edge normal corridor, independent of patched centerline. */
uint8_t img_get_road_recovered(void);
/* Read-only diagnostics: no relaxation of validity. Bottom is meaningful only
 * with current evidence/candidate, not an independent detector result. */
uint8_t img_diag_near_rows(void);
uint8_t img_diag_island_bottom(void);
void img_set_roundabout_guidance(int side);
void img_set_roundabout_entry(uint8_t enabled);
void img_set_exit_search(uint8_t enabled);
uint8_t img_get_entry_ready(void);
uint8_t img_get_entry_path_safe(void);
uint8_t img_get_exit_ready(void);
int16_t img_get_exit_error(void);
const entry_path_t *img_get_entry_path(void);
/* 0 approach, 1 current paired path, 2 width-assisted path, 3 invalid,
 * 4 inner-edge path, 5 bounded odometry bridge. */
uint8_t img_get_entry_status(void);
void img_set_entry_odometry(float distance_m,float yaw_deg,uint32_t tick,uint8_t valid);
const entry_bridge_guard_t *img_get_entry_bridge_guard(void);
uint8_t img_get_roundabout_preview_valid(void);
#endif
