#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_
#include "zf_common_headfile.h"

/* Full track width at row 90; tolerance is also in full-width pixels. */
#define IP_WIDTH_CAL_ROW       90
#define IP_WIDTH_CAL_PX        142
#define IP_WIDTH_TOLERANCE_PX  15

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

#endif
