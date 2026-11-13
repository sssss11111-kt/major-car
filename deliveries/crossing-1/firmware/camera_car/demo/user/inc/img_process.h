#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_
#include "zf_common_headfile.h"

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


#endif
