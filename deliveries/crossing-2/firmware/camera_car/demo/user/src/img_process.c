/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */


#include "img_process.h"
#include "string.h"

/*
 *@func: 二值化函数
 */
void binaryzation_process(unsigned char* _img, 
                          const unsigned short _rows, 
                          const unsigned short _cols, 
                          const unsigned int _threshold_value)
{
	unsigned short _size = _rows * _cols;
	for (unsigned short _i = 0; _i < _size; ++_i)
	{
		if (*(_img + _i) > _threshold_value)
			*(_img + _i) = 255;
		else
			*(_img + _i) = 0;
	}
}

/*
 *@func: 获取直方图
 */
void get_hist_gram(unsigned char* _img,
				   const unsigned short _rows, 
				   const unsigned short _cols, 
				   short* _hist_gram)
{
	memset(_hist_gram, 0, 256);
    for (unsigned short _i = 0; _i < _rows; ++_i)
    {
        for (unsigned short _j = 0; _j < _cols; ++_j)
        {
            _hist_gram[_img[_i * _cols + _j]]++;
        }
    }
}

/*
 *@func: 大津法求阈值
 */
unsigned char get_threshold_otsu(const short* _hist_gram)
{
    int X, Y, amount = 0;
    int pixel_back = 0, pixel_fore = 0, pixel_integral_back = 0, pixel_integral_fore = 0, pixel_integral = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int pixel_min_value, pixel_max_value;
    int threshold = 0;

    for (pixel_min_value = 0; pixel_min_value < 256 && _hist_gram[pixel_min_value] == 0; pixel_min_value++)
        ;
    for (pixel_max_value = 255; pixel_max_value > pixel_min_value && _hist_gram[pixel_max_value] == 0; pixel_max_value--)
        ;
    if (pixel_max_value == pixel_min_value)
        return pixel_max_value; // 图像中只有一个颜色
    if (pixel_min_value + 1 == pixel_max_value)
        return pixel_min_value; // 图像中只有二个颜色

    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        amount += _hist_gram[Y]; //  像素总数

    pixel_integral = 0;
    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        pixel_integral += _hist_gram[Y] * Y;
    SigmaB = -1;
    for (Y = pixel_min_value; Y < pixel_max_value; Y++)
    {
        pixel_back = pixel_back + _hist_gram[Y];
        pixel_fore = amount - pixel_back;
        OmegaBack = (double)pixel_back / amount;
        OmegaFore = (double)pixel_fore / amount;
        pixel_integral_back += _hist_gram[Y] * Y;
        pixel_integral_fore = pixel_integral - pixel_integral_back;
        MicroBack = (double)pixel_integral_back / pixel_back;
        MicroFore = (double)pixel_integral_fore / pixel_fore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            threshold = Y;
        }
    }
    return threshold;
}

void auxiliary_process(uint8_t* _src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols, 
					   unsigned char _threshold_val, 
					   uint8_t* _left_line, uint8_t* _mid_line, uint8_t* _right_line)
{
    uint8_t mid_point = _src_cols >> 1;
    if (_src_pixel_mat[(_src_rows - 1) * _src_cols + mid_point] < _threshold_val)
    {
        // 重新寻找扫线中点
        int16_t _tmp_left_mid = 0;
        int16_t _tmp_right_mid = _src_cols;
        for (int16_t j = (_src_cols >> 1); j < _src_cols - 2; ++j)
        {
            if (_src_pixel_mat[(_src_rows - 1) * _src_cols + j] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j + 1] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j + 2] > _threshold_val)
            {
                _tmp_right_mid = j + 2;
            }
        }

        for (int16_t j = (_src_cols >> 1); j > 2; --j)
        {
            if (_src_pixel_mat[(_src_rows - 1) * _src_cols + j] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j - 1] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j - 2] > _threshold_val)
            {
                _tmp_left_mid = j - 2;
            }
        }
        if (_tmp_right_mid - (_src_cols >> 1) < (_src_cols >> 1) - _tmp_left_mid)
        {
            mid_point = _tmp_right_mid;
        }
        else
        {
            mid_point = _tmp_left_mid;
        }
    }
    for (int i = _src_rows - 1; i >= 0; --i)
    {
        uint8_t cur_point = mid_point;
       // 扫描左线
        while (cur_point - 2 > 0)
        {
            _left_line[i] = 0;
            if (_src_pixel_mat[i * _src_cols + cur_point] < _threshold_val && 
				_src_pixel_mat[i * _src_cols + cur_point - 1] < _threshold_val && 
				_src_pixel_mat[i * _src_cols + cur_point - 2] < _threshold_val)
            {
                _left_line[i] = cur_point;
                break;
            }
            --cur_point;
        }
        // 扫描右线
        cur_point = mid_point;
        while (cur_point + 2 < _src_cols)
        {
            _right_line[i] = _src_cols - 1;
            if (_src_pixel_mat[i * _src_cols + cur_point] < _threshold_val &&
				_src_pixel_mat[i * _src_cols + cur_point + 1] < _threshold_val &&
				_src_pixel_mat[i * _src_cols + cur_point + 2] < _threshold_val)
            {
                _right_line[i] = cur_point;
                break;
            }
            ++cur_point;
        }

        _mid_line[i] = (_right_line[i] + _left_line[i]) >> 1;
        if (_left_line[i] != 0 || 
			_right_line[i] != _src_cols - 1)
        {
            mid_point = _mid_line[i];
        }
        else
        {
            mid_point = _src_cols >> 1;
        }
    }
}

