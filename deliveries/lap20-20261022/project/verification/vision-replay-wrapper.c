#include "img_process.h"
#include "steering_control.h"
#include <stdint.h>
static uint8_t left[120],mid[120],right[120];
__declspec(dllexport) void replay_reset(void) { img_process_reset(); }
__declspec(dllexport) void replay_turn(float *out)
{
    steering_result_t r=steering_control(img_get_error(),img_get_preview_error(),
        img_get_preview_valid(),92.5f,-1.0f,61.0f,124.0f);
    out[0]=r.command;out[1]=r.gain;out[2]=r.curve;
    out[3]=img_get_preview_error();out[4]=img_get_preview_valid();
}
__declspec(dllexport) void replay_lines(uint8_t *lines)
{
    for (int y=0;y<120;++y) {
        lines[y]=left[y];lines[120+y]=mid[y];lines[240+y]=right[y];
    }
}
__declspec(dllexport) int replay_frame(uint8_t *raw, int *result)
{
    auxiliary_process(raw,120,188,127,left,mid,right);
    result[0]=img_get_valid();result[1]=img_get_error();
    result[2]=img_get_usable_rows();result[3]=img_get_both_rows();
    return result[0] && result[3]>=3;
}
