/* C-car application entry point. GPL-3.0-or-later. */
#include "zf_common_headfile.h"
#include "car_app.h"
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);
    interrupt_global_enable(0);
    car_app_init();
    while(1) car_app_foreground();
}
