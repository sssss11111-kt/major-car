/* Active-low debounced button. Boot-held keys must be released first. */
#ifndef CAR_BUTTON_H
#define CAR_BUTTON_H
#include <stdint.h>
typedef struct { uint8_t raw,stable,armed; uint32_t changed; } car_button_t;
/* 1=press, 2=release. One event per stable edge, 30ms debounce. */
static uint8_t car_button_poll(car_button_t *b,uint8_t level,uint32_t now) {
    level=(uint8_t)(level!=0);
    if(level!=b->raw){b->raw=level;b->changed=now;}
    if((uint32_t)(now-b->changed)<30)return 0;
    if(!b->armed){if(level){b->stable=1;b->armed=1;}else b->stable=0;return 0;}
    if(level==b->stable)return 0;
    b->stable=level;return level?2:1;
}
#endif
