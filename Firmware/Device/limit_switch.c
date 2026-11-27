#include "limit_switch.h"
#include "bsp_gpio.h"
static limit_switch_t switches[3];
bsp_status_t limit_switch_configure(uint8_t j,bool high,uint32_t ms) {
 limit_switch_t empty={0};
 if(j>=3 || ms>0x7FFFFFFFU)return BSP_INVALID_ARGUMENT;
 switches[j]=empty; switches[j].configured=true; switches[j].active_high=high;
 switches[j].debounce_ms=ms; return BSP_OK;
}
bsp_status_t limit_switch_filter(limit_switch_t *s,bool pin,uint32_t now) {
 bool active;
 if(!s)return BSP_INVALID_ARGUMENT;
 if(!s->configured)return BSP_NOT_READY;
 active=pin==s->active_high; s->raw=pin;
 if(!s->initialized || active!=s->candidate) {
 s->candidate=active; s->changed_ms=now; s->initialized=true;
 }
 if((uint32_t)(now-s->changed_ms)>=s->debounce_ms) {
 s->debounced=s->candidate; s->valid=true;
 }
 return s->valid?BSP_OK:BSP_NOT_READY;
}
bsp_status_t limit_switch_read(uint8_t j,uint32_t now,limit_switch_t *out) {
 bool raw; bsp_status_t s;
 if(j>=3 || !out)return BSP_INVALID_ARGUMENT;
 if(!switches[j].configured)return BSP_NOT_READY;
 s=bsp_gpio_read_limit_raw(j,&raw);
 if(s!=BSP_OK) {switches[j].valid=false; *out=switches[j]; return s;}
 s=limit_switch_filter(&switches[j],raw,now); *out=switches[j]; return s;
}
