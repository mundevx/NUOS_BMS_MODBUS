#include "nuos_c6_led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

#define NUOS_C6_LED_GPIO 8
static led_strip_handle_t strip;
static volatile nuos_c6_led_state_t state=NUOS_C6_LED_BOOT;
static volatile int64_t pulse_until_us=0;

static void rgb(uint8_t r,uint8_t g,uint8_t b,bool on){
    if(!strip)return;
    if(on) led_strip_set_pixel(strip,0,r,g,b); else led_strip_clear(strip);
    led_strip_refresh(strip);
}
static void pattern(nuos_c6_led_state_t s,uint8_t *r,uint8_t *g,uint8_t *b,uint16_t *on,uint16_t *off){
    switch(s){
    case NUOS_C6_LED_H2_OFFLINE:*r=255;*g=0;*b=0;*on=250;*off=750;break;
    case NUOS_C6_LED_CONFIG_SYNC:*r=255;*g=120;*b=0;*on=180;*off=180;break;
    case NUOS_C6_LED_READY:*r=0;*g=255;*b=0;*on=150;*off=1850;break;
    case NUOS_C6_LED_ERROR:*r=255;*g=0;*b=0;*on=80;*off=80;break;
    default:*r=255;*g=255;*b=255;*on=250;*off=750;break;
    }
}
static void task(void *arg){(void)arg;bool was=false;int64_t start=esp_timer_get_time()/1000;nuos_c6_led_state_t prev=(nuos_c6_led_state_t)99;
    for(;;){
        nuos_c6_led_state_t s=state;
        if(s!=prev){prev=s;start=esp_timer_get_time()/1000;was=false;rgb(0,0,0,false);}
        if(esp_timer_get_time()<pulse_until_us){if(!was){was=true;rgb(0,80,255,true);}vTaskDelay(pdMS_TO_TICKS(25));continue;}
        uint8_t r,g,b;uint16_t on,off;pattern(s,&r,&g,&b,&on,&off);uint32_t cyc=on+off;bool should=((esp_timer_get_time()/1000-start)%cyc)<on;
        if(should!=was){was=should;rgb(r,g,b,should);}vTaskDelay(pdMS_TO_TICKS(25));
    }
}
void nuos_c6_led_init(void){
    led_strip_config_t c={.strip_gpio_num=NUOS_C6_LED_GPIO,.max_leds=1,.led_model=LED_MODEL_WS2812,.color_component_format=LED_STRIP_COLOR_COMPONENT_FMT_GRB,.flags.invert_out=false};
    led_strip_rmt_config_t r={.clk_src=RMT_CLK_SRC_DEFAULT,.resolution_hz=10*1000*1000,.flags.with_dma=false};
    if(led_strip_new_rmt_device(&c,&r,&strip)!=ESP_OK){strip=NULL;state=NUOS_C6_LED_ERROR;return;}
    xTaskCreate(task,"c6_rgb_led",3072,NULL,3,NULL);
}
void nuos_c6_led_set_state(nuos_c6_led_state_t s){state=s;}
void nuos_c6_led_modbus_activity(void){pulse_until_us=esp_timer_get_time()+120000;}
