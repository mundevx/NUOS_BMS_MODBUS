#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef enum { NUOS_C6_LED_BOOT=0, NUOS_C6_LED_H2_OFFLINE, NUOS_C6_LED_CONFIG_SYNC, NUOS_C6_LED_READY, NUOS_C6_LED_ERROR } nuos_c6_led_state_t;
void nuos_c6_led_init(void);
void nuos_c6_led_set_state(nuos_c6_led_state_t state);
void nuos_c6_led_modbus_activity(void);
