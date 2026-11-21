#ifndef DEVICE_TEMPLATES_H
#define DEVICE_TEMPLATES_H
#include "app_config.h"
#include <stddef.h>

size_t device_template_load(uint8_t device_type, uint8_t endpoint, endpoint_config_t *cfg);
void device_templates_json(char *out, size_t out_len);

#endif
