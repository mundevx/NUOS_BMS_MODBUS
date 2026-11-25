#ifndef DEVICE_TEMPLATES_H
#define DEVICE_TEMPLATES_H
#include "app_config.h"
#include <stddef.h>

typedef struct {
    uint8_t station_id;
    const char *name;
    /* Informational station metadata; LT polling uses exact attributes. */
    uint16_t read_register_count;
    uint16_t skip_register_count;
    uint16_t address_offset;
    uint16_t max_variables;
    uint32_t baud_rate;             /* 0 = not specified by supplied station map */
    bool baud_rate_needs_verification;
    uint8_t phase_id_type_code;     /* device-type marker; not a Modbus register */
} lt_panel_profile_info_t;

enum {
    LT_PANEL_STATION_01 = 1,
    LT_PANEL_STATION_07 = 2,
    LT_PANEL_STATION_09 = 3,
    LT_PANEL_STATION_10 = 4,
    LT_PANEL_STATION_11 = 5
};

size_t device_template_load(uint8_t device_type, uint8_t endpoint, endpoint_config_t *cfg);
esp_err_t device_template_load_lt_panel_profile(uint8_t station_id, uint8_t endpoint,
                                                endpoint_config_t *cfg,
                                                lt_panel_profile_info_t *info);
esp_err_t device_template_get_lt_panel_profile_info(uint8_t station_id,
                                                      lt_panel_profile_info_t *info);
void device_templates_json(char *out, size_t out_len);

#endif
