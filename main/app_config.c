#include "app_config.h"
#include <esp_log.h>
#include <esp_err.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "nuos_link_proto.h"
#include "device_templates.h"

static const char *TAG="APP_CONFIG";

/* Forward declaration: app_config_init() invokes this before its definition. */
static void migrate_existing_lt_endpoints(void);
#define NVS_NS "bms_cfg"
#define CFG_MAGIC 0x4E434647u /* NCFG */
#define CFG_VERSION 2u
#define TEMPLATE_NVS_NS "bms_tpl"
#define TEMPLATE_KEY_FMT "t%u"

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t attribute_count;
    uint8_t enabled;
    uint8_t slave_id;
    uint8_t device_type;
    uint16_t cluster_id;
} endpoint_blob_hdr_t;

static const serial_config_t DEFAULT_SERIAL={9600,8,1,0};
static const mqtt_display_config_t DEFAULT_MQTT={"",1883,"","","NUOS-C6","bms/data"};
#define ENDPOINT_COUNT_KEY "ep_count"
#define LT_STATION_KEY "lt_station"
#define DEFAULT_LT_STATION 1u
#define DEFAULT_ENDPOINT_COUNT 1u

static esp_err_t get_blob(const char *key,void *dst,size_t len,const void *def){nvs_handle_t h;esp_err_t e=nvs_open(NVS_NS,NVS_READONLY,&h);if(e!=ESP_OK){memcpy(dst,def,len);return ESP_OK;}size_t n=len;e=nvs_get_blob(h,key,dst,&n);nvs_close(h);if(e!=ESP_OK||n!=len)memcpy(dst,def,len);return ESP_OK;}
static esp_err_t set_blob(const char *key,const void *src,size_t len){nvs_handle_t h;esp_err_t e=nvs_open(NVS_NS,NVS_READWRITE,&h);if(e!=ESP_OK)return e;e=nvs_set_blob(h,key,src,len);if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);return e;}

static esp_err_t get_blob_ns(const char *ns,const char *key,void *dst,size_t *len)
{
    if (!ns || !key || !dst || !len) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e=nvs_open(ns,NVS_READONLY,&h);
    if(e!=ESP_OK) return e;
    e=nvs_get_blob(h,key,dst,len);
    nvs_close(h);
    return e;
}

static esp_err_t set_blob_ns(const char *ns,const char *key,const void *src,size_t len)
{
    if (!ns || !key || !src || !len) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e=nvs_open(ns,NVS_READWRITE,&h);
    if(e!=ESP_OK) return e;
    e=nvs_set_blob(h,key,src,len);
    if(e==ESP_OK) e=nvs_commit(h);
    nvs_close(h);
    return e;
}

static esp_err_t config_to_blob(const endpoint_config_t *c, uint8_t **out, size_t *out_sz)
{
    if(!c || !out || !out_sz || (c->attribute_count && !c->attributes)) return ESP_ERR_INVALID_ARG;
    if(c->attribute_count > APP_MAX_ATTRIBUTES) return ESP_ERR_INVALID_SIZE;
    size_t sz=sizeof(endpoint_blob_hdr_t)+(size_t)c->attribute_count*sizeof(modbus_attribute_t);
    uint8_t *blob=calloc(1,sz);
    if(!blob) return ESP_ERR_NO_MEM;
    endpoint_blob_hdr_t hdr={CFG_MAGIC,CFG_VERSION,c->attribute_count,c->enabled?1:0,c->slave_id,c->device_type,c->cluster_id};
    memcpy(blob,&hdr,sizeof(hdr));
    if(c->attribute_count) memcpy(blob+sizeof(hdr),c->attributes,(size_t)c->attribute_count*sizeof(modbus_attribute_t));
    *out=blob; *out_sz=sz; return ESP_OK;
}

static esp_err_t blob_to_config(const uint8_t *blob,size_t got,uint8_t ep,endpoint_config_t *c)
{
    if(!blob || !c || got<sizeof(endpoint_blob_hdr_t)) return ESP_ERR_INVALID_ARG;
    endpoint_blob_hdr_t hdr; memcpy(&hdr,blob,sizeof(hdr));
    if (hdr.attribute_count > APP_MAX_ATTRIBUTES) return ESP_ERR_INVALID_SIZE;
    size_t need=sizeof(hdr)+(size_t)hdr.attribute_count*sizeof(modbus_attribute_t);
    if(hdr.magic!=CFG_MAGIC || hdr.version!=CFG_VERSION || (ep && hdr.slave_id!=ep) || need!=got) return ESP_ERR_INVALID_SIZE;
    c->enabled=hdr.enabled!=0; c->slave_id=ep?ep:hdr.slave_id; c->device_type=hdr.device_type; c->cluster_id=hdr.cluster_id; c->attribute_count=hdr.attribute_count;
    if(c->attribute_count){
        c->attributes=malloc((size_t)c->attribute_count*sizeof(modbus_attribute_t));
        if(!c->attributes){ c->attribute_count=0; return ESP_ERR_NO_MEM; }
        memcpy(c->attributes,blob+sizeof(hdr),(size_t)c->attribute_count*sizeof(modbus_attribute_t));
    }
    return ESP_OK;
}

void app_config_free_endpoint(endpoint_config_t *cfg){if(!cfg)return;free(cfg->attributes);cfg->attributes=NULL;cfg->attribute_count=0;}
static uint16_t cluster_for_endpoint(uint8_t dev, uint8_t ep)
{
    switch (dev) {
        case DEVICE_LIGHTING:        return 0xFC00;
        case DEVICE_LT_PANEL:        return 0xFC10;
        case DEVICE_DG:              return 0xFC20;
        case DEVICE_UPS:             return (ep == 2) ? 0xFC31 : 0xFC30;
        case DEVICE_LIFT:            return 0xFC40;
        case DEVICE_ESCALATOR:       return 0xFC50;
        case DEVICE_FIRE_HYDRANT:    return 0xFC60;
        case DEVICE_ANEMOMETER:      return 0xFC70;
        case DEVICE_AC:              return 0xFC80;
        case DEVICE_VENTILATION_FAN: return 0xFC90;
        case DEVICE_SLD:             return 0xFCB0;
        case DEVICE_CUSTOM:          return 0xFCE0;
        default:                     return 0;
    }
}

static void endpoint_default(uint8_t ep,endpoint_config_t *c){memset(c,0,sizeof(*c));c->slave_id=ep;c->device_type=DEVICE_CUSTOM;c->cluster_id=0;}

esp_err_t app_config_init(void)
{
    esp_err_t e=nvs_flash_init();
    if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        e=nvs_flash_init();
    }
    if(e!=ESP_OK) return e;

    e = app_config_seed_device_templates();
    if (e != ESP_OK) return e;

    /*
     * Normalize any already-saved LT endpoints before the Modbus task starts.
     * No NVS erase is required.
     */
    migrate_existing_lt_endpoints();
    return ESP_OK;
}
esp_err_t app_config_load_serial(serial_config_t *c){return c?get_blob("serial",c,sizeof(*c),&DEFAULT_SERIAL):ESP_ERR_INVALID_ARG;}
esp_err_t app_config_load_lt_station(uint8_t *station_id)
{
    if (!station_id) return ESP_ERR_INVALID_ARG;
    *station_id = DEFAULT_LT_STATION;
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (e != ESP_OK) return ESP_OK;
    uint8_t value = DEFAULT_LT_STATION;
    e = nvs_get_u8(h, LT_STATION_KEY, &value);
    nvs_close(h);
    if (e == ESP_OK && value >= 1 && value <= 5) {
        *station_id = value;
    }
    return ESP_OK;
}
esp_err_t app_config_save_lt_station(uint8_t station_id)
{
    if (station_id < 1 || station_id > 5) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_u8(h, LT_STATION_KEY, station_id);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}
esp_err_t app_config_load_endpoint_count(uint8_t *count)
{
    if (!count) return ESP_ERR_INVALID_ARG;
    *count = DEFAULT_ENDPOINT_COUNT;
    nvs_handle_t h;
    esp_err_t e=nvs_open(NVS_NS,NVS_READONLY,&h);
    if(e!=ESP_OK) return ESP_OK;
    uint8_t v=DEFAULT_ENDPOINT_COUNT;
    e=nvs_get_u8(h,ENDPOINT_COUNT_KEY,&v);
    if(e==ESP_OK && v>=1 && v<=APP_MAX_ENDPOINTS){
        *count=v;
        nvs_close(h);
        return ESP_OK;
    }

    /* Compatibility: if Endpoint Count was introduced after endpoint
     * configuration already existed, derive the count from the highest
     * saved endpoint instead of resetting the UI to 1. */
    uint8_t highest=0;
    for(uint8_t ep=1;ep<=APP_MAX_ENDPOINTS;ep++){
        char key[16];
        snprintf(key,sizeof(key),"ep%u",ep);
        size_t sz=0;
        if(nvs_get_blob(h,key,NULL,&sz)==ESP_OK && sz>0) highest=ep;
    }
    nvs_close(h);
    if(highest>=1) *count=highest;
    return ESP_OK;
}

esp_err_t app_config_save_endpoint_count(uint8_t count)
{
    if(count<1 || count>APP_MAX_ENDPOINTS) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t e=nvs_open(NVS_NS,NVS_READWRITE,&h);
    if(e!=ESP_OK) return e;
    e=nvs_set_u8(h,ENDPOINT_COUNT_KEY,count);
    if(e==ESP_OK) e=nvs_commit(h);
    nvs_close(h);
    return e;
}

esp_err_t app_config_save_serial(const serial_config_t *c){return c?set_blob("serial",c,sizeof(*c)):ESP_ERR_INVALID_ARG;}


/*
 * Compatibility migration for the previous UPS model.
 *
 * Current model:
 *   EP1 = normal UPS measurements/status, cluster FC30
 *   EP2 = UPS fault/error model, cluster FC31
 *
 * The old firmware used a 16/12 split of the normal UPS register map.
 * EP1's old 16-attribute layout is migrated to the complete 28-attribute
 * EP1 template. EP2 is migrated to the new fault/error template only when
 * it is still carrying the old generic/FC30 cluster. Once EP2 is explicitly
 * saved as FC31, it is never overwritten by this compatibility migration.
 */
static bool s_ups_migration_running = false;
static void migrate_legacy_ups_endpoint_if_needed(uint8_t ep, endpoint_config_t *c)
{
    if (s_ups_migration_running || !c || !c->enabled ||
        c->device_type != DEVICE_UPS || !c->attributes || ep != 1 ||
        c->attribute_count != 16) {
        return;
    }

    s_ups_migration_running = true;

    endpoint_config_t replacement = {0};
    size_t count = device_template_load(DEVICE_UPS, ep, &replacement);
    if (count == 28 && replacement.attributes) {
        replacement.cluster_id = c->cluster_id ? c->cluster_id : replacement.cluster_id;
        esp_err_t e = app_config_save_endpoint(ep, &replacement);
        if (e == ESP_OK) {
            ESP_LOGW(TAG,
                     "Migrated legacy UPS EP%u: %u attrs -> 28 attrs (same register set for every slave)",
                     ep, c->attribute_count);
            app_config_free_endpoint(c);
            *c = replacement;
            memset(&replacement, 0, sizeof(replacement));
        } else {
            ESP_LOGE(TAG, "UPS EP%u migration save failed: %s", ep, esp_err_to_name(e));
        }
    } else {
        ESP_LOGE(TAG, "UPS EP%u migration failed: could not build 28-attribute template", ep);
    }

    app_config_free_endpoint(&replacement);
    s_ups_migration_running = false;
}

static bool s_ups_fault_migration_running = false;
static void migrate_legacy_ups_fault_endpoint_if_needed(uint8_t ep, endpoint_config_t *c)
{
    if (s_ups_fault_migration_running || !c || !c->enabled || ep != 2 ||
        c->device_type != DEVICE_UPS || !c->attributes ||
        (c->cluster_id != 0 && c->cluster_id != 0xFC30 && c->cluster_id != 0xFCE0)) {
        return;
    }

    s_ups_fault_migration_running = true;
    endpoint_config_t replacement = {0};
    size_t count = device_template_load(DEVICE_UPS, 2, &replacement);
    if (count == 4 && replacement.attributes) {
        esp_err_t e = app_config_save_endpoint(2, &replacement);
        if (e == ESP_OK) {
            ESP_LOGW(TAG, "Migrated legacy UPS EP2 to the distinct FC31 fault/error template");
            app_config_free_endpoint(c);
            *c = replacement;
            memset(&replacement, 0, sizeof(replacement));
        } else {
            ESP_LOGE(TAG, "UPS EP2 fault/error migration save failed: %s", esp_err_to_name(e));
        }
    }
    app_config_free_endpoint(&replacement);
    s_ups_fault_migration_running = false;
}

/* Migrate the old Lift template, where the endpoint number selected only
 * one status register.  The endpoint is a Modbus slave instance; every Lift
 * endpoint now receives the complete Lift register set. */
static bool s_lift_migration_running = false;
static void migrate_legacy_lift_endpoint_if_needed(uint8_t ep, endpoint_config_t *c)
{
    if (s_lift_migration_running || !c || !c->enabled ||
        c->device_type != DEVICE_LIFT || !c->attributes ||
        c->attribute_count == 7) {
        return;
    }

    s_lift_migration_running = true;
    endpoint_config_t replacement = {0};
    size_t count = device_template_load(DEVICE_LIFT, ep, &replacement);
    if (count == 7 && replacement.attributes) {
        replacement.cluster_id = c->cluster_id ? c->cluster_id : replacement.cluster_id;
        esp_err_t e = app_config_save_endpoint(ep, &replacement);
        if (e == ESP_OK) {
            ESP_LOGW(TAG,
                     "Migrated legacy Lift EP%u: %u attrs -> 7 attrs (same register set for every slave)",
                     ep, c->attribute_count);
            app_config_free_endpoint(c);
            *c = replacement;
            memset(&replacement, 0, sizeof(replacement));
        } else {
            ESP_LOGE(TAG, "Lift EP%u migration save failed: %s", ep, esp_err_to_name(e));
        }
    } else {
        ESP_LOGE(TAG, "Lift EP%u migration failed: could not build 7-attribute template", ep);
    }
    app_config_free_endpoint(&replacement);
    s_lift_migration_running = false;
}

/*
 * LT Panel configuration migration.
 *
 * LT Panel uses station-specific FC04 profiles selected in the web UI.
 * The exact, known legacy 12-attribute firmware map and stored configurations
 * that match a built-in station profile are migrated automatically. Arbitrary
 * custom LT Panel endpoint mappings are preserved; the user can explicitly
 * replace them from the station-profile control.
 */
static bool is_known_legacy_lt_endpoint(const endpoint_config_t *c)
{
    static const char *const names[12] = {
        "Voltage R", "Voltage Y", "Voltage B",
        "Current R", "Current Y", "Current B",
        "Apparent Power R", "Apparent Power Y", "Apparent Power B",
        "Frequency", "Power Consumption", "Power Factor"
    };
    static const uint16_t pdu_map[12] = {
        29, 31, 33, 35, 37, 39, 67, 69, 71, 131, 125, 161
    };
    static const uint16_t old_csv_map[12] = {
        3029, 3031, 3033, 3035, 3037, 3039,
        3067, 3069, 3071, 3131, 3125, 3161
    };

    if (!c || c->device_type != DEVICE_LT_PANEL ||
        c->attribute_count != 12 || !c->attributes) return false;

    for (uint16_t i = 0; i < 12; i++) {
        const modbus_attribute_t *a = &c->attributes[i];
        bool known_name = strcmp(a->name, names[i]) == 0;
        if (i == 10 && strcmp(a->name, "Power Consumption kWh") == 0) {
            known_name = true;
        }
        if (!known_name || a->function != MB_FC04 ||
            (a->address != pdu_map[i] && a->address != old_csv_map[i])) {
            return false;
        }
    }
    return true;
}

/*
 * Recognize an LT Panel configuration generated by one of the built-in
 * station profiles, allowing the previous one-location UINT16 width (1) to
 * be migrated to the new two-location byte-pair width (2). Custom maps are
 * left untouched unless every attribute identity matches a known profile.
 */
static bool lt_profile_mapping_matches_for_width_migration(
    const endpoint_config_t *stored, const endpoint_config_t *profile,
    bool *needs_update)
{
    if (needs_update) *needs_update = false;
    if (!stored || !profile || stored->device_type != DEVICE_LT_PANEL ||
        profile->device_type != DEVICE_LT_PANEL ||
        stored->attribute_count != profile->attribute_count ||
        !stored->attributes || !profile->attributes) {
        return false;
    }

    bool changed = false;
    for (uint16_t i = 0; i < stored->attribute_count; i++) {
        const modbus_attribute_t *old = &stored->attributes[i];
        const modbus_attribute_t *now = &profile->attributes[i];
        if (old->attribute_id != now->attribute_id ||
            old->function != now->function ||
            old->address != now->address ||
            old->data_type != now->data_type ||
            old->word_order != now->word_order ||
            old->packed_count != now->packed_count ||
            old->address2 != now->address2 ||
            old->address3 != now->address3 ||
            strcmp(old->name, now->name) != 0) {
            return false;
        }

        if (old->reg_count != now->reg_count) {
            if (!(now->reg_count == 2 && old->reg_count == 1 &&
                  (now->data_type == MB_TYPE_UINT16 ||
                   now->data_type == MB_TYPE_INT16))) {
                return false;
            }
            changed = true;
        }
    }

    if (needs_update) *needs_update = changed;
    return true;
}

/* Recognize the previous Station_01 map, where only one power-factor
 * attribute at PDU 117 existed, and expand it to the three-phase map:
 * PF-R 117-118, PF-Y 119-120, PF-B 121-122.
 *
 * This migration is deliberately strict: all eight non-PF attributes must
 * still match the Station_01 profile, and the old ninth attribute must be the
 * single Power Factor value at 117. Arbitrary/custom endpoint maps are not
 * changed.
 */
static bool lt_station01_single_pf_matches_three_phase_profile(
    const endpoint_config_t *stored,
    const endpoint_config_t *profile)
{
    if (!stored || !profile ||
        stored->device_type != DEVICE_LT_PANEL ||
        profile->device_type != DEVICE_LT_PANEL ||
        stored->attribute_count != 9 || profile->attribute_count != 11 ||
        !stored->attributes || !profile->attributes) {
        return false;
    }

    for (uint16_t i = 0; i < 8; i++) {
        const modbus_attribute_t *old = &stored->attributes[i];
        const modbus_attribute_t *now = &profile->attributes[i];
        if (old->attribute_id != now->attribute_id ||
            old->function != now->function ||
            old->address != now->address ||
            old->data_type != now->data_type ||
            old->word_order != now->word_order ||
            old->packed_count != now->packed_count ||
            old->address2 != now->address2 ||
            old->address3 != now->address3 ||
            strcmp(old->name, now->name) != 0) {
            return false;
        }
        if (old->reg_count != now->reg_count &&
            !(old->reg_count == 1 && now->reg_count == 2 &&
              (now->data_type == MB_TYPE_UINT16 ||
               now->data_type == MB_TYPE_INT16))) {
            return false;
        }
    }

    const modbus_attribute_t *old_pf = &stored->attributes[8];
    const modbus_attribute_t *new_pf_r = &profile->attributes[8];
    if (old_pf->attribute_id != 8 || old_pf->function != MB_FC04 ||
        old_pf->address != 117 ||
        (strcmp(old_pf->name, "Power Factor") != 0 &&
         strcmp(old_pf->name, "Power Factor R") != 0) ||
        old_pf->data_type != new_pf_r->data_type ||
        old_pf->word_order != new_pf_r->word_order ||
        old_pf->packed_count != new_pf_r->packed_count ||
        old_pf->address2 != new_pf_r->address2 ||
        old_pf->address3 != new_pf_r->address3 ||
        new_pf_r->attribute_id != 8 || new_pf_r->function != MB_FC04 ||
        new_pf_r->address != 117 || new_pf_r->reg_count != 2 ||
        strcmp(new_pf_r->name, "Power Factor R") != 0 ||
        (old_pf->reg_count != 1 && old_pf->reg_count != 2)) {
        return false;
    }

    static const uint16_t expected_addresses[3] = {117, 119, 121};
    static const char *const expected_names[3] = {
        "Power Factor R", "Power Factor Y", "Power Factor B"
    };
    for (uint16_t i = 0; i < 3; i++) {
        const modbus_attribute_t *a = &profile->attributes[8 + i];
        if (a->attribute_id != 8 + i || a->function != MB_FC04 ||
            a->address != expected_addresses[i] || a->reg_count != 2 ||
            a->data_type != old_pf->data_type ||
            strcmp(a->name, expected_names[i]) != 0) {
            return false;
        }
    }
    return true;
}

static bool migrate_lt_endpoint_station01_power_factor_phases(
    endpoint_config_t *stored)
{
    if (!stored || stored->device_type != DEVICE_LT_PANEL) return false;

    endpoint_config_t station01_profile = {0};
    esp_err_t e = device_template_load_lt_panel_profile(
        LT_PANEL_STATION_01, stored->slave_id, &station01_profile, NULL);
    bool matches = e == ESP_OK &&
        lt_station01_single_pf_matches_three_phase_profile(
            stored, &station01_profile);
    if (matches) {
        /* Replace the endpoint with the canonical profile so the two new
         * phase attributes receive stable IDs 9 and 10 and two-byte reads. */
        bool was_enabled = stored->enabled;
        uint16_t cluster_id = stored->cluster_id;
        app_config_free_endpoint(stored);
        *stored = station01_profile;
        memset(&station01_profile, 0, sizeof(station01_profile));
        stored->enabled = was_enabled;
        stored->cluster_id = cluster_id ? cluster_id : 0xFC10;
        ESP_LOGW(TAG,
                 "LT Panel EP%u Station_01 migrated: Power Factor expanded to R/Y/B at 117/119/121 (11 attributes)",
                 stored->slave_id);
    }
    app_config_free_endpoint(&station01_profile);
    return matches;
}

static bool migrate_lt_endpoint_byte_pair_widths(endpoint_config_t *stored)
{
    if (!stored || stored->device_type != DEVICE_LT_PANEL) return false;

    for (uint8_t station_id = 1; station_id <= 5; station_id++) {
        endpoint_config_t profile = {0};
        bool needs_update = false;
        esp_err_t e = device_template_load_lt_panel_profile(
            station_id, stored->slave_id, &profile, NULL);
        if (e == ESP_OK &&
            lt_profile_mapping_matches_for_width_migration(
                stored, &profile, &needs_update) && needs_update) {
            for (uint16_t i = 0; i < stored->attribute_count; i++) {
                stored->attributes[i].reg_count = profile.attributes[i].reg_count;
            }
            ESP_LOGW(TAG,
                     "LT Panel EP%u matched station profile %u; migrated UINT16 reads to two addresses",
                     stored->slave_id, station_id);
            app_config_free_endpoint(&profile);
            return true;
        }
        app_config_free_endpoint(&profile);
    }
    return false;
}

static void migrate_existing_lt_endpoints(void)
{
    uint8_t count = 0;
    uint8_t station_id = DEFAULT_LT_STATION;
    if (app_config_load_endpoint_count(&count) != ESP_OK) return;
    (void)app_config_load_lt_station(&station_id);

    for (uint8_t ep = 1; ep <= count; ep++) {
        endpoint_config_t current = {0};
        if (app_config_load_endpoint(ep, &current) != ESP_OK) {
            app_config_free_endpoint(&current);
            continue;
        }

        if (is_known_legacy_lt_endpoint(&current)) {
            endpoint_config_t replacement = {0};
            esp_err_t e = device_template_load_lt_panel_profile(
                station_id, ep, &replacement, NULL);
            if (e == ESP_OK) {
                replacement.enabled = current.enabled;
                replacement.cluster_id = current.cluster_id ? current.cluster_id : 0xFC10;
                e = app_config_save_endpoint(ep, &replacement);
            }
            if (e == ESP_OK) {
                ESP_LOGW(TAG,
                         "Migrated legacy LT Panel EP%u to station profile ID %u (%u attributes)",
                         ep, station_id, replacement.attribute_count);
            } else {
                ESP_LOGE(TAG,
                         "Legacy LT Panel EP%u migration failed: %s",
                         ep, esp_err_to_name(e));
            }
            app_config_free_endpoint(&replacement);
        } else if (migrate_lt_endpoint_station01_power_factor_phases(&current)) {
            esp_err_t e = app_config_save_endpoint(ep, &current);
            if (e != ESP_OK) {
                ESP_LOGE(TAG,
                         "LT Panel EP%u three-phase power-factor migration save failed: %s",
                         ep, esp_err_to_name(e));
            }
        } else if (migrate_lt_endpoint_byte_pair_widths(&current)) {
            esp_err_t e = app_config_save_endpoint(ep, &current);
            if (e != ESP_OK) {
                ESP_LOGE(TAG,
                         "LT Panel EP%u byte-pair width migration save failed: %s",
                         ep, esp_err_to_name(e));
            }
        }

        app_config_free_endpoint(&current);
    }
}

esp_err_t app_config_load_endpoint(uint8_t ep,endpoint_config_t *c){
    if(!c||ep<1||ep>APP_MAX_ENDPOINTS)return ESP_ERR_INVALID_ARG;
    memset(c,0,sizeof(*c)); endpoint_default(ep,c);
    char key[16];snprintf(key,sizeof(key),"ep%u",ep);
    nvs_handle_t h;esp_err_t e=nvs_open(NVS_NS,NVS_READONLY,&h);if(e!=ESP_OK)return ESP_OK;
    size_t sz=0;e=nvs_get_blob(h,key,NULL,&sz);if(e!=ESP_OK||sz<sizeof(endpoint_blob_hdr_t)){nvs_close(h);return ESP_OK;}
    uint8_t *blob=malloc(sz);if(!blob){nvs_close(h);return ESP_ERR_NO_MEM;}
    size_t got=sz;e=nvs_get_blob(h,key,blob,&got);nvs_close(h);if(e!=ESP_OK){free(blob);return e;}
    endpoint_blob_hdr_t hdr;memcpy(&hdr,blob,sizeof(hdr));
    if (hdr.attribute_count > APP_MAX_ATTRIBUTES) {
        ESP_LOGW(TAG, "EP%u invalid config blob: attribute_count=%u", ep, hdr.attribute_count);
        free(blob);
        return ESP_OK;
    }
    size_t need=sizeof(hdr)+(size_t)hdr.attribute_count*sizeof(modbus_attribute_t);
    if(hdr.magic!=CFG_MAGIC||hdr.version!=CFG_VERSION||hdr.slave_id!=ep||need!=got){ESP_LOGW(TAG,"EP%u invalid config blob; using defaults",ep);free(blob);return ESP_OK;}
    c->enabled=hdr.enabled!=0;c->slave_id=hdr.slave_id;c->device_type=hdr.device_type;c->cluster_id=hdr.cluster_id;c->attribute_count=hdr.attribute_count;
    if(c->attribute_count){c->attributes=malloc((size_t)c->attribute_count*sizeof(modbus_attribute_t));if(!c->attributes){free(blob);app_config_free_endpoint(c);return ESP_ERR_NO_MEM;}memcpy(c->attributes,blob+sizeof(hdr),(size_t)c->attribute_count*sizeof(modbus_attribute_t));}
    free(blob);
    migrate_legacy_ups_endpoint_if_needed(ep, c);
    migrate_legacy_ups_fault_endpoint_if_needed(ep, c);
    migrate_legacy_lift_endpoint_if_needed(ep, c);

    /* Normalize legacy generic-cluster configurations to the module-specific
     * cluster IDs used by the Zigbee2MQTT/Home Assistant converter. */
    uint16_t expected_cluster = cluster_for_endpoint(c->device_type, ep);
    /* Preserve an explicitly selected cluster. Only migrate legacy zero/FCExxx
     * generic clusters to the module/endpoint default. */
    if (expected_cluster != 0 && (c->cluster_id == 0 || c->cluster_id == 0xFCE0) &&
        c->cluster_id != expected_cluster) {
        c->cluster_id = expected_cluster;
        esp_err_t me = app_config_save_endpoint(ep, c);
        if (me != ESP_OK) {
            ESP_LOGW(TAG, "EP%u cluster migration save failed: %s", ep, esp_err_to_name(me));
        } else {
            ESP_LOGI(TAG, "EP%u cluster migrated to 0x%04X for device_type=%u", ep, expected_cluster, c->device_type);
        }
    }
    return ESP_OK;
}

esp_err_t app_config_save_endpoint(uint8_t ep,const endpoint_config_t *c)
{
    if(!c||ep<1||ep>APP_MAX_ENDPOINTS||c->slave_id!=ep)return ESP_ERR_INVALID_ARG;
    if(c->attribute_count > APP_MAX_ATTRIBUTES) return ESP_ERR_INVALID_SIZE;
    uint8_t *blob=NULL; size_t sz=0;
    esp_err_t e=config_to_blob(c,&blob,&sz);
    if(e!=ESP_OK)return e;
    char key[16]; snprintf(key,sizeof(key),"ep%u",ep);
    e=set_blob(key,blob,sz);
    free(blob); return e;
}

esp_err_t app_config_save_template(uint8_t device_type,const endpoint_config_t *c)
{
    if(!c || device_type>=DEVICE_COUNT) return ESP_ERR_INVALID_ARG;
    uint8_t *blob=NULL; size_t sz=0;
    esp_err_t e=config_to_blob(c,&blob,&sz);
    if(e!=ESP_OK)return e;
    char key[16]; snprintf(key,sizeof(key),TEMPLATE_KEY_FMT,(unsigned)device_type);
    e=set_blob_ns(TEMPLATE_NVS_NS,key,blob,sz);
    free(blob); return e;
}

esp_err_t app_config_load_template(uint8_t device_type,endpoint_config_t *c)
{
    if(!c || device_type>=DEVICE_COUNT) return ESP_ERR_INVALID_ARG;
    memset(c,0,sizeof(*c));
    char key[16]; snprintf(key,sizeof(key),TEMPLATE_KEY_FMT,(unsigned)device_type);
    size_t sz=0;
    nvs_handle_t h; esp_err_t e=nvs_open(TEMPLATE_NVS_NS,NVS_READONLY,&h);
    if(e==ESP_OK){ e=nvs_get_blob(h,key,NULL,&sz); nvs_close(h); }
    if(e!=ESP_OK) return ESP_ERR_NVS_NOT_FOUND;
    uint8_t *blob=malloc(sz); if(!blob)return ESP_ERR_NO_MEM;
    size_t got=sz; e=get_blob_ns(TEMPLATE_NVS_NS,key,blob,&got);
    if(e==ESP_OK) {
        e=blob_to_config(blob,got,0,c);
        /*
         * Older firmware seeded the LT Panel device template with a fixed
         * 12-register map.  Remove that legacy firmware-owned template once.
         * A web-edited template is retained normally; endpoint_post writes it
         * back to NVS after the user configures the panel in the browser.
         */
        if (e == ESP_OK && device_type == DEVICE_LT_PANEL &&
            c->attribute_count == 12 && c->attributes) {
            static const char *const legacy_lt_names[12] = {
                "Voltage R", "Voltage Y", "Voltage B",
                "Current R", "Current Y", "Current B",
                "Apparent Power R", "Apparent Power Y", "Apparent Power B",
                "Frequency", "Power Consumption", "Power Factor"
            };
            static const uint16_t legacy_lt_pdu_zero_based[12] = {
                0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12
            };
            static const uint16_t legacy_lt_pdu_43xxx[12] = {
                29, 31, 33, 35, 37, 39, 67, 69, 71, 131, 125, 161
            };
            static const uint16_t legacy_lt_pdu_3029_map[12] = {
                3029, 3031, 3033, 3035, 3037, 3039,
                3067, 3069, 3071, 3131, 3125, 3161
            };
            bool legacy_template = true;
            for (uint16_t i = 0; i < 12; ++i) {
                uint16_t address = c->attributes[i].address;
                bool known_address =
                    address == legacy_lt_pdu_zero_based[i] ||
                    address == legacy_lt_pdu_43xxx[i] ||
                    address == legacy_lt_pdu_3029_map[i];
                if (strcmp(c->attributes[i].name, legacy_lt_names[i]) != 0 ||
                    c->attributes[i].function != MB_FC04 || !known_address) {
                    legacy_template = false;
                    break;
                }
            }
            if (legacy_template) {
                uint8_t station_id = DEFAULT_LT_STATION;
                (void)app_config_load_lt_station(&station_id);
                endpoint_config_t replacement = {0};
                esp_err_t re = device_template_load_lt_panel_profile(
                    station_id, 1, &replacement, NULL);
                if (re == ESP_OK) {
                    app_config_free_endpoint(c);
                    *c = replacement;
                    memset(&replacement, 0, sizeof(replacement));
                    esp_err_t me = app_config_save_template(device_type, c);
                    if (me != ESP_OK) ESP_LOGW(TAG, "LT Panel template migration save failed: %s", esp_err_to_name(me));
                    else ESP_LOGW(TAG, "Migrated legacy LT Panel template to Station_%02u profile", station_id == 1 ? 1 : station_id == 2 ? 7 : station_id == 3 ? 9 : station_id == 4 ? 10 : 11);
                }
                app_config_free_endpoint(&replacement);
            }
        }

        /* Expand the known previous Station_01 single-PF template to the
         * three-phase PF map. This only applies when every existing attribute
         * exactly matches the old built-in profile; custom maps are preserved. */
        if (e == ESP_OK && device_type == DEVICE_LT_PANEL && c->attributes) {
            endpoint_config_t station01_profile = {0};
            esp_err_t pe = device_template_load_lt_panel_profile(
                LT_PANEL_STATION_01, 1, &station01_profile, NULL);
            if (pe == ESP_OK &&
                lt_station01_single_pf_matches_three_phase_profile(
                    c, &station01_profile)) {
                endpoint_config_t replacement = {0};
                pe = device_template_load_lt_panel_profile(
                    LT_PANEL_STATION_01, 1, &replacement, NULL);
                if (pe == ESP_OK) {
                    replacement.enabled = c->enabled;
                    replacement.cluster_id = c->cluster_id ? c->cluster_id : 0xFC10;
                    app_config_free_endpoint(c);
                    *c = replacement;
                    memset(&replacement, 0, sizeof(replacement));
                    esp_err_t me = app_config_save_template(device_type, c);
                    if (me != ESP_OK) {
                        ESP_LOGW(TAG,
                                 "LT Panel template three-phase PF migration save failed: %s",
                                 esp_err_to_name(me));
                    } else {
                        ESP_LOGW(TAG,
                                 "Migrated LT Panel template: Power Factor R/Y/B addresses 117/119/121");
                    }
                }
                app_config_free_endpoint(&replacement);
            }
            app_config_free_endpoint(&station01_profile);
        }

        /* Upgrade stored built-in LT Panel profile templates from the old
         * one-address UINT16 form to the new two-address byte-pair form. */
        if (e == ESP_OK && device_type == DEVICE_LT_PANEL && c->attributes) {
            for (uint8_t station_id = 1; station_id <= 5; station_id++) {
                endpoint_config_t profile = {0};
                bool needs_update = false;
                esp_err_t pe = device_template_load_lt_panel_profile(
                    station_id, 1, &profile, NULL);
                if (pe == ESP_OK &&
                    lt_profile_mapping_matches_for_width_migration(
                        c, &profile, &needs_update) && needs_update) {
                    for (uint16_t i = 0; i < c->attribute_count; i++) {
                        c->attributes[i].reg_count = profile.attributes[i].reg_count;
                    }
                    esp_err_t me = app_config_save_template(device_type, c);
                    if (me != ESP_OK) {
                        ESP_LOGW(TAG,
                                 "LT Panel template byte-pair migration save failed: %s",
                                 esp_err_to_name(me));
                    } else {
                        ESP_LOGW(TAG,
                                 "Migrated stored LT Panel template to two-address UINT16 reads (station profile %u)",
                                 station_id);
                    }
                    app_config_free_endpoint(&profile);
                    break;
                }
                app_config_free_endpoint(&profile);
            }
        }
    }
    free(blob); return e;
}

esp_err_t app_config_seed_device_templates(void)
{
    for(uint8_t dt=0; dt<DEVICE_COUNT; dt++){
        endpoint_config_t existing={0};
        esp_err_t e=app_config_load_template(dt,&existing);
        if(e==ESP_OK){ app_config_free_endpoint(&existing); continue; }
        endpoint_config_t def={0};
        size_t count=device_template_load(dt,1,&def);
        if(count==0 && dt!=DEVICE_CUSTOM){
            /* Empty templates are still valid: they carry device type and cluster ID. */
        }
        def.slave_id=1;
        def.device_type=dt;
        e=app_config_save_template(dt,&def);
        app_config_free_endpoint(&def);
        if(e!=ESP_OK){ ESP_LOGE(TAG,"Template seed failed device=%u: %s",dt,esp_err_to_name(e)); return e; }
        ESP_LOGI(TAG,"Seeded device template %u into flash",dt);
    }
    return ESP_OK;
}

esp_err_t app_config_delete_endpoint(uint8_t ep){if(ep<1||ep>APP_MAX_ENDPOINTS)return ESP_ERR_INVALID_ARG;char key[16];snprintf(key,sizeof(key),"ep%u",ep);nvs_handle_t h;esp_err_t e=nvs_open(NVS_NS,NVS_READWRITE,&h);if(e!=ESP_OK)return e;e=nvs_erase_key(h,key);if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);return e==ESP_ERR_NVS_NOT_FOUND?ESP_OK:e;}
esp_err_t app_config_load_mqtt(mqtt_display_config_t *c){return c?get_blob("mqtt",c,sizeof(*c),&DEFAULT_MQTT):ESP_ERR_INVALID_ARG;}
esp_err_t app_config_save_mqtt(const mqtt_display_config_t *c){return c?set_blob("mqtt",c,sizeof(*c)):ESP_ERR_INVALID_ARG;}
esp_err_t app_config_reset_defaults(void)
{
    nvs_handle_t h; esp_err_t e=nvs_open(NVS_NS,NVS_READWRITE,&h);
    if(e!=ESP_OK)return e;
    e=nvs_erase_all(h); if(e==ESP_OK)e=nvs_commit(h); nvs_close(h);
    if(e!=ESP_OK)return e;
    nvs_handle_t th;
    e=nvs_open(TEMPLATE_NVS_NS,NVS_READWRITE,&th);
    if(e==ESP_OK){ e=nvs_erase_all(th); if(e==ESP_OK)e=nvs_commit(th); nvs_close(th); }
    if(e!=ESP_OK && e!=ESP_ERR_NVS_NOT_FOUND)return e;
    e=app_config_save_endpoint_count(DEFAULT_ENDPOINT_COUNT);
    if(e!=ESP_OK)return e;
    return app_config_seed_device_templates();
}

esp_err_t app_config_calculate_zigbee_hash(uint32_t *out,uint8_t *eps_out)
{
    if (!out || !eps_out) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t crc = 0xFFFFFFFFu;
    uint8_t active = 0;

    for (uint8_t ep = 1; ep <= APP_MAX_ENDPOINTS; ep++) {
        endpoint_config_t c = {0};
        c.slave_id = ep;

        esp_err_t e = app_config_load_endpoint(ep, &c);
        if (e != ESP_OK) {
            app_config_free_endpoint(&c);
            return e;
        }

        uint16_t enabled_count = 0;
        if (c.enabled && c.cluster_id != 0) {
            for (uint16_t i = 0; i < c.attribute_count; i++) {
                if (c.attributes[i].enabled) {
                    enabled_count++;
                }
            }
        }

        /* The endpoint count in ZB_CONFIG_BEGIN must exactly match the
         * endpoints that nuos_uart_link.c will actually transmit. */
        if (enabled_count != 0) {
            active++;
        }

        uint8_t head[8] = {
            ep,
            (enabled_count != 0) ? 1u : 0u,
            c.device_type,
            (uint8_t)c.cluster_id,
            (uint8_t)(c.cluster_id >> 8),
            (uint8_t)enabled_count,
            (uint8_t)(enabled_count >> 8),
            0
        };
        crc = nuos_link_crc32_update(crc, head, sizeof(head));

        /* Hash only the attributes that are actually sent to H2. */
        if (enabled_count != 0) {
            for (uint16_t i = 0; i < c.attribute_count; i++) {
                if (!c.attributes[i].enabled) {
                    continue;
                }
                uint8_t rec[2] = {
                    c.attributes[i].attribute_id,
                    c.attributes[i].data_type
                };
                crc = nuos_link_crc32_update(crc, rec, sizeof(rec));
            }
        }

        app_config_free_endpoint(&c);
    }

    *out = nuos_link_crc32_finish(crc);
    *eps_out = active;
    return ESP_OK;
}

const char *device_type_name(uint8_t t){static const char*n[]={"LT Panel / Panel MFM","DG","UPS","Lift","Escalator","Fire Hydrant","Anemometer","AC","Ventilation Fan","Custom","SLD","Lighting"};return t<DEVICE_COUNT?n[t]:"Custom";}
const char *modbus_data_type_name(uint8_t t){static const char*n[]={"BOOL","UINT16","INT16","UINT32","INT32","FLOAT32","UINT64","INT64","STRING","UINT8","INT8","UINT24_t","UINT48_t"};return t<=MB_TYPE_UINT48?n[t]:"UINT16";}
