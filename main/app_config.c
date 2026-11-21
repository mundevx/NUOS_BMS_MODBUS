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
 * The current LT design is:
 *   - Modbus FC04 (Input Registers)
 *   - internal/on-wire PDU addresses 0..12
 *   - 12 configured attributes
 *
 * Older firmware stored one of these legacy maps:
 *   A) 3029,3031,3033,3035,3037,3039,3067,3069,3071,3125,3131,3161
 *      (caused by treating the 43xxx display references as standard 4xxxx)
 *   B) 29,31,33,35,37,39,67,69,71,125,131,161
 *      (caused by directly converting 43030,43032,... with the generic
 *       4xxxx-style helper)
 *
 * Accept either legacy map, regardless of whether it was stored as FC03
 * or FC04, and rewrite it once to the canonical FC04 / PDU 0..12 map.
 */
static void migrate_existing_lt_endpoints(void)
{
    uint8_t count = 0;
    if (app_config_load_endpoint_count(&count) != ESP_OK) {
        return;
    }

    for (uint8_t ep = 1; ep <= count; ep++) {
        endpoint_config_t c = {0};
        if (app_config_load_endpoint(ep, &c) == ESP_OK) {
            if (c.device_type == DEVICE_LT_PANEL && c.attribute_count == 12) {
                ESP_LOGI(TAG,
                         "LT EP%u current map: FC%u PDU0=%u PDU11=%u",
                         ep,
                         c.attributes[0].function,
                         c.attributes[0].address,
                         c.attributes[11].address);
            }
            app_config_free_endpoint(&c);
        }
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
            static const uint16_t legacy_lt_pdu[12] = {
                0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12
            };
            bool legacy_template = true;
            for (uint16_t i = 0; i < 12; ++i) {
                if (strcmp(c->attributes[i].name, legacy_lt_names[i]) != 0 ||
                    c->attributes[i].function != MB_FC04 ||
                    c->attributes[i].address != legacy_lt_pdu[i]) {
                    legacy_template = false;
                    break;
                }
            }
            if (legacy_template) {
                free(c->attributes);
                c->attributes = NULL;
                c->attribute_count = 0;
                c->enabled = true;
                c->device_type = DEVICE_LT_PANEL;
                c->cluster_id = 0xFC10;
                c->slave_id = 1;
                esp_err_t me = app_config_save_template(device_type, c);
                if (me != ESP_OK) {
                    ESP_LOGW(TAG, "Legacy LT template reset failed: %s",
                             esp_err_to_name(me));
                } else {
                    ESP_LOGW(TAG, "Removed legacy hard-coded LT Panel template; configure attributes in web UI");
                }
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
