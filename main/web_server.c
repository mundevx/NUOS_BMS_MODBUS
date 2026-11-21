#include "web_server.h"
#include "app_config.h"
#include "device_templates.h"
#include "modbus_master.h"
#include "nuos_uart_link.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_check.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG="WEB";
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

static esp_err_t send_json(httpd_req_t *r,cJSON *root){char *s=cJSON_PrintUnformatted(root);if(!s)return ESP_ERR_NO_MEM;httpd_resp_set_type(r,"application/json");esp_err_t e=httpd_resp_sendstr(r,s);free(s);cJSON_Delete(root);return e;}
static int body_json(httpd_req_t *r,cJSON **out){if(r->content_len<=0||r->content_len>1048576)return -1;char *b=calloc(1,r->content_len+1);if(!b)return -1;int got=httpd_req_recv(r,b,r->content_len);if(got!=r->content_len){free(b);return -1;}*out=cJSON_Parse(b);free(b);return *out?0:-1;}
static esp_err_t root_get(httpd_req_t *r){httpd_resp_set_type(r,"text/html; charset=utf-8");return httpd_resp_send(r,(const char*)index_html_start,index_html_end-index_html_start);}
static esp_err_t device_types_get(httpd_req_t *r){char b[512];device_templates_json(b,sizeof(b));httpd_resp_set_type(r,"application/json");return httpd_resp_sendstr(r,b);}
static esp_err_t serial_get(httpd_req_t *r){serial_config_t c;app_config_load_serial(&c);cJSON *o=cJSON_CreateObject();cJSON_AddNumberToObject(o,"baud_rate",c.baud_rate);cJSON_AddNumberToObject(o,"data_bits",c.data_bits);cJSON_AddNumberToObject(o,"stop_bits",c.stop_bits);cJSON_AddNumberToObject(o,"parity",c.parity);return send_json(r,o);}
static esp_err_t serial_post(httpd_req_t *r){cJSON *o;if(body_json(r,&o))return httpd_resp_send_err(r,400,"Invalid JSON");serial_config_t c;app_config_load_serial(&c);c.baud_rate=(uint32_t)cJSON_GetNumberValue(cJSON_GetObjectItem(o,"baud_rate"));c.data_bits=(uint8_t)cJSON_GetNumberValue(cJSON_GetObjectItem(o,"data_bits"));c.stop_bits=(uint8_t)cJSON_GetNumberValue(cJSON_GetObjectItem(o,"stop_bits"));c.parity=(uint8_t)cJSON_GetNumberValue(cJSON_GetObjectItem(o,"parity"));cJSON_Delete(o);if(c.baud_rate<300||c.baud_rate>1000000||c.data_bits<7||c.data_bits>8||(c.stop_bits!=1&&c.stop_bits!=2)||c.parity>2)return httpd_resp_send_err(r,400,"Invalid serial settings");if(app_config_save_serial(&c)!=ESP_OK||modbus_apply_serial_config()!=ESP_OK)return httpd_resp_send_err(r,500,"Could not apply serial settings");cJSON *ok=cJSON_CreateObject();cJSON_AddBoolToObject(ok,"ok",1);return send_json(r,ok);}
static uint32_t modbus_reference_from_pdu(uint8_t function, uint16_t address)
{
    switch (function) {
        case MB_FC03:
        case MB_FC06:
        case MB_FC16:
            return 40001u + address;
        case MB_FC04:
            return 30001u + address;
        case MB_FC02:
            return 10001u + address;
        case MB_FC01:
        case MB_FC05:
        case MB_FC15:
        default:
            return 1u + address;
    }
}

/* Convert the datasheet-style reference entered in the UI to a zero-based PDU address. */
static bool modbus_reference_to_pdu(uint8_t function, uint32_t reference, uint16_t *pdu)
{
    uint32_t base;
    uint32_t max_ref;

    if (pdu == NULL) return false;

    switch (function) {
        case MB_FC03:
        case MB_FC06:
        case MB_FC16:
            base = 40001u; max_ref = 49999u; break;
        case MB_FC04:
            base = 30001u; max_ref = 39999u; break;
        case MB_FC02:
            base = 10001u; max_ref = 19999u; break;
        case MB_FC01:
        case MB_FC05:
        case MB_FC15:
            base = 1u; max_ref = 9999u; break;
        default:
            return false;
    }

    if (reference < base || reference > max_ref) return false;
    uint32_t address = reference - base;
    if (address > 0xFFFFu) return false;
    *pdu = (uint16_t)address;
    return true;
}

static bool valid_modbus_function(uint8_t function)
{
    return function == MB_FC01 || function == MB_FC02 ||
           function == MB_FC03 || function == MB_FC04 ||
           function == MB_FC05 || function == MB_FC06 ||
           function == MB_FC15 || function == MB_FC16;
}

static void attr_to_json(cJSON *a, uint8_t device_type, const modbus_attribute_t *x)
{
    cJSON_AddBoolToObject(a, "enabled", x->enabled);
    cJSON_AddNumberToObject(a, "attribute_id", x->attribute_id);
    cJSON_AddNumberToObject(a, "function", x->function);
    /* address remains available for diagnostics/API compatibility; the UI edits reference. */
    cJSON_AddNumberToObject(a, "address", x->address);
    uint32_t reference;
    if (device_type == DEVICE_LT_PANEL && x->function == MB_FC04) {
        reference = 43001u + x->address;
    } else {
        reference = modbus_reference_from_pdu(x->function, x->address);
    }
    cJSON_AddNumberToObject(a, "reference", reference);
    cJSON_AddNumberToObject(a, "reg_count", x->reg_count);
    cJSON_AddNumberToObject(a, "data_type", x->data_type);
    cJSON_AddNumberToObject(a, "word_order", x->word_order);
    cJSON_AddNumberToObject(a, "packed_count", x->packed_count);
    cJSON_AddNumberToObject(a, "address2", x->address2);
    cJSON_AddNumberToObject(a, "address3", x->address3);
    cJSON_AddStringToObject(a, "name", x->name);
}
static esp_err_t endpoint_get(httpd_req_t *r)
{
    char q[32];
    uint8_t ep = 1;

    if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK) {
        char v[8];
        if (httpd_query_key_value(q, "ep", v, sizeof(v)) == ESP_OK) {
            ep = (uint8_t)atoi(v);
        }
    }

    if (ep == 0 || ep > APP_MAX_ENDPOINTS) {
        return httpd_resp_send_err(r, 400, "Endpoint must be 1..50");
    }

    endpoint_config_t *c = calloc(1, sizeof(*c));
    if (!c) {
        return httpd_resp_send_err(r, 500, "Out of memory loading endpoint");
    }

    if (app_config_load_endpoint(ep, c) != ESP_OK) {
        free(c);
        return httpd_resp_send_err(r, 500, "Could not load endpoint");
    }

    cJSON *o = cJSON_CreateObject();
    if (!o) {
        free(c);
        return httpd_resp_send_err(r, 500, "Out of memory");
    }

    cJSON_AddNumberToObject(o, "endpoint", ep);
    cJSON_AddBoolToObject(o, "enabled", c->enabled);
    cJSON_AddNumberToObject(o, "slave_id", c->slave_id);
    cJSON_AddNumberToObject(o, "device_type", c->device_type);
    cJSON_AddNumberToObject(o, "cluster_id", c->cluster_id);
    cJSON_AddStringToObject(o, "device_name", device_type_name(c->device_type));

    cJSON *arr = cJSON_AddArrayToObject(o, "attributes");
    if (!arr) {
        cJSON_Delete(o);
        free(c);
        return httpd_resp_send_err(r, 500, "Out of memory");
    }

    for (uint16_t i = 0; i < c->attribute_count; i++) {
        cJSON *a = cJSON_CreateObject();
        if (!a) {
            cJSON_Delete(o);
            free(c);
            return httpd_resp_send_err(r, 500, "Out of memory");
        }
        attr_to_json(a, c->device_type, &c->attributes[i]);
        cJSON_AddItemToArray(arr, a);
    }

    app_config_free_endpoint(c);
    free(c);
    return send_json(r, o);
}

static esp_err_t endpoints_get(httpd_req_t *r)
{
    cJSON *arr = cJSON_CreateArray();
    if (!arr) {
        return httpd_resp_send_err(r, 500, "Out of memory");
    }

    endpoint_config_t *c = calloc(1, sizeof(*c));
    if (!c) {
        cJSON_Delete(arr);
        return httpd_resp_send_err(r, 500, "Out of memory loading endpoints");
    }

    for (uint8_t ep = 1; ep <= APP_MAX_ENDPOINTS; ep++) {
        app_config_free_endpoint(c);
        if (app_config_load_endpoint(ep, c) != ESP_OK) {
            continue;
        }

        if (c->enabled || c->attribute_count) {
            cJSON *o = cJSON_CreateObject();
            if (!o) {
                free(c);
                cJSON_Delete(arr);
                return httpd_resp_send_err(r, 500, "Out of memory");
            }
            cJSON_AddNumberToObject(o, "endpoint", ep);
            cJSON_AddBoolToObject(o, "enabled", c->enabled);
            cJSON_AddNumberToObject(o, "device_type", c->device_type);
            cJSON_AddNumberToObject(o, "cluster_id", c->cluster_id);
            cJSON_AddStringToObject(o, "device_name", device_type_name(c->device_type));
            cJSON_AddNumberToObject(o, "attribute_count", c->attribute_count);
            cJSON_AddItemToArray(arr, o);
        }
    }

    app_config_free_endpoint(c);
    free(c);
    return send_json(r, arr);
}

static esp_err_t endpoint_post(httpd_req_t *r)
{
    cJSON *o;
    if (body_json(r, &o)) {
        return httpd_resp_send_err(r, 400, "Invalid JSON");
    }

    int ep = cJSON_GetNumberValue(cJSON_GetObjectItem(o, "endpoint"));
    if (ep < 1 || ep > APP_MAX_ENDPOINTS) {
        cJSON_Delete(o);
        return httpd_resp_send_err(r, 400, "Endpoint must be 1..50");
    }

    endpoint_config_t *c = calloc(1, sizeof(*c));
    if (!c) {
        cJSON_Delete(o);
        return httpd_resp_send_err(r, 500, "Out of memory");
    }

    c->enabled = cJSON_IsTrue(cJSON_GetObjectItem(o, "enabled"));
    c->slave_id = (uint8_t)ep;
    c->device_type = (uint8_t)cJSON_GetNumberValue(cJSON_GetObjectItem(o, "device_type"));
    cJSON *cluster = cJSON_GetObjectItem(o, "cluster_id");
    c->cluster_id = cluster ? (uint16_t)cJSON_GetNumberValue(cluster) : 0;

    cJSON *arr = cJSON_GetObjectItem(o, "attributes");
    if (!cJSON_IsArray(arr)) {
        app_config_free_endpoint(c); free(c);
        cJSON_Delete(o);
        return httpd_resp_send_err(r, 400, "attributes must be an array");
    }

    int n = cJSON_GetArraySize(arr);
    if (n < 0 || n > APP_MAX_ATTRIBUTES) {
        app_config_free_endpoint(c); free(c); cJSON_Delete(o);
        return httpd_resp_send_err(r, 400, "Attribute count must be 0..64 per endpoint");
    }
    c->attribute_count = (uint16_t)n;
    bool seen_ids[256] = {0};
    if (n) {
        c->attributes = calloc((size_t)n, sizeof(modbus_attribute_t));
        if (!c->attributes) { app_config_free_endpoint(c); free(c); cJSON_Delete(o); return httpd_resp_send_err(r, 500, "Out of memory for attributes"); }
    }

    for (int i = 0; i < n; i++) {
        cJSON *a = cJSON_GetArrayItem(arr, i);
        modbus_attribute_t *x = &c->attributes[i];

        x->enabled = !cJSON_HasObjectItem(a, "enabled") ||
                     cJSON_IsTrue(cJSON_GetObjectItem(a, "enabled"));
        int aid_num = (int)cJSON_GetNumberValue(cJSON_GetObjectItem(a, "attribute_id"));
        if (aid_num < 0 || aid_num > 255 || seen_ids[aid_num]) {
            app_config_free_endpoint(c); free(c); cJSON_Delete(o);
            return httpd_resp_send_err(r, 400, "Attribute IDs must be unique values 0..255");
        }
        seen_ids[aid_num] = true;
        x->attribute_id = (uint8_t)aid_num;
        x->function = (uint8_t)cJSON_GetNumberValue(cJSON_GetObjectItem(a, "function"));
        x->address = (uint16_t)cJSON_GetNumberValue(cJSON_GetObjectItem(a, "address"));

        if (!valid_modbus_function(x->function)) {
            app_config_free_endpoint(c); free(c);
            cJSON_Delete(o);
            return httpd_resp_send_err(r, 400, "Unsupported Modbus function");
        }

        if (cJSON_HasObjectItem(a, "reference")) {
            uint32_t ref = (uint32_t)cJSON_GetNumberValue(cJSON_GetObjectItem(a, "reference"));
            bool ref_ok = false;

            /*
             * LT Panel keeps its operator-facing 43xxx reference namespace,
             * but the Modbus frame uses canonical raw PDU addresses 0..12.
             */
            if (c->device_type == DEVICE_LT_PANEL &&
                x->function == MB_FC04 &&
                ref >= 43001u && ref <= 43999u) {
                x->address = (uint16_t)(ref - 43001u);
                ref_ok = true;
            } else {
                ref_ok = modbus_reference_to_pdu(x->function, ref, &x->address);
            }

            if (!ref_ok) {
                app_config_free_endpoint(c); free(c);
                cJSON_Delete(o);
                return httpd_resp_send_err(r, 400, "Invalid Modbus reference for selected function");
            }
        }

        x->reg_count = (uint8_t)cJSON_GetNumberValue(cJSON_GetObjectItem(a, "reg_count"));
        x->data_type = (uint8_t)cJSON_GetNumberValue(cJSON_GetObjectItem(a, "data_type"));
        if (x->data_type > MB_TYPE_UINT48) {
            app_config_free_endpoint(c); free(c); cJSON_Delete(o);
            return httpd_resp_send_err(r, 400, "Unsupported data_type");
        }
        x->word_order = (uint8_t)cJSON_GetNumberValue(cJSON_GetObjectItem(a, "word_order"));
        x->packed_count = (uint8_t)cJSON_GetNumberValue(cJSON_GetObjectItem(a, "packed_count"));
        x->address2 = (uint16_t)cJSON_GetNumberValue(cJSON_GetObjectItem(a, "address2"));
        x->address3 = (uint16_t)cJSON_GetNumberValue(cJSON_GetObjectItem(a, "address3"));

        const char *name = cJSON_GetStringValue(cJSON_GetObjectItem(a, "name"));
        snprintf(x->name, sizeof(x->name), "%s", name ? name : "Attribute");

        if (x->reg_count == 0) {
            x->reg_count = 1;
        }
        if (x->reg_count > 125) {
            x->reg_count = 125;
        }
        if (x->packed_count > 3) {
            app_config_free_endpoint(c); free(c); cJSON_Delete(o);
            return httpd_resp_send_err(r, 400, "packed_count must be 0..3");
        }
    }

    if (c->enabled && c->attribute_count && c->cluster_id == 0) {
        app_config_free_endpoint(c); free(c); cJSON_Delete(o);
        return httpd_resp_send_err(r, 400, "cluster_id is required for an enabled Zigbee endpoint");
    }

    cJSON_Delete(o);

    esp_err_t save_err = app_config_save_endpoint((uint8_t)ep, c);

    /*
     * The web page is the authoritative LT Panel Modbus-map editor.
     * Persist the exact attributes submitted by the browser as the LT Panel
     * device template too, so "Load Template" and endpoint provisioning use
     * the user's configured map instead of a compiled-in register list.
     */
    if (save_err == ESP_OK && c->device_type == DEVICE_LT_PANEL) {
        esp_err_t template_err = app_config_save_template(DEVICE_LT_PANEL, c);
        if (template_err != ESP_OK) {
            ESP_LOGE(TAG, "LT Panel template save failed: %s",
                     esp_err_to_name(template_err));
            save_err = template_err;
        }
    }

    app_config_free_endpoint(c);
    free(c);

    if (save_err != ESP_OK) {
        return httpd_resp_send_err(r, 500, "NVS save failed");
    }

    /* Push the saved Zigbee model to H2 immediately. The UART sync task
     * recalculates the latest hash, so rapid consecutive edits are coalesced
     * without waiting for the periodic reconciliation timer. */
    nuos_link_request_zigbee_config_sync();

    cJSON *ok = cJSON_CreateObject();
    cJSON_AddBoolToObject(ok, "ok", 1);
    cJSON_AddBoolToObject(ok, "config_sync_requested", 1);
    return send_json(r, ok);
}

static esp_err_t endpoint_delete(httpd_req_t *r){char q[32],v[8];if(httpd_req_get_url_query_str(r,q,sizeof(q))!=ESP_OK||httpd_query_key_value(q,"ep",v,sizeof(v))!=ESP_OK)return httpd_resp_send_err(r,400,"Missing ep");int ep=atoi(v);if(ep<1||ep>50)return httpd_resp_send_err(r,400,"Endpoint must be 1..50");if(app_config_delete_endpoint(ep)!=ESP_OK)return httpd_resp_send_err(r,500,"Delete failed");
    /* Keep the UI's stored endpoint count aligned with the actual highest
     * configured endpoint after a manual delete. The Zigbee hash itself uses
     * the exact active endpoint count and is the authoritative H2 value. */
    uint32_t hash=0; uint8_t active=0;
    if(app_config_calculate_zigbee_hash(&hash,&active)==ESP_OK && active>0) (void)app_config_save_endpoint_count(active);
    nuos_link_request_zigbee_config_sync();cJSON *ok=cJSON_CreateObject();cJSON_AddBoolToObject(ok,"ok",1);cJSON_AddBoolToObject(ok,"config_sync_requested",1);return send_json(r,ok);}
static esp_err_t template_get(httpd_req_t *r)
{
    char q[64],v[8];
    if(httpd_req_get_url_query_str(r,q,sizeof(q))!=ESP_OK ||
       httpd_query_key_value(q,"device_type",v,sizeof(v))!=ESP_OK)
        return httpd_resp_send_err(r,400,"Missing device_type");
    int dt=atoi(v);
    if(dt<0 || dt>=DEVICE_COUNT) return httpd_resp_send_err(r,400,"Invalid device_type");

    endpoint_config_t c={0};
    esp_err_t e=app_config_load_template((uint8_t)dt,&c);
    if(e==ESP_ERR_NVS_NOT_FOUND){
        device_template_load((uint8_t)dt,1,&c);
        c.slave_id=1; c.device_type=(uint8_t)dt;
        e=app_config_save_template((uint8_t)dt,&c);
    }
    if(e!=ESP_OK){ app_config_free_endpoint(&c); return httpd_resp_send_err(r,500,"Template load failed"); }

    cJSON *o=cJSON_CreateObject();
    cJSON_AddNumberToObject(o,"device_type",dt);
    cJSON_AddNumberToObject(o,"cluster_id",c.cluster_id);
    cJSON_AddStringToObject(o,"device_name",device_type_name((uint8_t)dt));
    cJSON *arr=cJSON_AddArrayToObject(o,"attributes");
    for(uint16_t i=0;i<c.attribute_count;i++){ cJSON *a=cJSON_CreateObject(); attr_to_json(a,c.device_type,&c.attributes[i]); cJSON_AddItemToArray(arr,a); }
    app_config_free_endpoint(&c);
    return send_json(r,o);
}

static esp_err_t endpoints_provision_post(httpd_req_t *r)
{
    cJSON *o; if(body_json(r,&o)) return httpd_resp_send_err(r,400,"Invalid JSON");
    int count=(int)cJSON_GetNumberValue(cJSON_GetObjectItem(o,"count"));
    int dt=(int)cJSON_GetNumberValue(cJSON_GetObjectItem(o,"device_type"));
    cJSON_Delete(o);
    if(count<1 || count>APP_MAX_ENDPOINTS || dt<0 || dt>=DEVICE_COUNT)
        return httpd_resp_send_err(r,400,"Invalid endpoint count/device type");

    /* Endpoint Count is intentionally destructive. The web UI confirms
     * before this API is called. Recreate exactly the requested count. */
    for(int ep=1;ep<=APP_MAX_ENDPOINTS;ep++){
        esp_err_t de=app_config_delete_endpoint((uint8_t)ep);
        if(de!=ESP_OK && de!=ESP_ERR_NVS_NOT_FOUND)
            return httpd_resp_send_err(r,500,"Failed to clear existing endpoint data");
    }

    for(int ep=1;ep<=count;ep++){
        endpoint_config_t tpl={0};
        esp_err_t template_load_err = app_config_load_template((uint8_t)dt, &tpl);
        if (template_load_err != ESP_OK) {
            (void)device_template_load((uint8_t)dt, (uint8_t)ep, &tpl);
        }
        if(tpl.attribute_count > 0 && !tpl.attributes){
            app_config_free_endpoint(&tpl);
            return httpd_resp_send_err(r,500,"Template allocation failed during provisioning");
        }
        tpl.enabled=true; tpl.slave_id=(uint8_t)ep; tpl.device_type=(uint8_t)dt;
        esp_err_t e=app_config_save_endpoint((uint8_t)ep,&tpl);
        app_config_free_endpoint(&tpl);
        if(e!=ESP_OK) return httpd_resp_send_err(r,500,"Endpoint save failed during provisioning");
    }

    esp_err_t ce=app_config_save_endpoint_count((uint8_t)count);
    if(ce!=ESP_OK) return httpd_resp_send_err(r,500,"Endpoint count save failed");

    nuos_link_request_zigbee_config_sync();
    cJSON *ok=cJSON_CreateObject(); cJSON_AddBoolToObject(ok,"ok",1); cJSON_AddNumberToObject(ok,"count",count); cJSON_AddBoolToObject(ok,"config_sync_requested",1); return send_json(r,ok);
}

static esp_err_t template_post(httpd_req_t *r)
{
    cJSON *o; if(body_json(r,&o)) return httpd_resp_send_err(r,400,"Invalid JSON");
    int ep=cJSON_GetNumberValue(cJSON_GetObjectItem(o,"endpoint"));
    int dt=cJSON_GetNumberValue(cJSON_GetObjectItem(o,"device_type"));
    cJSON_Delete(o);
    if(ep<1 || ep>APP_MAX_ENDPOINTS || dt<0 || dt>=DEVICE_COUNT)
        return httpd_resp_send_err(r,400,"Invalid endpoint/template");

    endpoint_config_t *c=calloc(1,sizeof(*c)); if(!c) return httpd_resp_send_err(r,500,"Out of memory");

    /*
     * Prefer the device template saved from the web editor.  In particular,
     * LT Panel must never be reconstructed from a firmware register map.
     */
    esp_err_t err = app_config_load_template((uint8_t)dt, c);
    if (err != ESP_OK) {
        memset(c, 0, sizeof(*c));
        (void)device_template_load((uint8_t)dt, (uint8_t)ep, c);
        err = c->attributes ? ESP_OK : ESP_ERR_NO_MEM;
    }
    c->slave_id=(uint8_t)ep; c->device_type=(uint8_t)dt;
    if(err==ESP_OK) err=app_config_save_endpoint((uint8_t)ep,c);
    app_config_free_endpoint(c); free(c);
    if(err!=ESP_OK) return httpd_resp_send_err(r,500,"Template save failed");
    nuos_link_request_zigbee_config_sync();
    return endpoint_get(r);
}
static esp_err_t write_post(httpd_req_t *r){cJSON *o;if(body_json(r,&o))return httpd_resp_send_err(r,400,"Invalid JSON");int ep=cJSON_GetNumberValue(cJSON_GetObjectItem(o,"endpoint"));int attr=cJSON_GetNumberValue(cJSON_GetObjectItem(o,"attribute_id"));const char *v=cJSON_GetStringValue(cJSON_GetObjectItem(o,"value"));esp_err_t e=(v&&ep>0&&ep<=50&&attr>=0)?modbus_write_attribute(ep,(uint8_t)attr,v):ESP_ERR_INVALID_ARG;cJSON_Delete(o);if(e!=ESP_OK)return httpd_resp_send_err(r,400,"Write failed or attribute is not a writable FC05/FC06");cJSON *ok=cJSON_CreateObject();cJSON_AddBoolToObject(ok,"ok",1);return send_json(r,ok);}
static esp_err_t status_get(httpd_req_t *r)
{
    cJSON *o=cJSON_CreateObject();
    if(!o) return httpd_resp_send_err(r,500,"Out of memory");

    uint8_t endpoint_count=1;
    uint8_t saved_device_type=DEVICE_CUSTOM;
    uint16_t saved_cluster_id=0;

    (void)app_config_load_endpoint_count(&endpoint_count);

    /* Report the persisted module selection as part of startup status.
     * For a provisioned device set (for example 25 LT Panel slaves), EP1
     * is representative of the selected module and avoids forcing the UI
     * back to Custom after a C6 reboot. */
    endpoint_config_t *startup_cfg=calloc(1,sizeof(*startup_cfg));
    if(startup_cfg){
        if(app_config_load_endpoint(1,startup_cfg)==ESP_OK && startup_cfg->enabled){
            saved_device_type=startup_cfg->device_type;
            saved_cluster_id=startup_cfg->cluster_id;
        }
        app_config_free_endpoint(startup_cfg);
        free(startup_cfg);
    }

    cJSON_AddStringToObject(o,"device","ESP32-C6");
    cJSON_AddStringToObject(o,"wifi","NUOS-BMS AP");
    cJSON_AddBoolToObject(o,"h2_joined",nuos_link_peer_joined());
    cJSON_AddNumberToObject(o,"h2_silent_ms",nuos_link_peer_silent_ms());
    cJSON_AddNumberToObject(o,"endpoint_count",endpoint_count);
    cJSON_AddNumberToObject(o,"device_type",saved_device_type);
    cJSON_AddNumberToObject(o,"cluster_id",saved_cluster_id);
    return send_json(r,o);
}
static esp_err_t modbus_status_get(httpd_req_t *r)
{
    cJSON *o=cJSON_CreateObject();
    if(!o) return httpd_resp_send_err(r,500,"Out of memory");
    cJSON *arr=cJSON_AddArrayToObject(o,"endpoints");
    if(!arr){cJSON_Delete(o);return httpd_resp_send_err(r,500,"Out of memory");}
    esp_err_t e=modbus_status_to_json(arr);
    if(e!=ESP_OK){cJSON_Delete(o);return httpd_resp_send_err(r,500,"Unable to read Modbus status");}
    return send_json(r,o);
}

static esp_err_t wifi_init_ap(void){ESP_RETURN_ON_ERROR(esp_netif_init(),TAG,"netif");ESP_RETURN_ON_ERROR(esp_event_loop_create_default(),TAG,"event loop");esp_netif_create_default_wifi_ap();wifi_init_config_t w=WIFI_INIT_CONFIG_DEFAULT();ESP_RETURN_ON_ERROR(esp_wifi_init(&w),TAG,"wifi init");wifi_config_t c={};snprintf((char*)c.ap.ssid,sizeof(c.ap.ssid),"NUOS-BMS");snprintf((char*)c.ap.password,sizeof(c.ap.password),"nuos1234");c.ap.ssid_len=strlen((char*)c.ap.ssid);c.ap.channel=1;c.ap.max_connection=4;c.ap.authmode=WIFI_AUTH_WPA2_PSK;ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP),TAG,"mode");ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP,&c),TAG,"config");return esp_wifi_start();}

esp_err_t web_server_start(void){ESP_RETURN_ON_ERROR(wifi_init_ap(),TAG,"AP");httpd_config_t c=HTTPD_DEFAULT_CONFIG();c.max_uri_handlers=20;c.stack_size=8192;c.recv_wait_timeout=10;c.send_wait_timeout=10;httpd_handle_t s=NULL;ESP_RETURN_ON_ERROR(httpd_start(&s,&c),TAG,"httpd");httpd_uri_t u={};u.method=HTTP_GET;u.uri="/";u.handler=root_get;httpd_register_uri_handler(s,&u);
u.uri="/api/device-types";u.handler=device_types_get;httpd_register_uri_handler(s,&u);u.uri="/api/serial";u.handler=serial_get;httpd_register_uri_handler(s,&u);u.method=HTTP_POST;u.handler=serial_post;httpd_register_uri_handler(s,&u);
u.method=HTTP_GET;u.uri="/api/endpoints";u.handler=endpoints_get;httpd_register_uri_handler(s,&u);u.uri="/api/endpoint";u.handler=endpoint_get;httpd_register_uri_handler(s,&u);u.method=HTTP_POST;u.handler=endpoint_post;httpd_register_uri_handler(s,&u);u.method=HTTP_DELETE;u.handler=endpoint_delete;httpd_register_uri_handler(s,&u);
u.method=HTTP_GET;u.uri="/api/template";u.handler=template_get;httpd_register_uri_handler(s,&u);u.method=HTTP_POST;u.uri="/api/template";u.handler=template_post;httpd_register_uri_handler(s,&u);u.uri="/api/endpoints/provision";u.handler=endpoints_provision_post;httpd_register_uri_handler(s,&u);u.uri="/api/write";u.handler=write_post;httpd_register_uri_handler(s,&u);u.method=HTTP_GET;u.uri="/api/status";u.handler=status_get;httpd_register_uri_handler(s,&u);u.uri="/api/modbus-status";u.handler=modbus_status_get;httpd_register_uri_handler(s,&u);
ESP_LOGI(TAG,"Web UI: http://192.168.4.1 (AP SSID NUOS-BMS password nuos1234)");return ESP_OK;}
