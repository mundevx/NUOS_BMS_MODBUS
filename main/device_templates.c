#include "device_templates.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * NUOS C6 dashboard-aligned device templates
 *
 * Rules:
 *  - Endpoint == Modbus slave ID == Zigbee endpoint.
 *  - Attribute IDs are sequential and stable within each template.
 *  - LT Panel register maps are stored as station-specific firmware profiles
 *    and can be selected/applied from the web UI.
 *  - Other supplied device register maps retain their existing definitions.
 *  - 3xxxx -> FC04, 4xxxx -> FC03.
 */

#ifndef MB_TYPE_UINT8
#define MB_TYPE_UINT8 9
#endif

static uint16_t pdu(uint32_t ref)
{
    if (ref >= 40001 && ref <= 49999) return (uint16_t)(ref - 40001);
    if (ref >= 30001 && ref <= 39999) return (uint16_t)(ref - 30001);
    if (ref >= 10001 && ref <= 19999) return (uint16_t)(ref - 10001);
    if (ref >= 1 && ref <= 9999) return (uint16_t)(ref - 1);
    return 0;
}
static uint16_t cluster_for_endpoint(uint8_t dev, uint8_t ep)
{
    switch (dev) {
        case DEVICE_LIGHTING:          return 0xFC00;
        case DEVICE_LT_PANEL:          return 0xFC10;
        case DEVICE_DG:                return 0xFC20;
        case DEVICE_UPS:               return 0xFC30;
        case DEVICE_LIFT:              return 0xFC40;
        case DEVICE_ESCALATOR:         return 0xFC50;
        case DEVICE_FIRE_HYDRANT:      return 0xFC60;
        case DEVICE_ANEMOMETER:        return 0xFC70;
        case DEVICE_AC:                return 0xFC80;
        case DEVICE_VENTILATION_FAN:   return 0xFC90;
        case DEVICE_SLD:               return 0xFCB0;
        case DEVICE_CUSTOM:            return 0xFCE0;
        default:                       return 0xFCE0;
    }
}

static void base(endpoint_config_t *c, uint8_t ep, uint8_t dev)
{
    memset(c, 0, sizeof(*c));
    c->enabled = true;
    c->slave_id = ep;
    c->device_type = dev;
    c->cluster_id = cluster_for_endpoint(dev, ep);
    c->attributes = calloc(APP_MAX_ATTRIBUTES, sizeof(*c->attributes));
}

static void add_ref(endpoint_config_t *c, const char *name, uint8_t fn,
                    uint32_t ref, uint8_t regs, uint8_t type)
{
    if (!c || c->attribute_count >= APP_MAX_ATTRIBUTES) return;

    modbus_attribute_t *a = &c->attributes[c->attribute_count];
    memset(a, 0, sizeof(*a));

    a->enabled = true;
    a->attribute_id = c->attribute_count;
    a->function = fn;
    a->address = pdu(ref);
    a->reg_count = regs;
    a->data_type = type;
    a->word_order = MB_WORD_BIG_FIRST;
    snprintf(a->name, sizeof(a->name), "%s", name);

    c->attribute_count++;
}

static void add_pdu(endpoint_config_t *c, const char *name, uint8_t fn,
                    uint16_t address, uint8_t regs, uint8_t type)
{
    if (!c || c->attribute_count >= APP_MAX_ATTRIBUTES) return;

    modbus_attribute_t *a = &c->attributes[c->attribute_count];
    memset(a, 0, sizeof(*a));

    a->enabled = true;
    a->attribute_id = c->attribute_count;
    a->function = fn;
    a->address = address;
    a->reg_count = regs;
    a->data_type = type;
    a->word_order = MB_WORD_BIG_FIRST;
    snprintf(a->name, sizeof(a->name), "%s", name);

    c->attribute_count++;
}
/* ----------------------------- LT PANEL -------------------------------- */
/*
 * Station register maps supplied for the LT Panel / L&T WL4405 family.
 *
 * The numeric addresses below are the raw zero-based Modbus PDU addresses
 * from V_moduleDeviceModbusAddress. Each UINT16 measurement occupies two
 * consecutive Modbus input-register addresses. The low 8 bits returned at
 * address N are the high data byte; the low 8 bits at N+1 are the low data
 * byte. Phase measurements start every two addresses.
 *
 * DEVICE_TYPE_BMS_LTPANEL_PhaseId is code 80 in the supplied V_moduleDeviceType
 * list. It is retained as profile metadata, not polled as a Modbus register:
 * it is outside the configured read window for Stations 01, 09, 10 and 11.
 */
typedef struct {
    lt_panel_profile_info_t info;
    uint16_t voltage[3];
    uint16_t current[3];
    uint16_t frequency;
    uint16_t power_consumption;
    uint16_t power_factor[3];
    uint8_t power_factor_phase_count;
} lt_panel_profile_map_t;

static const lt_panel_profile_map_t s_lt_panel_profiles[] = {
    {
        .info = { LT_PANEL_STATION_01, "Station_01 (Stations 01-06 / Depot)",
                  28, 0, 112, 56, 19200, false, 80 },
        .voltage = {143, 145, 147},
        .current = {151, 153, 155},
        .frequency = 157,
        .power_consumption = 159,
        .power_factor = {117, 119, 121},
        .power_factor_phase_count = 3
    },
    {
        .info = { LT_PANEL_STATION_07, "Station_07 (Stations 07-08)",
                  28, 0, 0, 112, 0, false, 80 },
        .voltage = {59, 81, 103},
        .current = {63, 85, 107},
        .frequency = 41,
        .power_consumption = 9,
        .power_factor = {65, 87, 109},
        .power_factor_phase_count = 3
    },
    {
        .info = { LT_PANEL_STATION_09, "Station_09", 12, 6, 201, 36,
                  19200, true, 80 },
        .voltage = {219, 221, 223},
        .current = {225, 227, 229},
        .frequency = 211,
        .power_consumption = 209,
        .power_factor = {201, 0, 0},
        .power_factor_phase_count = 1
    },
    {
        .info = { LT_PANEL_STATION_10, "Station_10", 2, 0, 3029, 136,
                  0, false, 80 },
        .voltage = {3030, 3032, 3034},
        .current = {3036, 3038, 3040},
        .frequency = 3132,
        .power_consumption = 3126,
        .power_factor = {3162, 0, 0},
        .power_factor_phase_count = 1
    },
    {
        .info = { LT_PANEL_STATION_11, "Station_11", 28, 0, 2996, 112,
                  0, false, 80 },
        .voltage = {3040, 3070, 3100},
        .current = {3042, 3072, 3102},
        .frequency = 3036,
        .power_consumption = 3030,
        .power_factor = {3006, 0, 0},
        .power_factor_phase_count = 1
    }
};

static const lt_panel_profile_map_t *lt_panel_find_profile(uint8_t station_id)
{
    for (size_t i = 0; i < sizeof(s_lt_panel_profiles) / sizeof(s_lt_panel_profiles[0]); i++) {
        if (s_lt_panel_profiles[i].info.station_id == station_id) {
            return &s_lt_panel_profiles[i];
        }
    }
    return NULL;
}

esp_err_t device_template_get_lt_panel_profile_info(uint8_t station_id,
                                                      lt_panel_profile_info_t *info)
{
    if (!info) return ESP_ERR_INVALID_ARG;
    const lt_panel_profile_map_t *profile = lt_panel_find_profile(station_id);
    if (!profile) return ESP_ERR_INVALID_ARG;
    *info = profile->info;
    return ESP_OK;
}

esp_err_t device_template_load_lt_panel_profile(uint8_t station_id, uint8_t endpoint,
                                                endpoint_config_t *cfg,
                                                lt_panel_profile_info_t *info)
{
    if (!cfg || endpoint == 0 || endpoint > APP_MAX_ENDPOINTS) {
        return ESP_ERR_INVALID_ARG;
    }

    const lt_panel_profile_map_t *profile = lt_panel_find_profile(station_id);
    if (!profile) return ESP_ERR_INVALID_ARG;

    base(cfg, endpoint, DEVICE_LT_PANEL);
    if (!cfg->attributes) return ESP_ERR_NO_MEM;

    if (info) *info = profile->info;

    static const char *const phase_names[] = {"R", "Y", "B"};
    for (uint8_t i = 0; i < 3; i++) {
        char name[APP_NAME_LEN];
        snprintf(name, sizeof(name), "Voltage %s", phase_names[i]);
        /* Each 16-bit LT value is exposed as two byte-addressed Modbus
         * locations: first address = high byte, next address = low byte. */
        add_pdu(cfg, name, MB_FC04, profile->voltage[i], 2, MB_TYPE_UINT16);
    }
    for (uint8_t i = 0; i < 3; i++) {
        char name[APP_NAME_LEN];
        snprintf(name, sizeof(name), "Current %s", phase_names[i]);
        add_pdu(cfg, name, MB_FC04, profile->current[i], 2, MB_TYPE_UINT16);
    }

    add_pdu(cfg, "Frequency", MB_FC04, profile->frequency, 2, MB_TYPE_UINT16);
    add_pdu(cfg, "Power Consumption", MB_FC04, profile->power_consumption,
            2, MB_TYPE_UINT32);

    if (profile->power_factor_phase_count == 3) {
        for (uint8_t i = 0; i < 3; i++) {
            char name[APP_NAME_LEN];
            snprintf(name, sizeof(name), "Power Factor %s", phase_names[i]);
            add_pdu(cfg, name, MB_FC04, profile->power_factor[i], 2, MB_TYPE_UINT16);
        }
    } else {
        add_pdu(cfg, "Power Factor", MB_FC04, profile->power_factor[0],
                2, MB_TYPE_UINT16);
    }

    return ESP_OK;
}

static void lt_panel(endpoint_config_t *c, uint8_t ep)
{
    /* Station_01 is the initial/default profile; the web UI can select another. */
    (void)device_template_load_lt_panel_profile(LT_PANEL_STATION_01, ep, c, NULL);
}

/* ----------------------------- DG --------------------------------------- */
/*
 * Dashboard has separate R/Y/B properties, so do NOT pack them into one
 * UINT48 attribute. The supplied DG mapping contains 46 individual
 * parameters. APP_MAX_ATTRIBUTES=64 supports the complete set.
 */
static void dg(endpoint_config_t *c, uint8_t ep)
{
    base(c, ep, DEVICE_DG);

    /* Main / Mains */
    add_pdu(c, "Main Voltage L-L R",       3, 1068, 2, MB_TYPE_UINT16);
    add_pdu(c, "Main Voltage L-L Y",       3, 1070, 2, MB_TYPE_UINT16);
    add_pdu(c, "Main Voltage L-L B",       3, 1072, 2, MB_TYPE_UINT16);

    add_pdu(c, "Main Voltage L-N R",       3, 1062, 2, MB_TYPE_UINT16);
    add_pdu(c, "Main Voltage L-N Y",       3, 1064, 2, MB_TYPE_UINT16);
    add_pdu(c, "Main Voltage L-N B",       3, 1066, 2, MB_TYPE_UINT16);

    add_pdu(c, "Main Current R",            3, 1078, 2, MB_TYPE_UINT16);
    add_pdu(c, "Main Current Y",            3, 1080, 2, MB_TYPE_UINT16);
    add_pdu(c, "Main Current B",            3, 1082, 2, MB_TYPE_UINT16);
    add_pdu(c, "Main Frequency",            3, 1060, 1, MB_TYPE_UINT16);
    add_pdu(c, "Main Earth Current",        3, 1084, 2, MB_TYPE_UINT16);

    /* Generator */
    add_pdu(c, "Generator Voltage L-L R",   3, 1040, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Voltage L-L Y",   3, 1042, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Voltage L-L B",   3, 1044, 2, MB_TYPE_UINT16);

    add_pdu(c, "Generator Frequency",       3, 1032, 1, MB_TYPE_UINT16);

    add_pdu(c, "Generator Voltage L-N R",   3, 1034, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Voltage L-N Y",   3, 1036, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Voltage L-N B",   3, 1038, 2, MB_TYPE_UINT16);

    add_pdu(c, "Generator Current R",        3, 1046, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Current Y",        3, 1048, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Current B",        3, 1050, 2, MB_TYPE_UINT16);
    
    add_pdu(c, "Generator Earth Current",    3, 1052, 2, MB_TYPE_UINT16);

    /* Load / power */
    add_pdu(c, "Main Load Watts R",          3, 1085, 2, MB_TYPE_UINT16);
    add_pdu(c, "Main Load Watts Y",          3, 1087, 2, MB_TYPE_UINT16);
    add_pdu(c, "Main Load Watts B",          3, 1089, 2, MB_TYPE_UINT16);

    add_pdu(c, "Generator Load Watts R",    3, 1054, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Load Watts Y",    3, 1056, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Load Watts B",    3, 1058, 2, MB_TYPE_UINT16);

    add_pdu(c, "Generator Power R",          3, 1555, 1, MB_TYPE_UINT8);
    add_pdu(c, "Generator Power Y",          3, 1556, 1, MB_TYPE_UINT8);
    add_pdu(c, "Generator Power B",          3, 1557, 1, MB_TYPE_UINT8);

    add_pdu(c, "Generator Var R",            3, 1547, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Var Y",            3, 1549, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Var B",            3, 1551, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Total Var",        3, 1553, 2, MB_TYPE_UINT16);

    /* Energy */
    add_pdu(c, "Generator Positive kWh",     3, 1801, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Positive kVAh",    3, 1805, 2, MB_TYPE_UINT16);
    add_pdu(c, "Generator Positive kVArh",  3, 1807, 2, MB_TYPE_UINT16);

    /* Engine */
    add_pdu(c, "Engine Speed",               3, 1031, 1, MB_TYPE_UINT16);
    add_pdu(c, "Charge Alternator Voltage",  3, 1029, 1, MB_TYPE_UINT16);
    add_pdu(c, "Engine Battery Voltage",     3, 1030, 1, MB_TYPE_UINT16);
    add_pdu(c, "Oil Pressure",               3, 1025, 1, MB_TYPE_UINT8);
    add_pdu(c, "Coolant Temperature",        3, 1026, 1, MB_TYPE_UINT8);
    add_pdu(c, "Engine Run Hours",           3, 1798, 2, MB_TYPE_UINT16);
    add_pdu(c, "Fuel Level",                 3, 1028, 1, MB_TYPE_UINT8);
    add_pdu(c, "DG Status",                  3, 1408, 1, MB_TYPE_UINT8);
}

/* ----------------------------- UPS -------------------------------------- */
/*
 * UPS has two distinct endpoint models in the NUOS BMS configuration:
 *   EP1 -> normal UPS measurements/status (cluster FC30)
 *   EP2 -> UPS fault/error information (cluster FC31)
 *
 * The supplied source material does not contain the Modbus register references
 * for the EP2 fault/error fields. Those fields are therefore created disabled
 * with reference 0, so the user can enter the real datasheet references in the
 * web UI without the firmware ever polling invented addresses.
 */
static void add_unmapped_disabled(endpoint_config_t *c, const char *name, uint8_t type)
{
    if (!c || !c->attributes || c->attribute_count >= APP_MAX_ATTRIBUTES) return;
    modbus_attribute_t *a = &c->attributes[c->attribute_count];
    memset(a, 0, sizeof(*a));
    a->enabled = false;
    a->attribute_id = c->attribute_count;
    a->function = 4;
    a->address = 0;
    a->reg_count = 1;
    a->data_type = type;
    a->word_order = MB_WORD_BIG_FIRST;
    snprintf(a->name, sizeof(a->name), "%s", name);
    c->attribute_count++;
}

static void ups_fault_error(endpoint_config_t *c, uint8_t ep)
{
    base(c, ep, DEVICE_UPS);
    add_unmapped_disabled(c, "Fault Status", MB_TYPE_UINT16);
    add_unmapped_disabled(c, "Fault Code", MB_TYPE_UINT16);
    add_unmapped_disabled(c, "Error Status", MB_TYPE_UINT16);
    add_unmapped_disabled(c, "Error Code", MB_TYPE_UINT16);
}

static void ups(endpoint_config_t *c, uint8_t ep)
{
    base(c, ep, DEVICE_UPS);

    add_ref(c, "Input Frequency",          4, 30004, 1, MB_TYPE_UINT16);
    add_ref(c, "Input Current A",          4, 30005, 1, MB_TYPE_UINT16);
    add_ref(c, "Input Current B",          4, 30006, 1, MB_TYPE_UINT16);
    add_ref(c, "Input Current C",          4, 30007, 1, MB_TYPE_UINT16);
    add_ref(c, "Input PF A",               4, 30008, 1, MB_TYPE_UINT16);
    add_ref(c, "Input PF B",               4, 30009, 1, MB_TYPE_UINT16);
    add_ref(c, "Input PF C",               4, 30010, 1, MB_TYPE_UINT16);

    add_ref(c, "Output Frequency",         4, 30014, 1, MB_TYPE_UINT16);
    add_ref(c, "Output Current A",         4, 30015, 1, MB_TYPE_UINT16);
    add_ref(c, "Output Current B",         4, 30016, 1, MB_TYPE_UINT16);
    add_ref(c, "Output Current C",         4, 30017, 1, MB_TYPE_UINT16);

    add_ref(c, "Output Power kW A",        4, 30018, 1, MB_TYPE_UINT16);
    add_ref(c, "Output Power kW B",        4, 30019, 1, MB_TYPE_UINT16);
    add_ref(c, "Output Power kW C",        4, 30020, 1, MB_TYPE_UINT16);

    add_ref(c, "Output Load % A",          4, 30021, 1, MB_TYPE_UINT16);
    add_ref(c, "Output Load % B",          4, 30022, 1, MB_TYPE_UINT16);
    add_ref(c, "Output Load % C",          4, 30023, 1, MB_TYPE_UINT16);
    //Comment

    add_ref(c, "Output Power kVA A",       4, 30047, 1, MB_TYPE_UINT16);
    add_ref(c, "Output Power kVA B",       4, 30048, 1, MB_TYPE_UINT16);
    add_ref(c, "Output Power kVA C",       4, 30049, 1, MB_TYPE_UINT16);

    add_ref(c, "DC Positive Voltage",      4, 30031, 1, MB_TYPE_UINT16);
    add_ref(c, "DC Negative Voltage",      4, 30032, 1, MB_TYPE_UINT16);
    add_ref(c, "Battery Temperature",      4, 30039, 1, MB_TYPE_UINT16);
    add_ref(c, "UPS System Status",        4, 30071, 1, MB_TYPE_UINT16);

    add_ref(c, "Parallel ID",              3, 40002, 1, MB_TYPE_UINT16);
    add_ref(c, "Number of Parallel UPS",   3, 40007, 1, MB_TYPE_UINT16);
    add_ref(c, "Output Dry Contact",       3, 40204, 1, MB_TYPE_UINT16);
    add_ref(c, "Input Dry Contact 1",      3, 40221, 1, MB_TYPE_UINT16);
}

/* ----------------------------- LIFT ------------------------------------- */
static void lift(endpoint_config_t *c, uint8_t ep)
{
    base(c, ep, DEVICE_LIFT);

    static const uint16_t addr[] = {
        30004, 30011, 30018, 30025, 30032, 30039, 30046
    };

    static const char *name[] = {
        "Lift Status Word 1",
        "Lift Status Word 2",
        "Lift Status Word 3",
        "Lift Status Word 4",
        "Lift Status Word 5",
        "Lift Status Word 6",
        "Lift Status Word 7"
    };

    for (uint8_t i = 0; i < 7; i++)
        add_ref(c, name[i], 4, addr[i], 1, MB_TYPE_UINT16);
}

/*
 * The following dashboard modules have a parameter list, but the supplied
 * C6 source does not contain a corresponding Modbus register map. Do not
 * create fake register addresses. The web endpoint editor remains the
 * authoritative source for these modules until their register maps are
 * supplied.
 */
static void dashboard_config_only(endpoint_config_t *c, uint8_t ep, uint8_t dev)
{
    base(c, ep, dev);
}

size_t device_template_load(uint8_t device_type, uint8_t endpoint,
                            endpoint_config_t *cfg)
{
    if (!cfg || endpoint == 0 || endpoint > APP_MAX_ENDPOINTS) return 0;

    switch (device_type) {
        case DEVICE_LT_PANEL:
            lt_panel(cfg, endpoint);
            break;

        case DEVICE_DG:
            dg(cfg, endpoint);
            break;

        case DEVICE_UPS:
            if (endpoint == 2) ups_fault_error(cfg, endpoint);
            else ups(cfg, endpoint);
            break;

        case DEVICE_LIFT:
            lift(cfg, endpoint);
            break;

        case DEVICE_ESCALATOR:
        case DEVICE_FIRE_HYDRANT:
        case DEVICE_ANEMOMETER:
        case DEVICE_AC:
        case DEVICE_VENTILATION_FAN:
        case DEVICE_SLD:
        case DEVICE_LIGHTING:
        case DEVICE_CUSTOM:
        default:
            dashboard_config_only(cfg, endpoint, device_type);
            break;
    }

    return cfg->attribute_count;
}

void device_templates_json(char *out, size_t n)
{
    snprintf(out, n,
        "["
        "\"LT Panel / Panel MFM\","
        "\"DG\","
        "\"UPS\","
        "\"Lift\","
        "\"Escalator\","
        "\"Fire Hydrant\","
        "\"Anemometer\","
        "\"AC\","
        "\"Ventilation Fan\","
        "\"Custom\","
        "\"SLD\","
        "\"Lighting\""
        "]");
}
