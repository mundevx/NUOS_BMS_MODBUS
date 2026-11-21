NUOS C6 LINKER FIX

This fixes the linker failure where these symbols were undefined:
- app_config_init
- app_config_load_endpoint
- app_config_save_endpoint
- app_config_delete_endpoint
- app_config_load_serial
- app_config_save_serial
- app_config_calculate_zigbee_hash
- device_type_name

Root cause:
The C6 build was compiling main.c, modbus_master.c, nuos_uart_link.c and
web_server.c, but app_config.c and device_templates.c were missing from the
main component source list.

This package therefore includes the actual C6 app_config.c and
device_templates.c AND a CMakeLists.txt that explicitly compiles both.

IMPORTANT:
- This is ESP32-C6 NUOS BMS only.
- nuos_uart_link.c is the C6 UART implementation.
- Do NOT replace it with the H2 Zigbee nuos_uart_link.c.
- nuos_link_proto.h is the shared C6<->H2 protocol header.
- The packed DG UINT48 configuration is retained.

Replace the C6 main component files with this package, then:
  idf.py fullclean
  idf.py build

If your existing project has additional component dependencies in its
CMakeLists.txt, preserve them while adding app_config.c and
device_templates.c to SRCS.


Endpoint/attribute model (all device templates):
- Endpoint N is Modbus slave ID N.
- A device template defines the complete register/attribute set for one device type.
- Loading that template for any endpoint uses the same register set; the endpoint number changes only the slave ID.
- No template uses endpoint number to select a subset of attributes.
- The old UPS 16/12 split is migrated to the complete 28-attribute UPS set.
- The old Lift one-register-per-endpoint mapping is migrated to the complete 7-register Lift set.
- C6 attribute storage uses dynamically allocated arrays with a uint16_t attribute_count; there is no compile-time 16-attribute ceiling.
