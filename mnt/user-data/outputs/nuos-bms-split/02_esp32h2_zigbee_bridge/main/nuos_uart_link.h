/*
 * NUOS BMS - ESP32-H2 side of the UART link (receiver)
 *
 * Receives one bulk frame per endpoint from the ESP32-C6 Modbus board and
 * pushes it into the Zigbee layer.
 */

#ifndef NUOS_UART_LINK_H_
#define NUOS_UART_LINK_H_

#include <stdint.h>
#include <stdbool.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "nuos_link_proto.h"

/*
 * Link UART on the H2.
 *
 * UART1 is used here so the H2 console stays on UART0 and remains easy to
 * monitor while the Zigbee stack is running. To make it UART0 <-> UART0 on
 * both boards, set NUOS_LINK_UART to UART_NUM_0, set the pins to 24/23 (the
 * H2 UART0 defaults) and move this console to USB-Serial-JTAG the same way
 * sdkconfig.defaults does on the C6.
 */
#define NUOS_LINK_UART              UART_NUM_1
#define NUOS_LINK_TX_PIN            10      /* -> C6 RX (GPIO17) */
#define NUOS_LINK_RX_PIN            11      /* <- C6 TX (GPIO16) */

/* STATUS heartbeat sent to the C6. Its watchdog window is 3 beats. */
#define NUOS_LINK_HEARTBEAT_MS      2000

/* Depth of the RX -> Zigbee work queue, in endpoints. */
#define NUOS_LINK_QUEUE_DEPTH       8

/*
 * Installs the UART driver and starts the RX, worker and heartbeat tasks.
 * Safe to call before esp_zb_start(); frames arriving before the network is
 * joined are answered with NUOS_LINK_ST_NOT_JOINED and dropped.
 */
esp_err_t nuos_link_init(void);

/*
 * Called from the Zigbee task once esp_zb_start() has returned.
 *
 * Before that point esp_zb_lock_acquire() must not be used - the stack
 * mutex does not exist yet - so the link reports "not joined" and the
 * ESP32-C6 stays idle.
 */
void nuos_link_notify_zb_started(void);

void nuos_link_log_stats(void);

#endif /* NUOS_UART_LINK_H_ */
