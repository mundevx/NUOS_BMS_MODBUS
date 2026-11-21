/*
 * NUOS BMS - UART link protocol between the Modbus board and the Zigbee board
 *
 * THIS FILE MUST BE BYTE-IDENTICAL IN BOTH PROJECTS.
 *   01_esp32c6_modbus_uart/main/nuos_link_proto.h   (producer, ESP32-C6)
 *   02_esp32h2_zigbee_bridge/main/nuos_link_proto.h (consumer, ESP32-H2)
 *
 * -------------------------------------------------------------------------
 * FRAME LAYOUT
 * -------------------------------------------------------------------------
 *
 *   off  size  field
 *   ---  ----  ---------------------------------------------------------
 *    0    1    SOF0        0xA5
 *    1    1    SOF1        0x5A
 *    2    1    VER         0x01
 *    3    1    TYPE        see NUOS_LINK_TYPE_*
 *    4    1    SEQ         wraps 0..255, echoed in the ACK
 *    5    1    ENDPOINT    Zigbee endpoint this frame refers to (1..30)
 *    6    1    LEN         payload length, 0..NUOS_LINK_MAX_PAYLOAD
 *    7   LEN   PAYLOAD
 *   7+L   2    CRC16       little-endian, CRC-16/MODBUS over off 2..6+LEN
 *                          (VER through the last payload byte)
 *
 * The CRC deliberately reuses the same polynomial/seed as the Modbus CRC
 * already implemented on the C6, so there is only one CRC routine in the
 * product.
 *
 * SOF + LEN + CRC means the receiver resynchronises on its own. That
 * matters because the ROM bootloader of the sender prints a few bytes on
 * UART0 at every reset; those bytes are simply dropped as junk.
 *
 * -------------------------------------------------------------------------
 * EP_DATA payload (type 0x01, C6 -> H2)
 * -------------------------------------------------------------------------
 *   10 x uint32_t, little-endian = 40 bytes.
 *   Total frame on the wire: 7 + 40 + 2 = 49 bytes.
 *
 * -------------------------------------------------------------------------
 * RAW_ASDU payload (type 0x04, C6 -> H2) - optional passthrough mode
 * -------------------------------------------------------------------------
 *   off  size  field
 *    0    2    cluster id     (little-endian)
 *    1    2    dst short addr (little-endian)
 *    4    1    dst endpoint
 *    5    n    pre-built ZCL ASDU, relayed verbatim by APSDE-DATA.request
 *
 *   Use this only if you want the ZCL frame built on the Modbus board.
 *   The default and recommended mode is EP_DATA, which keeps all APS
 *   size/chunking knowledge on the Zigbee board where the APS limit is.
 */

#ifndef NUOS_LINK_PROTO_H_
#define NUOS_LINK_PROTO_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* Wire constants                                                             */
/* -------------------------------------------------------------------------- */

#define NUOS_LINK_SOF0              0xA5u
#define NUOS_LINK_SOF1              0x5Au
#define NUOS_LINK_VERSION           0x01u

#define NUOS_LINK_HDR_LEN           7u
#define NUOS_LINK_CRC_LEN           2u
#define NUOS_LINK_MAX_PAYLOAD       64u
#define NUOS_LINK_MAX_FRAME \
    (NUOS_LINK_HDR_LEN + NUOS_LINK_MAX_PAYLOAD + NUOS_LINK_CRC_LEN)

#define NUOS_LINK_OFF_VER           2u
#define NUOS_LINK_OFF_TYPE          3u
#define NUOS_LINK_OFF_SEQ           4u
#define NUOS_LINK_OFF_EP            5u
#define NUOS_LINK_OFF_LEN           6u
#define NUOS_LINK_OFF_PAYLOAD       7u

/* Frame types */
#define NUOS_LINK_TYPE_EP_DATA      0x01u   /* C6 -> H2 : 10 x uint32       */
#define NUOS_LINK_TYPE_ACK          0x02u   /* H2 -> C6 : 1 byte status     */
#define NUOS_LINK_TYPE_NAK          0x03u   /* H2 -> C6 : 1 byte reason     */
#define NUOS_LINK_TYPE_RAW_ASDU     0x04u   /* C6 -> H2 : verbatim ASDU     */
#define NUOS_LINK_TYPE_STATUS       0x10u   /* H2 -> C6 : 1 byte flags      */
#define NUOS_LINK_TYPE_PING         0x11u   /* C6 -> H2 : ask for STATUS    */

/* ACK / NAK status codes */
#define NUOS_LINK_ST_OK             0x00u
#define NUOS_LINK_ST_NOT_JOINED     0x01u   /* Zigbee network not joined    */
#define NUOS_LINK_ST_BAD_ENDPOINT   0x02u
#define NUOS_LINK_ST_BAD_LENGTH     0x03u
#define NUOS_LINK_ST_ZB_ERROR       0x04u   /* set_attr / APSDE failed      */
#define NUOS_LINK_ST_BUSY           0x05u   /* work queue full              */

/* STATUS payload flags */
#define NUOS_LINK_FLAG_ZB_JOINED    0x01u

/* -------------------------------------------------------------------------- */
/* Shared application constants                                               */
/* -------------------------------------------------------------------------- */

#define NUOS_LINK_ATTR_COUNT        10u     /* uint32 values per endpoint   */
#define NUOS_LINK_FIRST_ENDPOINT    1u
#define NUOS_LINK_TOTAL_ENDPOINTS   30u

#define NUOS_LINK_EP_DATA_LEN       (NUOS_LINK_ATTR_COUNT * 4u)   /* 40 */

#define NUOS_LINK_BAUDRATE          115200

/* -------------------------------------------------------------------------- */
/* CRC-16/MODBUS                                                              */
/* -------------------------------------------------------------------------- */

static inline uint16_t nuos_link_crc16(const uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFFu;

    for (uint16_t pos = 0; pos < length; pos++) {

        crc ^= (uint16_t)buffer[pos];

        for (uint8_t i = 0; i < 8; i++) {

            if (crc & 0x0001u) {
                crc >>= 1;
                crc ^= 0xA001u;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* -------------------------------------------------------------------------- */
/* Little-endian helpers                                                      */
/* -------------------------------------------------------------------------- */

static inline void nuos_link_put_u16(uint8_t *out, uint16_t v)
{
    out[0] = (uint8_t)(v & 0xFFu);
    out[1] = (uint8_t)(v >> 8);
}

static inline uint16_t nuos_link_get_u16(const uint8_t *in)
{
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static inline void nuos_link_put_u32(uint8_t *out, uint32_t v)
{
    out[0] = (uint8_t)(v & 0xFFu);
    out[1] = (uint8_t)((v >> 8) & 0xFFu);
    out[2] = (uint8_t)((v >> 16) & 0xFFu);
    out[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline uint32_t nuos_link_get_u32(const uint8_t *in)
{
    return (uint32_t)in[0] |
           ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) |
           ((uint32_t)in[3] << 24);
}

static inline uint8_t nuos_link_pack_values(uint8_t *out,
                                            const uint32_t *values,
                                            uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        nuos_link_put_u32(&out[i * 4u], values[i]);
    }

    return (uint8_t)(count * 4u);
}

static inline void nuos_link_unpack_values(const uint8_t *in,
                                           uint32_t *values,
                                           uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        values[i] = nuos_link_get_u32(&in[i * 4u]);
    }
}

/* -------------------------------------------------------------------------- */
/* Frame builder                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Serialises one frame into out[] (must be NUOS_LINK_MAX_FRAME bytes).
 * Returns the total number of bytes to transmit, or 0 on bad arguments.
 */
static inline uint16_t nuos_link_build(uint8_t *out,
                                       uint8_t type,
                                       uint8_t seq,
                                       uint8_t endpoint,
                                       const uint8_t *payload,
                                       uint8_t len)
{
    if (out == NULL || len > NUOS_LINK_MAX_PAYLOAD) {
        return 0;
    }

    if (len > 0 && payload == NULL) {
        return 0;
    }

    out[0]                    = NUOS_LINK_SOF0;
    out[1]                    = NUOS_LINK_SOF1;
    out[NUOS_LINK_OFF_VER]    = NUOS_LINK_VERSION;
    out[NUOS_LINK_OFF_TYPE]   = type;
    out[NUOS_LINK_OFF_SEQ]    = seq;
    out[NUOS_LINK_OFF_EP]     = endpoint;
    out[NUOS_LINK_OFF_LEN]    = len;

    for (uint8_t i = 0; i < len; i++) {
        out[NUOS_LINK_OFF_PAYLOAD + i] = payload[i];
    }

    uint16_t crc_span = (uint16_t)(NUOS_LINK_HDR_LEN - NUOS_LINK_OFF_VER + len);
    uint16_t crc = nuos_link_crc16(&out[NUOS_LINK_OFF_VER], crc_span);

    nuos_link_put_u16(&out[NUOS_LINK_OFF_PAYLOAD + len], crc);

    return (uint16_t)(NUOS_LINK_HDR_LEN + len + NUOS_LINK_CRC_LEN);
}

/* -------------------------------------------------------------------------- */
/* Byte-at-a-time parser with automatic resynchronisation                     */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t  type;
    uint8_t  seq;
    uint8_t  endpoint;
    uint8_t  len;
    const uint8_t *payload;     /* points into the parser buffer */
} nuos_link_frame_t;

typedef struct {
    uint8_t  buf[NUOS_LINK_MAX_FRAME];
    uint16_t idx;
    uint16_t need;              /* total frame length, known after LEN */
    uint32_t crc_errors;
    uint32_t resyncs;
} nuos_link_parser_t;

static inline void nuos_link_parser_init(nuos_link_parser_t *p)
{
    p->idx        = 0;
    p->need       = 0;
    p->crc_errors = 0;
    p->resyncs    = 0;
}

/*
 * Feeds one received byte. Returns true when *frame describes a complete,
 * CRC-valid frame. The payload pointer stays valid until the next call.
 */
static inline bool nuos_link_parser_push(nuos_link_parser_t *p,
                                         uint8_t byte,
                                         nuos_link_frame_t *frame)
{
    /* Hunt for the start-of-frame pattern. */
    if (p->idx == 0) {
        if (byte == NUOS_LINK_SOF0) {
            p->buf[p->idx++] = byte;
        }
        return false;
    }

    if (p->idx == 1) {
        if (byte == NUOS_LINK_SOF1) {
            p->buf[p->idx++] = byte;
        } else if (byte == NUOS_LINK_SOF0) {
            /* 0xA5 0xA5 ... - keep the newest 0xA5 as the SOF0 */
            p->idx = 1;
        } else {
            p->idx = 0;
        }
        return false;
    }

    p->buf[p->idx++] = byte;

    /* Version check as early as possible. */
    if (p->idx == NUOS_LINK_OFF_VER + 1u) {
        if (byte != NUOS_LINK_VERSION) {
            p->idx = 0;
            p->resyncs++;
        }
        return false;
    }

    /* LEN just arrived - we now know the full frame length. */
    if (p->idx == NUOS_LINK_OFF_LEN + 1u) {
        if (byte > NUOS_LINK_MAX_PAYLOAD) {
            p->idx = 0;
            p->resyncs++;
            return false;
        }
        p->need = (uint16_t)(NUOS_LINK_HDR_LEN + byte + NUOS_LINK_CRC_LEN);
        return false;
    }

    if (p->need == 0 || p->idx < p->need) {
        return false;
    }

    /* Full frame in the buffer - verify the CRC. */
    uint8_t  len      = p->buf[NUOS_LINK_OFF_LEN];
    uint16_t crc_span = (uint16_t)(NUOS_LINK_HDR_LEN - NUOS_LINK_OFF_VER + len);
    uint16_t calc     = nuos_link_crc16(&p->buf[NUOS_LINK_OFF_VER], crc_span);
    uint16_t rx       = nuos_link_get_u16(&p->buf[NUOS_LINK_OFF_PAYLOAD + len]);

    p->idx  = 0;
    p->need = 0;

    if (calc != rx) {
        p->crc_errors++;
        return false;
    }

    frame->type     = p->buf[NUOS_LINK_OFF_TYPE];
    frame->seq      = p->buf[NUOS_LINK_OFF_SEQ];
    frame->endpoint = p->buf[NUOS_LINK_OFF_EP];
    frame->len      = len;
    frame->payload  = &p->buf[NUOS_LINK_OFF_PAYLOAD];

    return true;
}

#endif /* NUOS_LINK_PROTO_H_ */
