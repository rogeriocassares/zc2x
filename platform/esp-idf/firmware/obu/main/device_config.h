#pragma once

#include "hal/gpio_types.h"

/* -----------------------------------------------------------------------
 * OBU hardware configuration.
 * Update these definitions to match your board layout.
 * ----------------------------------------------------------------------- */

/* CAN (TWAI) — connect to the vehicle CAN bus transceiver */
#define OBU_CAN_TX_PIN GPIO_NUM_4
#define OBU_CAN_RX_PIN GPIO_NUM_5

/* XBee radio — UART1 */
// #define OBU_XBEE_TX_PIN GPIO_NUM_17 // XBee DIn
// #define OBU_XBEE_RX_PIN GPIO_NUM_16 // Xbee DOut
#define OBU_XBEE_TX_PIN GPIO_NUM_2
#define OBU_XBEE_RX_PIN GPIO_NUM_3
#define OBU_XBEE_BAUD 115200

/* WiFi credentials */
#define OBU_WIFI_SSID "CompreFSAELive👕👚🎥"
#define OBU_WIFI_PASS "Formulive25*"

/* NATS Core server */
#define OBU_NATS_SERVER "192.168.80.10"
#define OBU_NATS_PORT 4222
#define OBU_NATS_SUBJECT "zc2x.can.obu"

/* 6-byte device identifier for this OBU unit */
#define OBU_DEVICE_ID {0x01, 0x00, 0x00, 0x00, 0x00, 0x00}
