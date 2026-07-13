#pragma once

#include "hal/gpio_types.h"

/* -----------------------------------------------------------------------
 * RSU hardware and network configuration.
 * Update these definitions to match your deployment environment.
 * ----------------------------------------------------------------------- */

/* XBee radio — UART1 */
#define RSU_XBEE_TX_PIN GPIO_NUM_2
#define RSU_XBEE_RX_PIN GPIO_NUM_3
#define RSU_XBEE_BAUD 115200

/* WiFi credentials */
#define RSU_WIFI_SSID "CompreFSAELive👕👚🎥"
#define RSU_WIFI_PASS "Formulive25*"
// #define RSU_WIFI_SSID "Roger_Phone"
// #define RSU_WIFI_PASS "12345678"

/* NATS Core server */

// #define RSU_NATS_SERVER "192.168.80.10" /* update to your NATS server IP */
#define RSU_NATS_SERVER "192.168.80.10" /* update to your NATS server IP */
#define RSU_NATS_PORT 4222
#define RSU_NATS_SUBJECT "zc2x.can.rsu"

/* 6-byte device identifier for this RSU unit */
#define RSU_DEVICE_ID {0x02, 0x00, 0x00, 0x00, 0x00, 0x00}
