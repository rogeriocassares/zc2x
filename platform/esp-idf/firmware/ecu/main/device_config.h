#pragma once

#include "hal/gpio_types.h"

/* -----------------------------------------------------------------------
 * ECU hardware and simulator configuration.
 *
 * Pins are selected per chip below, based on whichever target you picked
 * with `idf.py set-target` (see scripts/set-target.sh for a friendly
 * wrapper: wrover -> esp32, s3 -> esp32s3, c6mini -> esp32c6). set-target
 * itself can't be driven from this file — it configures the toolchain
 * before the compiler even runs — but once it's set, CONFIG_IDF_TARGET_*
 * is defined automatically by ESP-IDF and picks the right pins here.
 * Add a new #elif branch to support another board; nothing else changes.
 * ----------------------------------------------------------------------- */
#if CONFIG_IDF_TARGET_ESP32S3
#define ECU_CAN_TX_PIN GPIO_NUM_1
#define ECU_CAN_RX_PIN GPIO_NUM_21
#elif CONFIG_IDF_TARGET_ESP32C6
#error "ECU pins not yet defined for esp32c6 — fill in ECU_CAN_TX_PIN/ECU_CAN_RX_PIN for your C6 board wiring, then remove this #error."
#elif CONFIG_IDF_TARGET_ESP32
#error "ECU pins not yet defined for esp32 (Wrover) — fill in ECU_CAN_TX_PIN/ECU_CAN_RX_PIN for your Wrover board wiring, then remove this #error."
#else
#error "Unsupported IDF target for ECU — add a pin block for this chip above."
#endif

/* CAN simulator timing */
#define ECU_CAN_BITRATE 1000000

/* Scheduler tick: the can_tx_task loop period, in ms. This must be the GCD
 * of every ECU_CAN_PERIOD_*_MS below (10 ms here) so each message fires on
 * an exact tick boundary without needing multiple tasks or an LCM scheme. */
#define ECU_CAN_TICK_MS 10U

/* CAN2 message IDs and periods (standard 11-bit IDs). Mirrors
 * docs/architecture/zc2x-can2.dbc — keep both in sync by hand. This is the
 * message/signal layout MoTeC M1 Tune transmits on CAN2 in production; here
 * it's simulated so OBU/RSU/services can be exercised without the car. */
#define ECU_CAN_ID_WHEEL_SPEEDS 0x100U
#define ECU_CAN_PERIOD_WHEEL_SPEEDS_MS 10U /* 100 Hz */

#define ECU_CAN_ID_CHASSIS_DYNAMICS 0x110U
#define ECU_CAN_PERIOD_CHASSIS_DYNAMICS_MS 10U /* 100 Hz */

#define ECU_CAN_ID_BRAKES 0x120U
#define ECU_CAN_PERIOD_BRAKES_MS 10U /* 100 Hz */

#define ECU_CAN_ID_ENGINE_CORE 0x200U
#define ECU_CAN_PERIOD_ENGINE_CORE_MS 20U /* 50 Hz */

#define ECU_CAN_ID_ENGINE_TEMPS_PRESSURES 0x210U
#define ECU_CAN_PERIOD_ENGINE_TEMPS_PRESSURES_MS 100U /* 10 Hz */

#define ECU_CAN_ID_INTAKE_TEMP_FUEL_USED 0x211U
#define ECU_CAN_PERIOD_INTAKE_TEMP_FUEL_USED_MS 100U /* 10 Hz */

#define ECU_CAN_ID_EXHAUST_TEMPS 0x220U
#define ECU_CAN_PERIOD_EXHAUST_TEMPS_MS 200U /* 5 Hz */

/* -----------------------------------------------------------------------
 * Tuning — queue/stack sizing. Bump these if you see TWAI tx timeouts or
 * task-related crashes; no other file needs to change.
 * ----------------------------------------------------------------------- */

/* TWAI driver tx queue depth (frames buffered before twai_node_transmit blocks) */
#define ECU_CAN_TX_QUEUE_DEPTH 8

/* can_tx_task stack size, in words (4 bytes each on esp32s3) */
#define ECU_CAN_TX_TASK_STACK 4096

/* How often can_tx_task logs a periodic queued/failed summary instead of
 * staying silent between boot and the first failure. See OBU_STATS_LOG_
 * INTERVAL_MS for the same reasoning — per-frame logging at up to 300
 * msgs/sec would flood the console. */
#define ECU_STATS_LOG_INTERVAL_MS 1000
