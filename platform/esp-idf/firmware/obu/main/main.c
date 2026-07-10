/**
 * @file main.c
 * @brief OBU firmware entry point.
 *
 * Data path: CAN (TWAI) -> zc2x_packet_t -> XBee (UART1)
 *
 * Wire format transmitted over UART1:
 *   [0xAA][0x55][0xC2][0x58][fixed-size packet bytes]
 *
 * Uses the ESP-IDF v6.0 TWAI driver (esp_twai.h / esp_twai_onchip.h).
 * Frames are received via ISR callback and posted to a FreeRTOS queue
 * for processing by the main task.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "device_config.h"
#include "zc2x_packet.h"

static const char *TAG = "OBU";

/* -------------------------------------------------------------------------
 * CAN payload carried inside every zc2x_packet_t (13 bytes, packed)
 * ---------------------------------------------------------------------- */

typedef struct __attribute__((packed))
{
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} zc2x_can_payload_t;

/* Internal copy of a received CAN frame -- safe to queue between ISR/task */
typedef struct
{
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} can_rx_frame_t;

/* -------------------------------------------------------------------------
 * Static state
 * ---------------------------------------------------------------------- */

static const uint8_t s_device_id[ZC2X_DEVICE_ID_SIZE] = OBU_DEVICE_ID;
static uint32_t s_sequence = 0;
static twai_node_handle_t s_twai_node;
static QueueHandle_t s_can_rx_queue;

/* -------------------------------------------------------------------------
 * TWAI ISR callback -- runs in ISR context
 *
 * Reads the incoming frame from hardware and posts a copy to the queue.
 * Returns true when a higher-priority task has been woken.
 * ---------------------------------------------------------------------- */

static bool IRAM_ATTR on_can_rx_done(twai_node_handle_t node,
                                     const twai_rx_done_event_data_t *edata,
                                     void *user_ctx)
{
  uint8_t data_buf[8] = {0};
  twai_frame_t frame = {
      .buffer = data_buf,
      .buffer_len = sizeof(data_buf),
  };

  if (twai_node_receive_from_isr(node, &frame) != ESP_OK)
  {
    return false;
  }

  uint8_t dlc = (frame.header.dlc > 8U) ? 8U : (uint8_t)frame.header.dlc;

  can_rx_frame_t rx;
  rx.id = frame.header.id;
  rx.dlc = dlc;
  memcpy(rx.data, data_buf, dlc);

  BaseType_t higher_task_woken = pdFALSE;
  xQueueSendFromISR(s_can_rx_queue, &rx, &higher_task_woken);
  return higher_task_woken == pdTRUE;
}

/* -------------------------------------------------------------------------
 * UART1 (XBee) initialisation
 * ---------------------------------------------------------------------- */

static void xbee_init(void)
{
  const uart_config_t cfg = {
      .baud_rate = OBU_XBEE_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,
                               OBU_XBEE_TX_PIN, OBU_XBEE_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0));

  ESP_LOGI(TAG, "UART1 (XBee) ready: TX=%d RX=%d baud=%d",
           OBU_XBEE_TX_PIN, OBU_XBEE_RX_PIN, OBU_XBEE_BAUD);
}

/* -------------------------------------------------------------------------
 * TWAI (CAN) initialisation -- ESP-IDF v6.0 new API
 *
 * Listen-only: no ACKs transmitted, no effect on the live CAN bus.
 * Accept-all filter: mask = 0 means every ID bit is "don't care".
 * ---------------------------------------------------------------------- */

static void can_init(void)
{
  s_can_rx_queue = xQueueCreate(32, sizeof(can_rx_frame_t));
  assert(s_can_rx_queue != NULL);

  const twai_onchip_node_config_t node_cfg = {
      .io_cfg = {
          .tx = OBU_CAN_TX_PIN,
          .rx = OBU_CAN_RX_PIN,
          .quanta_clk_out = GPIO_NUM_NC,
          .bus_off_indicator = GPIO_NUM_NC,
      },
      .bit_timing = {
          .bitrate = 500000, /* 500 kbps -- update for your CAN bus speed */
      },
      .tx_queue_depth = 1,
      .flags = {
          .enable_listen_only = 1,
      },
  };
  ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &s_twai_node));

  /* Accept all CAN frames: mask = 0 means every bit is "don't care" */
  const twai_mask_filter_config_t filter = {
      .id = 0,
      .mask = 0,
  };
  ESP_ERROR_CHECK(twai_node_config_mask_filter(s_twai_node, 0, &filter));

  const twai_event_callbacks_t cbs = {
      .on_rx_done = on_can_rx_done,
  };
  ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_twai_node, &cbs, NULL));
  ESP_ERROR_CHECK(twai_node_enable(s_twai_node));

  ESP_LOGI(TAG, "TWAI (CAN) listening: TX=%d RX=%d 500 kbps",
           OBU_CAN_TX_PIN, OBU_CAN_RX_PIN);
}

/* -------------------------------------------------------------------------
 * Send a packet over UART1 (XBee)
 *
 * Wire format:
 *   [0xAA][0x55][0xC2][0x58][fixed-size packet bytes]
 *
 * The 4-byte sync marker makes false alignment much less likely than the
 * previous 2-byte marker. Only header + valid payload bytes are transmitted
 * (not the zero padding).
 * ---------------------------------------------------------------------- */

#define XBEE_SYNC_0 ((uint8_t)0xAAu)
#define XBEE_SYNC_1 ((uint8_t)0x55u)
#define XBEE_SYNC_2 ((uint8_t)0xC2u)
#define XBEE_SYNC_3 ((uint8_t)0x58u)

static void xbee_send_packet(const zc2x_packet_t *pkt)
{
  static const uint8_t sync[4] = {XBEE_SYNC_0, XBEE_SYNC_1,
                                  XBEE_SYNC_2, XBEE_SYNC_3};
  uart_write_bytes(UART_NUM_1, (const char *)sync, sizeof(sync));
  uart_write_bytes(UART_NUM_1, (const char *)pkt, sizeof(*pkt));
}

/* -------------------------------------------------------------------------
 * CAN processing task
 * ---------------------------------------------------------------------- */

static void can_rx_task(void *arg)
{
  can_rx_frame_t rx;
  zc2x_can_payload_t can_pl;
  zc2x_packet_t pkt;

  while (1)
  {
    if (xQueueReceive(s_can_rx_queue, &rx, portMAX_DELAY) != pdTRUE)
    {
      continue;
    }

    can_pl.id = rx.id;
    can_pl.dlc = rx.dlc;
    memcpy(can_pl.data, rx.data, rx.dlc);
    if (rx.dlc < 8U)
    {
      memset(can_pl.data + rx.dlc, 0, 8U - rx.dlc);
    }

    zc2x_result_t res = zc2x_packet_init(
        &pkt,
        ZC2X_PACKET_TYPE_CAN_FRAME,
        s_device_id,
        s_sequence,
        (uint64_t)esp_timer_get_time(),
        can_pl.id,
        can_pl.data,
        can_pl.dlc);

    if (res != ZC2X_OK)
    {
      ESP_LOGE(TAG, "packet_init failed (%d) seq=%lu dropped",
               res, (unsigned long)s_sequence);
      s_sequence++;
      continue;
    }

    xbee_send_packet(&pkt);

    ESP_LOGI(TAG, "CAN id=0x%03lx dlc=%u seq=%lu sent (%u bytes wire)",
             (unsigned long)rx.id, (unsigned)rx.dlc,
             (unsigned long)s_sequence,
             (unsigned)(4U + sizeof(zc2x_packet_t)));

    s_sequence++;
  }
}

/* -------------------------------------------------------------------------
 * Entry point
 * ---------------------------------------------------------------------- */

void app_main(void)
{
  ESP_LOGI(TAG, "ZC2X OBU starting");
  xbee_init();
  can_init();
  xTaskCreate(can_rx_task, "can_rx_task", 4096, NULL,
              configMAX_PRIORITIES - 1, NULL);
}
