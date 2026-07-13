/**
 * @file main.c
 * @brief ECU simulator firmware entry point.
 *
 * Data path: ECU simulator -> CAN (TWAI) -> OBU -> XBee -> RSU -> NATS
 *
 * This firmware continuously transmits CAN frames configured in
 * device_config.h so OBU/RSU can be validated end-to-end.
 */

#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "device_config.h"

static const char *TAG = "ECU";
static twai_node_handle_t s_twai_node;

static void can_init(void)
{
  const twai_onchip_node_config_t node_cfg = {
      .io_cfg = {
          .tx = ECU_CAN_TX_PIN,
          .rx = ECU_CAN_RX_PIN,
          .quanta_clk_out = GPIO_NUM_NC,
          .bus_off_indicator = GPIO_NUM_NC,
      },
      .bit_timing = {
          .bitrate = ECU_CAN_BITRATE,
      },
      .tx_queue_depth = 8,
      .flags = {
          .enable_listen_only = 0,
      },
  };

  ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &s_twai_node));
  ESP_ERROR_CHECK(twai_node_enable(s_twai_node));

  ESP_LOGI(TAG, "TWAI (CAN) tx ready: TX=%d RX=%d bitrate=%d",
           ECU_CAN_TX_PIN, ECU_CAN_RX_PIN, ECU_CAN_BITRATE);
}

static void can_tx_task(void *arg)
{
  static const uint8_t payload[8] = ECU_CAN_DATA_BYTES;
  const uint32_t tx_period_ms = (ECU_CAN_MESSAGES_PER_SEC == 0U)
                                    ? 1000U
                                    : (1000U / ECU_CAN_MESSAGES_PER_SEC);
  uint32_t seq = 0;

  while (1)
  {
    twai_frame_t frame = {
        .header = {
            .id = ECU_CAN_ID,
            .ide = 0,
            .rtr = 0,
            .dlc = ECU_CAN_DLC,
        },
        .buffer = (uint8_t *)payload,
        .buffer_len = ECU_CAN_DLC,
    };

    esp_err_t err = twai_node_transmit(s_twai_node, &frame, pdMS_TO_TICKS(100));
    if (err == ESP_OK)
    {
      ESP_LOGI(TAG, "CAN tx id=0x%03lx dlc=%u seq=%lu data=01 02 03 04 05 06 07 08",
               (unsigned long)ECU_CAN_ID,
               (unsigned)ECU_CAN_DLC,
               (unsigned long)seq);
      seq++;
    }
    else
    {
      ESP_LOGW(TAG, "CAN tx failed (err=0x%x)", (unsigned)err);
    }

    vTaskDelay(pdMS_TO_TICKS(tx_period_ms));
  }
}

void app_main(void)
{
  ESP_LOGI(TAG, "ZC2X ECU simulator starting");

  assert(ECU_CAN_DLC <= 8U);
  assert(ECU_CAN_MESSAGES_PER_SEC > 0U);

  can_init();

  xTaskCreate(can_tx_task, "can_tx_task", 4096, NULL,
              configMAX_PRIORITIES - 1, NULL);
}
