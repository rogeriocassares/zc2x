/**
 * @file main.c
 * @brief OBU firmware entry point.
 *
 * Data path:
 *   CAN (TWAI) -> zc2x_packet_t -> XBee (UART1)
 *   CAN (TWAI) -> zc2x_packet_t -> WiFi -> NATS Core
 *
 * XBee and NATS paths are independent: a WiFi/NATS outage must not block
 * CAN reception or XBee transmission.
 */

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "device_config.h"
#include "zc2x_packet.h"

static const char *TAG = "OBU";

typedef struct __attribute__((packed))
{
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} zc2x_can_payload_t;

typedef struct
{
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} can_rx_frame_t;

static const uint8_t s_device_id[ZC2X_DEVICE_ID_SIZE] = OBU_DEVICE_ID;
static uint32_t s_sequence = 0;
static twai_node_handle_t s_twai_node;
static QueueHandle_t s_can_rx_queue;
static QueueHandle_t s_nats_queue;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
  {
    ESP_LOGW(TAG, "WiFi disconnected — retrying");
    esp_wifi_connect();
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
  else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "WiFi connected — IP " IPSTR, IP2STR(&ev->ip_info.ip));
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static void wifi_init(void)
{
  s_wifi_event_group = xEventGroupCreate();
  assert(s_wifi_event_group != NULL);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &wifi_event_handler, NULL));

  wifi_config_t wifi_cfg = {};
  strncpy((char *)wifi_cfg.sta.ssid, OBU_WIFI_SSID,
          sizeof(wifi_cfg.sta.ssid) - 1);
  strncpy((char *)wifi_cfg.sta.password, OBU_WIFI_PASS,
          sizeof(wifi_cfg.sta.password) - 1);
  wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "WiFi init done — connecting to SSID: %s", OBU_WIFI_SSID);
}

static int nats_connect(void)
{
  struct sockaddr_in addr = {
      .sin_family = AF_INET,
      .sin_port = htons(OBU_NATS_PORT),
  };
  inet_pton(AF_INET, OBU_NATS_SERVER, &addr.sin_addr);

  int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock < 0)
  {
    ESP_LOGE(TAG, "socket() failed: errno %d", errno);
    return -1;
  }

  if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0)
  {
    ESP_LOGE(TAG, "connect() to %s:%d failed: errno %d",
             OBU_NATS_SERVER, OBU_NATS_PORT, errno);
    close(sock);
    return -1;
  }

  char info_buf[512];
  int n = recv(sock, info_buf, sizeof(info_buf) - 1, 0);
  if (n <= 0)
  {
    ESP_LOGE(TAG, "recv INFO failed");
    close(sock);
    return -1;
  }
  info_buf[n] = '\0';
  ESP_LOGI(TAG, "NATS server: %.80s", info_buf);

  const char *connect_msg = "CONNECT {\"verbose\":false}\r\n";
  if (send(sock, connect_msg, strlen(connect_msg), 0) < 0)
  {
    ESP_LOGE(TAG, "send CONNECT failed");
    close(sock);
    return -1;
  }

  ESP_LOGI(TAG, "Connected to NATS %s:%d", OBU_NATS_SERVER, OBU_NATS_PORT);
  return sock;
}

static int nats_publish(int sock, const char *subject,
                        const uint8_t *data, size_t len)
{
  char header[64];
  int hlen = snprintf(header, sizeof(header),
                      "PUB %s %zu\r\n", subject, len);

  if (send(sock, header, (size_t)hlen, 0) < 0)
    return -1;
  if (send(sock, data, len, 0) < 0)
    return -1;
  if (send(sock, "\r\n", 2, 0) < 0)
    return -1;

  return 0;
}

static int nats_process(int sock)
{
  char buf[128];
  ssize_t n = recv(sock, buf, sizeof(buf) - 1, MSG_DONTWAIT);
  if (n > 0)
  {
    buf[n] = '\0';
    if (strstr(buf, "PING") != NULL)
    {
      if (send(sock, "PONG\r\n", 6, 0) < 0)
      {
        return -1;
      }
    }
    return 0;
  }

  if (n == 0)
  {
    return -1;
  }

  if (errno == EWOULDBLOCK || errno == EAGAIN)
  {
    return 0;
  }

  return -1;
}

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
          .bitrate = 500000,
      },
      .tx_queue_depth = 1,
      .flags = {
          .enable_listen_only = 1,
      },
  };
  ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &s_twai_node));

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

    if (xQueueSend(s_nats_queue, &pkt, 0) != pdTRUE)
    {
      ESP_LOGW(TAG, "NATS queue full — dropping seq=%lu",
               (unsigned long)pkt.sequence);
    }

    ESP_LOGI(TAG, "CAN id=0x%03lx dlc=%u seq=%lu sent xbee+nats-queue",
             (unsigned long)rx.id, (unsigned)rx.dlc,
             (unsigned long)s_sequence);

    s_sequence++;
  }
}

static void nats_task(void *arg)
{
  int nats_sock = -1;
  zc2x_packet_t pkt;

  while (1)
  {
    EventBits_t wifi_bits = xEventGroupGetBits(s_wifi_event_group);
    if ((wifi_bits & WIFI_CONNECTED_BIT) == 0)
    {
      if (nats_sock >= 0)
      {
        close(nats_sock);
        nats_sock = -1;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (nats_sock < 0)
    {
      nats_sock = nats_connect();
      if (nats_sock < 0)
      {
        ESP_LOGW(TAG, "NATS connect failed — retrying in 5 s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
    }

    if (xQueueReceive(s_nats_queue, &pkt, pdMS_TO_TICKS(250)) != pdTRUE)
    {
      if (nats_process(nats_sock) != 0)
      {
        ESP_LOGW(TAG, "NATS keepalive failed — reconnecting");
        close(nats_sock);
        nats_sock = -1;
      }
      continue;
    }

    if (nats_publish(nats_sock, OBU_NATS_SUBJECT,
                     (const uint8_t *)&pkt, sizeof(pkt)) != 0)
    {
      ESP_LOGE(TAG, "NATS publish failed — reconnecting");
      close(nats_sock);
      nats_sock = -1;
      continue;
    }

    if (nats_process(nats_sock) != 0)
    {
      ESP_LOGW(TAG, "NATS keepalive failed — reconnecting");
      close(nats_sock);
      nats_sock = -1;
      continue;
    }

    ESP_LOGI(TAG, "→ NATS src=obu subject='%s' seq=%lu type=%u (%zu bytes)",
             OBU_NATS_SUBJECT,
             (unsigned long)pkt.sequence,
             pkt.type,
             sizeof(pkt));
  }
}

void app_main(void)
{
  ESP_LOGI(TAG, "ZC2X OBU starting");

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  s_nats_queue = xQueueCreate(32, sizeof(zc2x_packet_t));
  assert(s_nats_queue != NULL);

  xbee_init();
  can_init();
  wifi_init();

  xTaskCreate(can_rx_task, "can_rx_task", 6144, NULL,
              configMAX_PRIORITIES - 1, NULL);
  xTaskCreate(nats_task, "nats_task", 8192, NULL,
              configMAX_PRIORITIES - 2, NULL);
}
