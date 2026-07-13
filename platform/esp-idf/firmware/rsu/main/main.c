/**
 * @file main.c
 * @brief RSU firmware entry point.
 *
 * Data path: XBee (UART1) → zc2x_packet_t → WiFi → NATS Core
 *
 * Wire format received on UART1:
 *   [0xAA][0x55][0xC2][0x58][fixed-size packet bytes]
 *
 * The RSU validates each received packet and publishes the raw packet bytes
 * to the NATS subject RSU_NATS_SUBJECT.  Packet contents are never modified.
 *
 * NATS client is a minimal TCP implementation (publish-only, PING/PONG).
 * No JetStream, no TLS, no subscriptions — MVP only.
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "device_config.h"
#include "zc2x_packet.h"

static const char *TAG = "RSU";

/** Total bytes ever read from UART1 (any value). 0 = wir   ing/RF problem. */
static uint32_t s_uart_rx_total;
static bool s_uart_sync_seen;
static QueueHandle_t s_packet_queue;

/* -------------------------------------------------------------------------
 * WiFi
 * ---------------------------------------------------------------------- */

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
  strncpy((char *)wifi_cfg.sta.ssid, RSU_WIFI_SSID,
          sizeof(wifi_cfg.sta.ssid) - 1);
  strncpy((char *)wifi_cfg.sta.password, RSU_WIFI_PASS,
          sizeof(wifi_cfg.sta.password) - 1);
  wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "WiFi init done — connecting to SSID: %s", RSU_WIFI_SSID);
}

/* -------------------------------------------------------------------------
 * Minimal NATS TCP client
 *
 * Protocol (NATS Core, plain TCP):
 *   ← INFO {...}\r\n
 *   → CONNECT {"verbose":false}\r\n
 *   → PUB <subject> <N>\r\n<N bytes payload>\r\n
 *   ← PING\r\n  (periodic server keepalive)
 *   → PONG\r\n
 * ---------------------------------------------------------------------- */

static int nats_connect(void)
{
  struct sockaddr_in addr = {
      .sin_family = AF_INET,
      .sin_port = htons(RSU_NATS_PORT),
  };
  inet_pton(AF_INET, RSU_NATS_SERVER, &addr.sin_addr);

  int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock < 0)
  {
    ESP_LOGE(TAG, "socket() failed: errno %d", errno);
    return -1;
  }

  if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0)
  {
    ESP_LOGE(TAG, "connect() to %s:%d failed: errno %d",
             RSU_NATS_SERVER, RSU_NATS_PORT, errno);
    close(sock);
    return -1;
  }

  /* Drain the INFO message sent by the server on connect */
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

  /* Send CONNECT with verbose disabled to suppress +OK responses */
  const char *connect_msg = "CONNECT {\"verbose\":false}\r\n";
  if (send(sock, connect_msg, strlen(connect_msg), 0) < 0)
  {
    ESP_LOGE(TAG, "send CONNECT failed");
    close(sock);
    return -1;
  }

  ESP_LOGI(TAG, "Connected to NATS %s:%d", RSU_NATS_SERVER, RSU_NATS_PORT);
  return sock;
}

/**
 * @brief Publish binary data to a NATS subject.
 *
 * @return 0 on success, -1 on send error (caller should reconnect).
 */
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

/**
 * @brief Drain pending data from the NATS socket and respond to PING.
 *
 * Called after each publish to keep the connection alive.
 */
static void nats_process(int sock)
{
  char buf[128];
  ssize_t n = recv(sock, buf, sizeof(buf) - 1, MSG_DONTWAIT);
  if (n > 0)
  {
    buf[n] = '\0';
    if (strstr(buf, "PING") != NULL)
    {
      send(sock, "PONG\r\n", 6, 0);
    }
  }
}

/* -------------------------------------------------------------------------
 * UART1 (XBee) initialisation
 * ---------------------------------------------------------------------- */

static void xbee_init(void)
{
  const uart_config_t cfg = {
      .baud_rate = RSU_XBEE_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  /* Install driver first — ESP-IDF applies pin routing after install. */
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 4096, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,
                               RSU_XBEE_TX_PIN, RSU_XBEE_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_LOGI(TAG, "UART1 (XBee) ready: TX=%d RX=%d baud=%d",
           RSU_XBEE_TX_PIN, RSU_XBEE_RX_PIN, RSU_XBEE_BAUD);
  ESP_LOGI(TAG,
           "XBee DOUT must reach GPIO%d. XCTU on this XBee does NOT loop "
           "sent bytes back to the ESP32 — use a second radio or wire a USB "
           "serial adapter TX directly to GPIO%d for a local test.",
           RSU_XBEE_RX_PIN, RSU_XBEE_RX_PIN);
}

/* -------------------------------------------------------------------------
 * uart_read_exact — read exactly 'len' bytes with a total deadline.
 *
 * WHY: uart_read_bytes() applies its timeout per ring-buffer chunk, not
 * to the total requested length.  When the XBee splits a single logical
 * packet across two or more RF transmissions (e.g. 15 bytes arrive, then
 * 26 bytes arrive 200 ms later), a plain uart_read_bytes(26, 500ms) may
 * return after only 15 bytes, treating the rest as a new unrelated stream.
 *
 * This helper loops, accumulating bytes, until 'len' bytes are collected
 * or 'total_ms' milliseconds have elapsed from the first call.
 *
 * Returns the number of bytes actually read (== len on success).
 * ---------------------------------------------------------------------- */
static int uart_read_exact(uint8_t *buf, int len, int total_ms)
{
  int received = 0;
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(total_ms);

  while (received < len)
  {
    TickType_t now = xTaskGetTickCount();
    TickType_t remaining = (deadline > now) ? (deadline - now) : 0;
    if (remaining == 0)
      break;

    int n = uart_read_bytes(UART_NUM_1, buf + received,
                            (uint32_t)(len - received), remaining);
    if (n > 0)
      received += n;
  }
  return received;
}

static void log_packet_bytes(const zc2x_packet_t *pkt)
{
  const uint8_t *raw = (const uint8_t *)pkt;
  char hex[(sizeof(zc2x_packet_t) * 3U) + 1U];
  size_t pos = 0;

  for (size_t i = 0; i < sizeof(zc2x_packet_t) && pos + 3U < sizeof(hex); ++i)
  {
    int written = snprintf(&hex[pos], sizeof(hex) - pos, "%02X%s",
                           raw[i], (i + 1U < sizeof(zc2x_packet_t)) ? " " : "");
    if (written < 0)
    {
      break;
    }
    pos += (size_t)written;
  }

  hex[sizeof(hex) - 1U] = '\0';

  ESP_LOGI(TAG,
           "RX pkt fields: type=%u device_id=%02X %02X %02X %02X %02X %02X seq=%lu ts=%llu can_id=0x%08lx dlc=%u crc16=0x%04x",
           (unsigned)pkt->type,
           pkt->device_id[0], pkt->device_id[1], pkt->device_id[2],
           pkt->device_id[3], pkt->device_id[4], pkt->device_id[5],
           (unsigned long)pkt->sequence,
           (unsigned long long)pkt->timestamp,
           (unsigned long)pkt->can_id,
           (unsigned)pkt->dlc,
           (unsigned)pkt->crc16);

  ESP_LOGI(TAG, "RX pkt bytes: %s", hex);
}

/* -------------------------------------------------------------------------
 * UART receive task
 * ---------------------------------------------------------------------- */

static void uart_rx_task(void *arg)
{
  zc2x_packet_t pkt;

  while (1)
  {
    /* --- Step 1: scan for sync marker {0xAA, 0x55, 0xC2, 0x58} then read header ---
     *
     * Read byte-by-byte until the 4-byte marker is found.  This guarantees
     * byte alignment regardless of startup noise, XBee resets, or any
     * mid-packet disruption on the radio link.
     * -------------------------------------------------------------------*/
    {
      uint8_t b;
      uint8_t sync[4] = {0};
      uint32_t skipped = 0;
      uint32_t idle_reports = 0;

      while (1)
      {
        int nr = uart_read_bytes(UART_NUM_1, &b, 1, pdMS_TO_TICKS(500));
        if (nr != 1)
        {
          idle_reports++;
          if (idle_reports % 6 == 0) /* ~3 s */
          {
            size_t buffered = 0;
            uart_get_buffered_data_len(UART_NUM_1, &buffered);
            ESP_LOGI(TAG,
                     "UART1 alive — waiting for sync {0xAA 0x55 0xC2 0x58} "
                     "(rx_total=%lu, skipped=%lu, buffered=%u, idle=%lu x 500ms)",
                     (unsigned long)s_uart_rx_total,
                     (unsigned long)skipped,
                     (unsigned)buffered,
                     (unsigned long)idle_reports);
          }
          vTaskDelay(pdMS_TO_TICKS(1));
          continue;
        }

        idle_reports = 0;
        s_uart_rx_total++;

        sync[0] = sync[1];
        sync[1] = sync[2];
        sync[2] = sync[3];
        sync[3] = b;

        if (sync[0] == 0xAAu && sync[1] == 0x55u &&
            sync[2] == 0xC2u && sync[3] == 0x58u)
        {
          break; /* sync marker found */
        }
        skipped++;
      }

      if (skipped > 0)
      {
        if (s_uart_sync_seen)
        {
          ESP_LOGW(TAG, "UART sync re-aligned after %lu spurious bytes",
                   (unsigned long)skipped);
        }
        else
        {
          ESP_LOGI(TAG, "UART sync aligned after %lu startup bytes",
                   (unsigned long)skipped);
          s_uart_sync_seen = true;
        }
      }
    }

    /* Read the fixed packet body — 2 s total deadline covers XBee splits */
    int n = uart_read_exact((uint8_t *)&pkt, (int)sizeof(zc2x_packet_t), 2000);

    if (n != (int)sizeof(zc2x_packet_t))
    {
      ESP_LOGW(TAG, "incomplete packet after sync (%d/%u) -- discarding",
               n, (unsigned)sizeof(zc2x_packet_t));
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    /* --- Step 2: validate the packet --- */
    zc2x_result_t res = zc2x_packet_validate(&pkt);
    if (res != ZC2X_OK)
    {
      ESP_LOGW(TAG, "packet validation failed (%d) — discarding seq=%lu",
               res, (unsigned long)pkt.sequence);
      continue;
    }

    log_packet_bytes(&pkt);

    if (xQueueSend(s_packet_queue, &pkt, 0) != pdTRUE)
    {
      ESP_LOGW(TAG, "packet queue full — dropping seq=%lu",
               (unsigned long)pkt.sequence);
    }
  }
}

/* -------------------------------------------------------------------------
 * NATS publish task
 * ---------------------------------------------------------------------- */

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

    if (xQueueReceive(s_packet_queue, &pkt, pdMS_TO_TICKS(250)) != pdTRUE)
    {
      nats_process(nats_sock);
      continue;
    }

    if (nats_publish(nats_sock, RSU_NATS_SUBJECT,
                     (const uint8_t *)&pkt, sizeof(pkt)) != 0)
    {
      ESP_LOGE(TAG, "NATS publish failed — reconnecting");
      close(nats_sock);
      nats_sock = -1;
      continue;
    }

    nats_process(nats_sock);

    ESP_LOGI(TAG, "→ NATS src=rsu subject='%s' seq=%lu type=%u (%zu bytes)",
             RSU_NATS_SUBJECT,
             (unsigned long)pkt.sequence,
             pkt.type,
             sizeof(pkt));
  }
}

/* -------------------------------------------------------------------------
 * Entry point
 * ---------------------------------------------------------------------- */

void app_main(void)
{
  ESP_LOGI(TAG, "ZC2X RSU starting");

  /* NVS must be initialised before WiFi */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  s_packet_queue = xQueueCreate(16, sizeof(zc2x_packet_t));
  assert(s_packet_queue != NULL);

  xbee_init();
  wifi_init();

  xTaskCreate(uart_rx_task, "uart_rx_task", 8192, NULL,
              configMAX_PRIORITIES - 2, NULL);
  xTaskCreate(nats_task, "nats_task", 8192, NULL,
              configMAX_PRIORITIES - 3, NULL);
}