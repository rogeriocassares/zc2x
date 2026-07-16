/**
 * @file main.c
 * @brief ECU simulator firmware entry point.
 *
 * Data path: ECU simulator -> CAN (TWAI) -> OBU -> XBee -> RSU -> NATS
 *
 * Simulates the CAN2 message/signal set defined in
 * docs/architecture/zc2x-can2.dbc (the layout MoTeC M1 Tune transmits on
 * CAN2 in production) so OBU/RSU/services/input/nats can be validated
 * end-to-end without the car. Each message is emitted on its own configured
 * period (device_config.h) with per-signal values that wander plausibly
 * within realistic physical ranges via a bounded random walk, rather than
 * static bytes.
 */

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "device_config.h"

static const char *TAG = "ECU";
static twai_node_handle_t s_twai_node;

/* -------------------------------------------------------------------------
 * Little-endian encode helpers. RFC-0001 §8 mandates little-endian on the
 * wire, which is also ESP32's native byte order, so these are plain memcpy
 * — no byte-swap routine needed.
 * ---------------------------------------------------------------------- */
static inline void put_u8(uint8_t *buf, size_t off, uint8_t v)
{
  buf[off] = v;
}
static inline void put_u16(uint8_t *buf, size_t off, uint16_t v)
{
  memcpy(&buf[off], &v, sizeof(v));
}
static inline void put_i16(uint8_t *buf, size_t off, int16_t v)
{
  memcpy(&buf[off], &v, sizeof(v));
}
static inline void put_u32(uint8_t *buf, size_t off, uint32_t v)
{
  memcpy(&buf[off], &v, sizeof(v));
}
static inline void put_i32(uint8_t *buf, size_t off, int32_t v)
{
  memcpy(&buf[off], &v, sizeof(v));
}

/* -------------------------------------------------------------------------
 * Simulated vehicle state — one field per DBC signal, updated in place by
 * each message's encode_*() function every time it fires.
 * ---------------------------------------------------------------------- */
typedef struct
{
  float wheel_fl_kmh, wheel_fr_kmh, wheel_rl_kmh, wheel_rr_kmh;
  float steering_deg, g_lat, g_long, ground_speed_kmh;
  float brake_front_bar, brake_rear_bar;
  float rpm, throttle_pct, lambda, map_kpa;
  uint8_t gear;
  float coolant_c, oil_temp_c, oil_pressure_bar, fuel_line_bar;
  float mat_c;
  uint32_t fuel_used_raw;
  float egt1_c, egt2_c, egt3_c;
  /* double, not float: at 1e-7 deg/LSB resolution, a magnitude-~50 degree
   * coordinate needs ~9 significant decimal digits — beyond float32's ~7
   * digits of precision, which would round-trip incorrectly through the
   * random walk below. Cheap on ESP32 (has FPU support for double), and
   * this runs at only 10Hz. */
  double gps_lat, gps_lon;
  float gps_alt_m, gps_speed_kmh;
} sim_state_t;

static void sim_state_init(sim_state_t *s)
{
  *s = (sim_state_t){
      .wheel_fl_kmh = 80, .wheel_fr_kmh = 80, .wheel_rl_kmh = 80, .wheel_rr_kmh = 80,
      .steering_deg = 0, .g_lat = 0, .g_long = 0, .ground_speed_kmh = 80,
      .brake_front_bar = 5, .brake_rear_bar = 4,
      .rpm = 4500, .throttle_pct = 30, .lambda = 1.0f, .map_kpa = 100,
      .gear = 3,
      .coolant_c = 90, .oil_temp_c = 100, .oil_pressure_bar = 4, .fuel_line_bar = 4,
      .mat_c = 35, .fuel_used_raw = 0,
      .egt1_c = 750, .egt2_c = 750, .egt3_c = 750,
      /* Arbitrary placeholder reference point — not any specific real
       * track — the walk below just wanders within ~1km of it. */
      .gps_lat = -23.5505, .gps_lon = -46.6333,
      .gps_alt_m = 760, .gps_speed_kmh = 80,
  };
}

/* Same random walk as random_walk_f but for double, used only by
 * lat/lon (see the precision comment on sim_state_t.gps_lat). */
static double random_walk_d(double value, double step, double min, double max)
{
  double delta = ((double)esp_random() / (double)UINT32_MAX * 2.0 - 1.0) * step;
  value += delta;
  if (value < min)
  {
    value = min;
  }
  if (value > max)
  {
    value = max;
  }
  return value;
}

/* Bounded random walk: nudges value by up to +/-step, clamped to [min,max].
 * Used for every simulated signal except Gear (discrete state machine, see
 * encode_pe1) and FuelUsedRaw (monotonic counter, see encode_pe3). */
static float random_walk_f(float value, float step, float min, float max)
{
  float delta = ((float)esp_random() / (float)UINT32_MAX * 2.0f - 1.0f) * step;
  value += delta;
  if (value < min)
  {
    value = min;
  }
  if (value > max)
  {
    value = max;
  }
  return value;
}

/* -------------------------------------------------------------------------
 * Per-message encoders. Each applies the random walk / state update for its
 * signals, then packs them at the byte offsets and scales defined in
 * docs/architecture/zc2x-can2.dbc.
 * ---------------------------------------------------------------------- */
static void encode_cd1(uint8_t *buf, sim_state_t *s)
{
  s->wheel_fl_kmh = random_walk_f(s->wheel_fl_kmh, 2.0f, 0, 260);
  s->wheel_fr_kmh = random_walk_f(s->wheel_fr_kmh, 2.0f, 0, 260);
  s->wheel_rl_kmh = random_walk_f(s->wheel_rl_kmh, 2.0f, 0, 260);
  s->wheel_rr_kmh = random_walk_f(s->wheel_rr_kmh, 2.0f, 0, 260);
  put_u16(buf, 0, (uint16_t)(s->wheel_fl_kmh / 0.1f));
  put_u16(buf, 2, (uint16_t)(s->wheel_fr_kmh / 0.1f));
  put_u16(buf, 4, (uint16_t)(s->wheel_rl_kmh / 0.1f));
  put_u16(buf, 6, (uint16_t)(s->wheel_rr_kmh / 0.1f));
}

static void encode_cd2(uint8_t *buf, sim_state_t *s)
{
  s->steering_deg = random_walk_f(s->steering_deg, 8.0f, -450, 450);
  s->g_lat = random_walk_f(s->g_lat, 0.05f, -2.0f, 2.0f);
  s->g_long = random_walk_f(s->g_long, 0.05f, -2.0f, 2.0f);
  s->ground_speed_kmh = random_walk_f(s->ground_speed_kmh, 2.0f, 0, 260);
  put_i16(buf, 0, (int16_t)(s->steering_deg / 0.1f));
  put_i16(buf, 2, (int16_t)(s->g_lat / 0.001f));
  put_i16(buf, 4, (int16_t)(s->g_long / 0.001f));
  put_u16(buf, 6, (uint16_t)(s->ground_speed_kmh / 0.1f));
}

static void encode_cd3(uint8_t *buf, sim_state_t *s)
{
  s->brake_front_bar = random_walk_f(s->brake_front_bar, 3.0f, 0, 150);
  s->brake_rear_bar = random_walk_f(s->brake_rear_bar, 2.5f, 0, 130);
  put_u16(buf, 0, (uint16_t)(s->brake_front_bar / 0.1f));
  put_u16(buf, 2, (uint16_t)(s->brake_rear_bar / 0.1f));
}

static void encode_pe1(uint8_t *buf, sim_state_t *s)
{
  s->rpm = random_walk_f(s->rpm, 150.0f, 800, 14000);
  s->throttle_pct = random_walk_f(s->throttle_pct, 5.0f, 0, 100);
  s->lambda = random_walk_f(s->lambda, 0.01f, 0.75f, 1.2f);
  s->map_kpa = random_walk_f(s->map_kpa, 5.0f, 30, 250);

  /* ~1% chance per tick (50 Hz) of shifting one gear up or down, clamped to
   * [0,6] — never emits 255 ("unknown"), which is a real-hardware-only
   * sentinel (see the DBC's CM_ SG_ comment on Gear). */
  if ((esp_random() % 100U) < 1U)
  {
    int8_t delta = (esp_random() & 1U) ? 1 : -1;
    int8_t next = (int8_t)s->gear + delta;
    if (next < 0)
    {
      next = 0;
    }
    if (next > 6)
    {
      next = 6;
    }
    s->gear = (uint8_t)next;
  }

  /* Monotonic accumulator: more RPM+throttle -> more fuel consumed. The
   * constant is arbitrary — only monotonicity and rough correlation with
   * engine load matter for a simulator. */
  s->fuel_used_raw += (uint32_t)(s->rpm * s->throttle_pct / 100.0f * 0.02f);

  put_u16(buf, 0, (uint16_t)s->rpm);
  put_u8(buf, 2, (uint8_t)s->throttle_pct);
  put_u16(buf, 3, (uint16_t)(s->lambda / 0.001f));
  put_u16(buf, 5, (uint16_t)(s->map_kpa / 0.1f));
  put_u8(buf, 7, s->gear);
}

/* Engine temperatures + oil pressure — all thermally-slow signals grouped
 * together (coolant, oil, intake air temp) regardless of exact subsystem,
 * per the DBC's PE2 comment. Deliberately does NOT include fuel line
 * pressure — see encode_pe3. */
static void encode_pe2(uint8_t *buf, sim_state_t *s)
{
  s->coolant_c = random_walk_f(s->coolant_c, 0.3f, 70, 110);
  s->oil_temp_c = random_walk_f(s->oil_temp_c, 0.3f, 80, 130);
  s->mat_c = random_walk_f(s->mat_c, 0.2f, 15, 65);
  s->oil_pressure_bar = random_walk_f(s->oil_pressure_bar, 0.1f, 1.5f, 6.0f);
  put_i16(buf, 0, (int16_t)(s->coolant_c / 0.1f));
  put_i16(buf, 2, (int16_t)(s->oil_temp_c / 0.1f));
  put_i16(buf, 4, (int16_t)(s->mat_c / 0.1f));
  put_u16(buf, 6, (uint16_t)(s->oil_pressure_bar / 0.1f));
}

/* Fuel system only, kept separate from PE2's engine temps for a cleaner
 * engine-vs-fuel split (see the DBC's PE3 comment). */
static void encode_pe3(uint8_t *buf, sim_state_t *s)
{
  s->fuel_line_bar = random_walk_f(s->fuel_line_bar, 0.1f, 3.0f, 5.0f);
  put_u16(buf, 0, (uint16_t)(s->fuel_line_bar / 0.1f));
  put_u32(buf, 2, s->fuel_used_raw);
}

static void encode_pe4(uint8_t *buf, sim_state_t *s)
{
  s->egt1_c = random_walk_f(s->egt1_c, 5.0f, 500, 950);
  s->egt2_c = random_walk_f(s->egt2_c, 5.0f, 500, 950);
  s->egt3_c = random_walk_f(s->egt3_c, 5.0f, 500, 950);
  put_i16(buf, 0, (int16_t)s->egt1_c);
  put_i16(buf, 2, (int16_t)s->egt2_c);
  put_i16(buf, 4, (int16_t)s->egt3_c);
}

/* Step ~0.00002 deg/call (~2.2m at this call rate of once per 10 ticks =
 * 100ms, i.e. ~22 m/s / ~79 km/h) wandering within ~1km of the placeholder
 * reference point set in sim_state_init — a plausible-looking track loop,
 * not a real one. */
static void encode_gps1(uint8_t *buf, sim_state_t *s)
{
  s->gps_lat = random_walk_d(s->gps_lat, 0.00002, -23.5605, -23.5405);
  s->gps_lon = random_walk_d(s->gps_lon, 0.00002, -46.6433, -46.6233);
  put_i32(buf, 0, (int32_t)(s->gps_lat / 0.0000001));
  put_i32(buf, 4, (int32_t)(s->gps_lon / 0.0000001));
}

static void encode_gps2(uint8_t *buf, sim_state_t *s)
{
  s->gps_alt_m = random_walk_f(s->gps_alt_m, 0.5f, 710, 810);
  s->gps_speed_kmh = random_walk_f(s->gps_speed_kmh, 2.0f, 0, 260);
  put_i16(buf, 0, (int16_t)(s->gps_alt_m / 0.1f));
  put_u16(buf, 2, (uint16_t)(s->gps_speed_kmh / 0.1f));
}

/* -------------------------------------------------------------------------
 * Table-driven scheduler — one task, one ECU_CAN_TICK_MS tick, each message
 * fires when the tick count is a multiple of its period_ticks.
 * ---------------------------------------------------------------------- */
typedef void (*encode_fn_t)(uint8_t *buf, sim_state_t *state);

typedef struct
{
  uint32_t can_id;
  uint8_t dlc;
  uint32_t period_ticks;
  encode_fn_t encode;
  const char *name;
} can_msg_def_t;

static const can_msg_def_t s_messages[] = {
    {ECU_CAN_ID_CD1, 8, ECU_CAN_PERIOD_CD1_MS / ECU_CAN_TICK_MS,
     encode_cd1, "CD1"},
    {ECU_CAN_ID_CD2, 8, ECU_CAN_PERIOD_CD2_MS / ECU_CAN_TICK_MS,
     encode_cd2, "CD2"},
    {ECU_CAN_ID_CD3, 4, ECU_CAN_PERIOD_CD3_MS / ECU_CAN_TICK_MS,
     encode_cd3, "CD3"},
    {ECU_CAN_ID_PE1, 8, ECU_CAN_PERIOD_PE1_MS / ECU_CAN_TICK_MS,
     encode_pe1, "PE1"},
    {ECU_CAN_ID_PE2, 8, ECU_CAN_PERIOD_PE2_MS / ECU_CAN_TICK_MS,
     encode_pe2, "PE2"},
    {ECU_CAN_ID_PE3, 6, ECU_CAN_PERIOD_PE3_MS / ECU_CAN_TICK_MS,
     encode_pe3, "PE3"},
    {ECU_CAN_ID_PE4, 6, ECU_CAN_PERIOD_PE4_MS / ECU_CAN_TICK_MS,
     encode_pe4, "PE4"},
    {ECU_CAN_ID_GPS1, 8, ECU_CAN_PERIOD_GPS1_MS / ECU_CAN_TICK_MS,
     encode_gps1, "GPS1"},
    {ECU_CAN_ID_GPS2, 4, ECU_CAN_PERIOD_GPS2_MS / ECU_CAN_TICK_MS,
     encode_gps2, "GPS2"},
};
#define NUM_MESSAGES (sizeof(s_messages) / sizeof(s_messages[0]))

/* Per-message persistent storage for the payload bytes and twai_frame_t
 * passed to twai_node_transmit(). This MUST NOT be stack-local / reused
 * across messages: the underlying driver (_node_queue_tx() in
 * esp_twai_onchip.c) does not copy the frame or its buffer — it queues the
 * *pointer* (twai_ctx->p_curr_tx = frame; xQueueSend(..., &frame, ...)) for
 * the hardware/ISR to read later, and returns ESP_OK as soon as the pointer
 * is queued, not once the frame is actually transmitted. A stack-local
 * buffer reused on the very next loop iteration gets overwritten with the
 * next message's data before the hardware ever reads it — confirmed as the
 * root cause of a real regression: ECU logged zero transmit failures (queue
 * acceptance always succeeds) while no receiver ever saw a coherent frame,
 * because firing multiple 100Hz messages back-to-back within one 10ms tick
 * left no gap before the shared stack slot got reused. One buffer/frame per
 * message index, indexed the same as s_messages[], keeps each message's
 * memory untouched until that same message is due again (>=10ms later —
 * always more than enough for the queue to drain at 1Mbps). */
static uint8_t s_payload_buffers[NUM_MESSAGES][8];
static twai_frame_t s_frames[NUM_MESSAGES];

static void can_init(void)
{
  const twai_onchip_node_config_t node_cfg = {
      .io_cfg = {
          .tx = 1,
          .rx = 21,
          // .tx = ECU_CAN_TX_PIN,
          // .rx = ECU_CAN_RX_PIN,
          .quanta_clk_out = GPIO_NUM_NC,
          .bus_off_indicator = GPIO_NUM_NC,
      },
      .bit_timing = {
          .bitrate = ECU_CAN_BITRATE,
      },
      .tx_queue_depth = ECU_CAN_TX_QUEUE_DEPTH,
      .flags = {
          /* ECU is a bench-test simulator: on a bus with no other node,
           * ECU's own frames would get an ACK error on every transmission
           * and drive it into bus-off after ~30 frames. enable_self_test
           * makes ECU satisfy its own ACK requirement, which is exactly
           * what it's for ("Use this mode for self testing" —
           * esp_twai_onchip.h). Kept on regardless of OBU's own CAN mode
           * (OBU runs Normal/ACKing today — see obu/main/main.c can_init())
           * so this simulator stays robust even bench-tested alone. Real
           * vehicle ECUs (e.g. the MoTeC M180 this simulates) don't need
           * this since a live bus always has other nodes to ACK. */
          .enable_self_test = 1,
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
  static sim_state_t state;
  sim_state_init(&state);
  uint32_t tick = 0;
  uint32_t sent_since_log = 0;
  uint32_t failed_since_log = 0;
  int64_t last_log_us = esp_timer_get_time();

  while (1)
  {
    for (size_t i = 0; i < NUM_MESSAGES; i++)
    {
      const can_msg_def_t *m = &s_messages[i];
      if (tick % m->period_ticks != 0U)
      {
        continue;
      }

      /* Persistent per-message storage — see the comment on
       * s_payload_buffers/s_frames above for why this can't be stack-local. */
      uint8_t *payload = s_payload_buffers[i];
      m->encode(payload, &state);

      s_frames[i] = (twai_frame_t){
          .header = {
              .id = m->can_id,
              .ide = 0,
              .rtr = 0,
              .dlc = m->dlc,
          },
          .buffer = payload,
          .buffer_len = m->dlc,
      };

      esp_err_t err = twai_node_transmit(s_twai_node, &s_frames[i], pdMS_TO_TICKS(100));
      if (err != ESP_OK)
      {
        failed_since_log++;
        ESP_LOGW(TAG, "CAN tx 0x%03lx (%s) failed (err=0x%x)",
                 (unsigned long)m->can_id, m->name, (unsigned)err);
      }
      else
      {
        sent_since_log++;
      }
    }

    /* Periodic summary instead of per-frame logging (which would flood the
     * console at up to 300 msgs/sec — see OBU_STATS_LOG_INTERVAL_MS for the
     * same reasoning elsewhere in this repo). Also closes a real
     * debuggability gap: with no periodic log at all, ECU running correctly
     * and ECU stuck/silent looked identical from the outside — exactly the
     * ambiguity that made an earlier real bug here harder to pin down. */
    int64_t now = esp_timer_get_time();
    if (now - last_log_us >= (int64_t)ECU_STATS_LOG_INTERVAL_MS * 1000)
    {
      ESP_LOGI(TAG, "stats: %lu CAN tx queued, %lu tx failures in last %d ms",
               (unsigned long)sent_since_log, (unsigned long)failed_since_log,
               ECU_STATS_LOG_INTERVAL_MS);
      sent_since_log = 0;
      failed_since_log = 0;
      last_log_us = now;
    }

    tick++;
    vTaskDelay(pdMS_TO_TICKS(ECU_CAN_TICK_MS));
  }
}

void app_main(void)
{
  ESP_LOGI(TAG, "ZC2X ECU simulator starting (%u simulated CAN messages)",
           (unsigned)NUM_MESSAGES);

  for (size_t i = 0; i < NUM_MESSAGES; i++)
  {
    assert(s_messages[i].dlc <= 8U);
    assert(s_messages[i].period_ticks > 0U);
  }

  can_init();

  xTaskCreate(can_tx_task, "can_tx_task", ECU_CAN_TX_TASK_STACK, NULL,
              configMAX_PRIORITIES - 1, NULL);
}
