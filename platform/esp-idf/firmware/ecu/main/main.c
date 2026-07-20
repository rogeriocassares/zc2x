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
#include <stdbool.h>
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
  /* Tire surface temperature, 3-point IR array per corner (inner/middle/
   * outer across the tread) — see docs/architecture/zc2x-can2.dbc's BO_ 304
   * comment for the MLX90614 sensor and encoding this mirrors. */
  float ttfl_inner_c, ttfl_middle_c, ttfl_outer_c;
  float ttfr_inner_c, ttfr_middle_c, ttfr_outer_c;
  float ttrl_inner_c, ttrl_middle_c, ttrl_outer_c;
  float ttrr_inner_c, ttrr_middle_c, ttrr_outer_c;
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

  /* PDM30/M130/C125/L180 custom re-aggregation — see the DBC's CM_ BU_
   * comments for each device (docs/architecture/zc2x-can2.dbc) for why
   * these are a custom layout, not a mirror of any device's native CAN
   * broadcast. Per-output arrays (not individually-named fields) since 30
   * outputs is genuinely homogeneous, indexed data — unlike e.g. the TTU
   * corners, which are semantically distinct one-offs worth naming. */
  float pd_battery_v, pd_total_current_a, pd_internal_temp_c;
  uint8_t pd_error_flags;
  float pd_output_current_a[30];
  float pd_output_voltage_v[30];
  bool pd_output_status[30];
  bool pd_input_state[16];
  float pd_input_voltage_v[16];

  float ec_lambda2, ec_ignition_timing_deg, ec_battery_v, ec_baro_kpa;
  float ec_knock_pct, ec_injector_duty_pct, ec_cam_intake_deg, ec_cam_exhaust_deg;

  float da_g_lat, da_g_long, da_g_vert;
  float da_temp_c, da_supply_v, da_battery_v;

  float lg_g_lat, lg_g_long, lg_g_vert;
} sim_state_t;

static void sim_state_init(sim_state_t *s)
{
  *s = (sim_state_t){
      .wheel_fl_kmh = 80, .wheel_fr_kmh = 80, .wheel_rl_kmh = 80, .wheel_rr_kmh = 80,
      .steering_deg = 0, .g_lat = 0, .g_long = 0, .ground_speed_kmh = 80,
      .brake_front_bar = 5, .brake_rear_bar = 4,
      /* All three zones start equal — see encode_ttufl for why they're left
       * to diverge only via independent random walk, not a forced profile. */
      .ttfl_inner_c = 85, .ttfl_middle_c = 85, .ttfl_outer_c = 85,
      .ttfr_inner_c = 85, .ttfr_middle_c = 85, .ttfr_outer_c = 85,
      .ttrl_inner_c = 85, .ttrl_middle_c = 85, .ttrl_outer_c = 85,
      .ttrr_inner_c = 85, .ttrr_middle_c = 85, .ttrr_outer_c = 85,
      .rpm = 4500, .throttle_pct = 30, .lambda = 1.0f, .map_kpa = 100,
      .gear = 3,
      .coolant_c = 90, .oil_temp_c = 100, .oil_pressure_bar = 4, .fuel_line_bar = 4,
      .mat_c = 35, .fuel_used_raw = 0,
      .egt1_c = 750, .egt2_c = 750, .egt3_c = 750,
      /* Arbitrary placeholder reference point — not any specific real
       * track — the walk below just wanders within ~1km of it. */
      .gps_lat = -23.5505, .gps_lon = -46.6333,
      .gps_alt_m = 760, .gps_speed_kmh = 80,

      .pd_battery_v = 13.2f, .pd_internal_temp_c = 45,
      .ec_lambda2 = 1.0f, .ec_ignition_timing_deg = 20, .ec_battery_v = 13.2f, .ec_baro_kpa = 101.3f,
      .ec_injector_duty_pct = 30,
      .da_g_vert = 1.0f, .da_temp_c = 35, .da_supply_v = 5.0f, .da_battery_v = 13.2f,
      .lg_g_vert = 1.0f,
  };

  /* Roughly a third of the PDM's outputs/inputs start "on" — arbitrary, just
   * enough variety that OutputStatus/InputState aren't all-zero at boot. */
  for (size_t i = 0; i < 30; i++)
  {
    s->pd_output_status[i] = (i % 3) == 0;
    s->pd_output_current_a[i] = s->pd_output_status[i] ? 2.0f : 0.0f;
    s->pd_output_voltage_v[i] = s->pd_output_status[i] ? 13.0f : 0.0f;
  }
  for (size_t i = 0; i < 16; i++)
  {
    s->pd_input_state[i] = (i % 3) == 0;
    s->pd_input_voltage_v[i] = s->pd_input_state[i] ? 5.0f : 0.0f;
  }
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
  put_u16(buf, 0, (uint16_t)((s->steering_deg + 720.0f) / 0.1f));
  put_u16(buf, 2, (uint16_t)((s->g_lat + 5.0f) / 0.001f));
  put_u16(buf, 4, (uint16_t)((s->g_long + 5.0f) / 0.001f));
  put_u16(buf, 6, (uint16_t)(s->ground_speed_kmh / 0.1f));
}

static void encode_cd3(uint8_t *buf, sim_state_t *s)
{
  s->brake_front_bar = random_walk_f(s->brake_front_bar, 3.0f, 0, 150);
  s->brake_rear_bar = random_walk_f(s->brake_rear_bar, 2.5f, 0, 130);
  put_u16(buf, 0, (uint16_t)(s->brake_front_bar / 0.1f));
  put_u16(buf, 2, (uint16_t)(s->brake_rear_bar / 0.1f));
}

/* Tire surface temperature, one message per corner (see the DBC's BO_ 304
 * comment for why: 4 physically independent TTU transmitters in production,
 * not 4 signals aggregated by one ECU like CD1's wheel speeds). All three
 * zones (inner/middle/outer) start equal and wander independently — this
 * simulator doesn't model tread-position or cross-corner thermal physics,
 * only plausible bounded motion, same as CD1's four wheel speeds. */
static void encode_ttufl(uint8_t *buf, sim_state_t *s)
{
  s->ttfl_inner_c = random_walk_f(s->ttfl_inner_c, 1.0f, 40, 140);
  s->ttfl_middle_c = random_walk_f(s->ttfl_middle_c, 1.0f, 40, 140);
  s->ttfl_outer_c = random_walk_f(s->ttfl_outer_c, 1.0f, 40, 140);
  put_u16(buf, 0, (uint16_t)((s->ttfl_inner_c + 40.0f) / 0.1f));
  put_u16(buf, 2, (uint16_t)((s->ttfl_middle_c + 40.0f) / 0.1f));
  put_u16(buf, 4, (uint16_t)((s->ttfl_outer_c + 40.0f) / 0.1f));
}

static void encode_ttufr(uint8_t *buf, sim_state_t *s)
{
  s->ttfr_inner_c = random_walk_f(s->ttfr_inner_c, 1.0f, 40, 140);
  s->ttfr_middle_c = random_walk_f(s->ttfr_middle_c, 1.0f, 40, 140);
  s->ttfr_outer_c = random_walk_f(s->ttfr_outer_c, 1.0f, 40, 140);
  put_u16(buf, 0, (uint16_t)((s->ttfr_inner_c + 40.0f) / 0.1f));
  put_u16(buf, 2, (uint16_t)((s->ttfr_middle_c + 40.0f) / 0.1f));
  put_u16(buf, 4, (uint16_t)((s->ttfr_outer_c + 40.0f) / 0.1f));
}

static void encode_tturl(uint8_t *buf, sim_state_t *s)
{
  s->ttrl_inner_c = random_walk_f(s->ttrl_inner_c, 1.0f, 40, 140);
  s->ttrl_middle_c = random_walk_f(s->ttrl_middle_c, 1.0f, 40, 140);
  s->ttrl_outer_c = random_walk_f(s->ttrl_outer_c, 1.0f, 40, 140);
  put_u16(buf, 0, (uint16_t)((s->ttrl_inner_c + 40.0f) / 0.1f));
  put_u16(buf, 2, (uint16_t)((s->ttrl_middle_c + 40.0f) / 0.1f));
  put_u16(buf, 4, (uint16_t)((s->ttrl_outer_c + 40.0f) / 0.1f));
}

static void encode_tturr(uint8_t *buf, sim_state_t *s)
{
  s->ttrr_inner_c = random_walk_f(s->ttrr_inner_c, 1.0f, 40, 140);
  s->ttrr_middle_c = random_walk_f(s->ttrr_middle_c, 1.0f, 40, 140);
  s->ttrr_outer_c = random_walk_f(s->ttrr_outer_c, 1.0f, 40, 140);
  put_u16(buf, 0, (uint16_t)((s->ttrr_inner_c + 40.0f) / 0.1f));
  put_u16(buf, 2, (uint16_t)((s->ttrr_middle_c + 40.0f) / 0.1f));
  put_u16(buf, 4, (uint16_t)((s->ttrr_outer_c + 40.0f) / 0.1f));
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
  put_u16(buf, 3, (uint16_t)((s->lambda - 0.5f) / 0.001f));
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
  put_u16(buf, 0, (uint16_t)((s->coolant_c + 40.0f) / 0.1f));
  put_u16(buf, 2, (uint16_t)((s->oil_temp_c + 40.0f) / 0.1f));
  put_u16(buf, 4, (uint16_t)((s->mat_c + 40.0f) / 0.1f));
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
  put_u16(buf, 0, (uint16_t)s->egt1_c);
  put_u16(buf, 2, (uint16_t)s->egt2_c);
  put_u16(buf, 4, (uint16_t)s->egt3_c);
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
  put_u16(buf, 0, (uint16_t)((s->gps_alt_m + 1000.0f) / 0.1f));
  put_u16(buf, 2, (uint16_t)(s->gps_speed_kmh / 0.1f));
}

/* -------------------------------------------------------------------------
 * PDM30/M130/C125/L180 custom re-aggregation — see the DBC's CM_ BU_
 * comments for each device (docs/architecture/zc2x-can2.dbc) for why these
 * are a custom layout, not a mirror of any device's native CAN broadcast.
 * ---------------------------------------------------------------------- */
static void encode_pd1(uint8_t *buf, sim_state_t *s)
{
  s->pd_battery_v = random_walk_f(s->pd_battery_v, 0.05f, 11, 15);
  s->pd_internal_temp_c = random_walk_f(s->pd_internal_temp_c, 0.5f, 20, 80);

  float total = 0;
  for (size_t i = 0; i < 30; i++)
  {
    total += s->pd_output_current_a[i];
  }
  s->pd_total_current_a = total;

  put_u16(buf, 0, (uint16_t)(s->pd_battery_v / 0.01f));
  put_u16(buf, 2, (uint16_t)(s->pd_total_current_a / 0.1f));
  put_u16(buf, 4, (uint16_t)((s->pd_internal_temp_c + 40.0f) / 0.1f));
  put_u8(buf, 6, s->pd_error_flags); /* always 0 in this simulator — no fault to report */
}

/* Encodes up to 8 consecutive elements of a 0.1-unit-per-LSB, unsigned
 * 8-bit array bank, starting at `start`. Shared by PD2-PD9 (output
 * current/voltage) — the only difference between those 8 messages is
 * which 30-element array and which slice of it, so one function replaces
 * what would otherwise be 8 near-identical hand-written loops. */
static void encode_pd_bank_u8(uint8_t *buf, const float *values, size_t start, size_t n)
{
  for (size_t i = 0; i < n; i++)
  {
    buf[i] = (uint8_t)(values[start + i] / 0.1f);
  }
}

/* Updates the full 30-element output-current array once per tick. Called
 * only from encode_pd2 (the first of PD2-PD5, all sharing one period) so
 * the walk happens exactly once per cycle regardless of how many messages
 * read slices of the result within that same cycle. */
static void encode_pd_currents(sim_state_t *s)
{
  for (size_t i = 0; i < 30; i++)
  {
    s->pd_output_current_a[i] = random_walk_f(s->pd_output_current_a[i], 0.3f, 0, 20);
  }
}

/* Same idea as encode_pd_currents, for output voltage — called only from
 * encode_pd6 (the first of PD6-PD9). */
static void encode_pd_voltages(sim_state_t *s)
{
  for (size_t i = 0; i < 30; i++)
  {
    s->pd_output_voltage_v[i] = random_walk_f(s->pd_output_voltage_v[i], 0.3f, 0, 14);
  }
}

static void encode_pd2(uint8_t *buf, sim_state_t *s)
{
  encode_pd_currents(s);
  encode_pd_bank_u8(buf, s->pd_output_current_a, 0, 8);
}
static void encode_pd3(uint8_t *buf, sim_state_t *s) { encode_pd_bank_u8(buf, s->pd_output_current_a, 8, 8); }
static void encode_pd4(uint8_t *buf, sim_state_t *s) { encode_pd_bank_u8(buf, s->pd_output_current_a, 16, 8); }
static void encode_pd5(uint8_t *buf, sim_state_t *s) { encode_pd_bank_u8(buf, s->pd_output_current_a, 24, 6); }

static void encode_pd6(uint8_t *buf, sim_state_t *s)
{
  encode_pd_voltages(s);
  encode_pd_bank_u8(buf, s->pd_output_voltage_v, 0, 8);
}
static void encode_pd7(uint8_t *buf, sim_state_t *s) { encode_pd_bank_u8(buf, s->pd_output_voltage_v, 8, 8); }
static void encode_pd8(uint8_t *buf, sim_state_t *s) { encode_pd_bank_u8(buf, s->pd_output_voltage_v, 16, 8); }
static void encode_pd9(uint8_t *buf, sim_state_t *s) { encode_pd_bank_u8(buf, s->pd_output_voltage_v, 24, 6); }

/* Packs up to 32 booleans into a little-endian bitfield starting at byte 0
 * of buf, one bit per value. Shared by PD10 (30 output statuses) and PD11
 * (16 input states) — matches MoTeC's own real PDM DBC convention of
 * packing many 1-bit flags into a single frame (confirmed via a genuine
 * PDM15 CAN-import screenshot) rather than one message per flag. */
static void encode_pd_bits(uint8_t *buf, const bool *values, size_t n)
{
  size_t nbytes = (n + 7) / 8;
  memset(buf, 0, nbytes);
  for (size_t i = 0; i < n; i++)
  {
    if (values[i])
    {
      buf[i / 8] |= (uint8_t)(1U << (i % 8));
    }
  }
}

static void encode_pd10(uint8_t *buf, sim_state_t *s)
{
  /* ~2% chance per tick of one output flipping on/off — same "rare state
   * change" idiom as encode_pe1's gear shifts, applied per-output. */
  if ((esp_random() % 100U) < 2U)
  {
    size_t idx = esp_random() % 30U;
    s->pd_output_status[idx] = !s->pd_output_status[idx];
  }
  encode_pd_bits(buf, s->pd_output_status, 30);
}

static void encode_pd11(uint8_t *buf, sim_state_t *s)
{
  if ((esp_random() % 100U) < 2U)
  {
    size_t idx = esp_random() % 16U;
    s->pd_input_state[idx] = !s->pd_input_state[idx];
  }
  encode_pd_bits(buf, s->pd_input_state, 16);
}

/* Updates the full 16-element input-voltage array once per tick — called
 * only from encode_pd12 (the first of PD12-PD15), same reasoning as
 * encode_pd_currents/encode_pd_voltages above. */
static void encode_pd_input_voltages(sim_state_t *s)
{
  for (size_t i = 0; i < 16; i++)
  {
    s->pd_input_voltage_v[i] = random_walk_f(s->pd_input_voltage_v[i], 0.1f, 0, 14);
  }
}

static void encode_pd_input_voltage_bank(uint8_t *buf, const float *values, size_t start, size_t n)
{
  for (size_t i = 0; i < n; i++)
  {
    put_u16(buf, i * 2, (uint16_t)(values[start + i] / 0.01f));
  }
}

static void encode_pd12(uint8_t *buf, sim_state_t *s)
{
  encode_pd_input_voltages(s);
  encode_pd_input_voltage_bank(buf, s->pd_input_voltage_v, 0, 4);
}
static void encode_pd13(uint8_t *buf, sim_state_t *s) { encode_pd_input_voltage_bank(buf, s->pd_input_voltage_v, 4, 4); }
static void encode_pd14(uint8_t *buf, sim_state_t *s) { encode_pd_input_voltage_bank(buf, s->pd_input_voltage_v, 8, 4); }
static void encode_pd15(uint8_t *buf, sim_state_t *s) { encode_pd_input_voltage_bank(buf, s->pd_input_voltage_v, 12, 4); }

/* M130 channels beyond CD1-3/PE1-4's baseline — see the DBC's BO_ 1088/1089
 * comments for what's included and why. */
static void encode_ec1(uint8_t *buf, sim_state_t *s)
{
  s->ec_lambda2 = random_walk_f(s->ec_lambda2, 0.01f, 0.75f, 1.2f);
  s->ec_ignition_timing_deg = random_walk_f(s->ec_ignition_timing_deg, 0.5f, 0, 40);
  s->ec_battery_v = random_walk_f(s->ec_battery_v, 0.05f, 11, 15);
  s->ec_baro_kpa = random_walk_f(s->ec_baro_kpa, 0.1f, 95, 105);
  put_u16(buf, 0, (uint16_t)((s->ec_lambda2 - 0.5f) / 0.001f));
  put_u16(buf, 2, (uint16_t)((s->ec_ignition_timing_deg + 10.0f) / 0.1f));
  put_u16(buf, 4, (uint16_t)(s->ec_battery_v / 0.01f));
  put_u16(buf, 6, (uint16_t)((s->ec_baro_kpa - 80.0f) / 0.1f));
}

static void encode_ec2(uint8_t *buf, sim_state_t *s)
{
  s->ec_knock_pct = random_walk_f(s->ec_knock_pct, 1.0f, 0, 15); /* mostly low, occasional light knock */
  s->ec_injector_duty_pct = random_walk_f(s->ec_injector_duty_pct, 2.0f, 10, 90);
  s->ec_cam_intake_deg = random_walk_f(s->ec_cam_intake_deg, 0.3f, -20, 20);
  s->ec_cam_exhaust_deg = random_walk_f(s->ec_cam_exhaust_deg, 0.3f, -20, 20);
  put_u8(buf, 0, (uint8_t)s->ec_knock_pct);
  put_u8(buf, 1, (uint8_t)s->ec_injector_duty_pct);
  put_u16(buf, 2, (uint16_t)((s->ec_cam_intake_deg + 30.0f) / 0.1f));
  put_u16(buf, 4, (uint16_t)((s->ec_cam_exhaust_deg + 30.0f) / 0.1f));
}

/* C125's own internal sensors — see the DBC's BO_ 1120/1121 comments for
 * why these stay separate from CD2's/L180's G-force and PDM's/M130's
 * battery-voltage signals. */
static void encode_da1(uint8_t *buf, sim_state_t *s)
{
  s->da_g_lat = random_walk_f(s->da_g_lat, 0.05f, -2.0f, 2.0f);
  s->da_g_long = random_walk_f(s->da_g_long, 0.05f, -2.0f, 2.0f);
  s->da_g_vert = random_walk_f(s->da_g_vert, 0.02f, 0.5f, 1.5f); /* ~1g gravity baseline, road/kerb bumps */
  put_u16(buf, 0, (uint16_t)((s->da_g_lat + 5.0f) / 0.001f));
  put_u16(buf, 2, (uint16_t)((s->da_g_long + 5.0f) / 0.001f));
  put_u16(buf, 4, (uint16_t)((s->da_g_vert + 5.0f) / 0.001f));
}

static void encode_da2(uint8_t *buf, sim_state_t *s)
{
  s->da_temp_c = random_walk_f(s->da_temp_c, 0.2f, 20, 60);
  s->da_supply_v = random_walk_f(s->da_supply_v, 0.01f, 4.8f, 5.2f);
  s->da_battery_v = random_walk_f(s->da_battery_v, 0.05f, 11, 15);
  put_u16(buf, 0, (uint16_t)((s->da_temp_c + 40.0f) / 0.1f));
  put_u16(buf, 2, (uint16_t)(s->da_supply_v / 0.01f));
  put_u16(buf, 4, (uint16_t)(s->da_battery_v / 0.01f));
}

/* L180's own internal accelerometer — see the DBC's BO_ 1136 comment. */
static void encode_lg1(uint8_t *buf, sim_state_t *s)
{
  s->lg_g_lat = random_walk_f(s->lg_g_lat, 0.05f, -2.0f, 2.0f);
  s->lg_g_long = random_walk_f(s->lg_g_long, 0.05f, -2.0f, 2.0f);
  s->lg_g_vert = random_walk_f(s->lg_g_vert, 0.02f, 0.5f, 1.5f);
  put_u16(buf, 0, (uint16_t)((s->lg_g_lat + 5.0f) / 0.001f));
  put_u16(buf, 2, (uint16_t)((s->lg_g_long + 5.0f) / 0.001f));
  put_u16(buf, 4, (uint16_t)((s->lg_g_vert + 5.0f) / 0.001f));
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
    /* In production these 4 messages come from 4 physically separate TTU
     * nodes, not ECU (see the DBC's BU_ TTUFL/TTUFR/TTURL/TTURR comments) —
     * on the bench, one simulator stands in for the whole bus's traffic
     * regardless of which node would really transmit each message. */
    {ECU_CAN_ID_TTUFL, 6, ECU_CAN_PERIOD_TTUFL_MS / ECU_CAN_TICK_MS,
     encode_ttufl, "TTUFL"},
    {ECU_CAN_ID_TTUFR, 6, ECU_CAN_PERIOD_TTUFR_MS / ECU_CAN_TICK_MS,
     encode_ttufr, "TTUFR"},
    {ECU_CAN_ID_TTURL, 6, ECU_CAN_PERIOD_TTURL_MS / ECU_CAN_TICK_MS,
     encode_tturl, "TTURL"},
    {ECU_CAN_ID_TTURR, 6, ECU_CAN_PERIOD_TTURR_MS / ECU_CAN_TICK_MS,
     encode_tturr, "TTURR"},
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
    {ECU_CAN_ID_PD1, 7, ECU_CAN_PERIOD_PD1_MS / ECU_CAN_TICK_MS,
     encode_pd1, "PD1"},
    {ECU_CAN_ID_PD2, 8, ECU_CAN_PERIOD_PD_CURRENT_MS / ECU_CAN_TICK_MS,
     encode_pd2, "PD2"},
    {ECU_CAN_ID_PD3, 8, ECU_CAN_PERIOD_PD_CURRENT_MS / ECU_CAN_TICK_MS,
     encode_pd3, "PD3"},
    {ECU_CAN_ID_PD4, 8, ECU_CAN_PERIOD_PD_CURRENT_MS / ECU_CAN_TICK_MS,
     encode_pd4, "PD4"},
    {ECU_CAN_ID_PD5, 6, ECU_CAN_PERIOD_PD_CURRENT_MS / ECU_CAN_TICK_MS,
     encode_pd5, "PD5"},
    {ECU_CAN_ID_PD6, 8, ECU_CAN_PERIOD_PD_VOLTAGE_MS / ECU_CAN_TICK_MS,
     encode_pd6, "PD6"},
    {ECU_CAN_ID_PD7, 8, ECU_CAN_PERIOD_PD_VOLTAGE_MS / ECU_CAN_TICK_MS,
     encode_pd7, "PD7"},
    {ECU_CAN_ID_PD8, 8, ECU_CAN_PERIOD_PD_VOLTAGE_MS / ECU_CAN_TICK_MS,
     encode_pd8, "PD8"},
    {ECU_CAN_ID_PD9, 6, ECU_CAN_PERIOD_PD_VOLTAGE_MS / ECU_CAN_TICK_MS,
     encode_pd9, "PD9"},
    {ECU_CAN_ID_PD10, 4, ECU_CAN_PERIOD_PD_STATE_MS / ECU_CAN_TICK_MS,
     encode_pd10, "PD10"},
    {ECU_CAN_ID_PD11, 2, ECU_CAN_PERIOD_PD_STATE_MS / ECU_CAN_TICK_MS,
     encode_pd11, "PD11"},
    {ECU_CAN_ID_PD12, 8, ECU_CAN_PERIOD_PD_INPUT_VOLTAGE_MS / ECU_CAN_TICK_MS,
     encode_pd12, "PD12"},
    {ECU_CAN_ID_PD13, 8, ECU_CAN_PERIOD_PD_INPUT_VOLTAGE_MS / ECU_CAN_TICK_MS,
     encode_pd13, "PD13"},
    {ECU_CAN_ID_PD14, 8, ECU_CAN_PERIOD_PD_INPUT_VOLTAGE_MS / ECU_CAN_TICK_MS,
     encode_pd14, "PD14"},
    {ECU_CAN_ID_PD15, 8, ECU_CAN_PERIOD_PD_INPUT_VOLTAGE_MS / ECU_CAN_TICK_MS,
     encode_pd15, "PD15"},
    {ECU_CAN_ID_EC1, 8, ECU_CAN_PERIOD_EC_MS / ECU_CAN_TICK_MS,
     encode_ec1, "EC1"},
    {ECU_CAN_ID_EC2, 6, ECU_CAN_PERIOD_EC_MS / ECU_CAN_TICK_MS,
     encode_ec2, "EC2"},
    {ECU_CAN_ID_DA1, 6, ECU_CAN_PERIOD_DA1_MS / ECU_CAN_TICK_MS,
     encode_da1, "DA1"},
    {ECU_CAN_ID_DA2, 6, ECU_CAN_PERIOD_DA2_MS / ECU_CAN_TICK_MS,
     encode_da2, "DA2"},
    {ECU_CAN_ID_LG1, 6, ECU_CAN_PERIOD_LG1_MS / ECU_CAN_TICK_MS,
     encode_lg1, "LG1"},
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
