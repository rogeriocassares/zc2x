#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    ZC2X_EVENT_NONE = 0,

    ZC2X_EVENT_BOOT,

    ZC2X_EVENT_HEARTBEAT,

    ZC2X_EVENT_CAN_FRAME,

    ZC2X_EVENT_GNSS_FIX,

    ZC2X_EVENT_WIFI,

    ZC2X_EVENT_XBEE

  } zc2x_event_type_t;

  typedef struct
  {
    uint64_t timestamp;

    uint32_t sequence;

    uint32_t source;

    zc2x_event_type_t type;

    uint16_t flags;

  } zc2x_event_t;

  void zc2x_event_init(
      zc2x_event_t *event,
      zc2x_event_type_t type);

#ifdef __cplusplus
}
#endif