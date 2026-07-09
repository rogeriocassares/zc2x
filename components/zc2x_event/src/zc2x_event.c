#include "zc2x_event.h"

#include <string.h>

static uint32_t g_sequence = 0;

void zc2x_event_init(
    zc2x_event_t *event,
    zc2x_event_type_t type)
{
  memset(event, 0, sizeof(*event));

  event->type = type;

  event->sequence = ++g_sequence;
}