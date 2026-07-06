#include "zc2x_config.h"

static zc2x_config_t g_config;

void zc2x_config_init(
    zc2x_role_t role,
    uint32_t device_id,
    const char *device_name)
{
  g_config.role = role;

  g_config.device_id = device_id;

  g_config.device_name = device_name;
}

const zc2x_config_t *zc2x_config_get(void)
{
  return &g_config;
}

zc2x_role_t zc2x_config_get_role(void)
{
  return g_config.role;
}