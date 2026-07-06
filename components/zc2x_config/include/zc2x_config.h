#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    ZC2X_ROLE_OBU = 0,
    ZC2X_ROLE_RSU

  } zc2x_role_t;

  typedef struct
  {
    zc2x_role_t role;

    uint32_t device_id;

    const char *device_name;

  } zc2x_config_t;

  void zc2x_config_init(
      zc2x_role_t role,
      uint32_t device_id,
      const char *device_name);

  const zc2x_config_t *zc2x_config_get(void);

  zc2x_role_t zc2x_config_get_role(void);

#ifdef __cplusplus
}
#endif