#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "device_config.h"
#include "zc2x_core.h"
#include "zc2x_logger.h"
#include "zc2x_config.h"

void app_main(void)
{
  zc2x_core_init();

  zc2x_logger_init();

  zc2x_config_init(
      ZC2X_DEVICE_ROLE,
      ZC2X_DEVICE_ID,
      ZC2X_DEVICE_NAME);

  const zc2x_config_t *cfg = zc2x_config_get();

  printf("Device ID   : %lu\n", (unsigned long)cfg->device_id);

  printf("Device Name : %s\n", cfg->device_name);

  printf("Role        : %d\n", cfg->role);

  while (true)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}