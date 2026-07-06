#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
  printf("\n");
  printf("=========================\n");
  printf(" ZC2X OBU\n");
  printf(" Version 0.0.1\n");
  printf("=========================\n");

  while (1)
  {
    printf("RSU Alive\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}