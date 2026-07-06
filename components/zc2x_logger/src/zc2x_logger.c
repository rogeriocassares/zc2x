#include "zc2x_logger.h"

#include "esp_log.h"

static const char *TAG = "ZC2X";

void zc2x_logger_init(void)
{
  ESP_LOGI(TAG, "Logger initialized");
}

void zc2x_log_info(const char *msg)
{
  ESP_LOGI(TAG, "%s", msg);
}

void zc2x_log_warn(const char *msg)
{
  ESP_LOGW(TAG, "%s", msg);
}

void zc2x_log_error(const char *msg)
{
  ESP_LOGE(TAG, "%s", msg);
}