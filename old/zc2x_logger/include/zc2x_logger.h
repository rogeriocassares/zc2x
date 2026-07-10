#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

  void zc2x_logger_init(void);

  void zc2x_log_info(const char *msg);

  void zc2x_log_warn(const char *msg);

  void zc2x_log_error(const char *msg);

#ifdef __cplusplus
}
#endif