idf_component_register(
  SRCS
  "main.c"

  INCLUDE_DIRS
  "."

  REQUIRES
  zc2x_core
  zc2x_logger
  zc2x_config
)