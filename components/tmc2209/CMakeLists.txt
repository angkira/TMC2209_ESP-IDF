idf_component_register(
  SRCS "src/control.c" "src/register.c" "src/setup.c" "tmc2209.c" 
  INCLUDE_DIRS "include"
  REQUIRES "driver"
  PRIV_REQUIRES ""
)
