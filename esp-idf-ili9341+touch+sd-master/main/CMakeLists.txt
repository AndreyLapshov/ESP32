idf_component_register(
  SRCS 	"main.c"      
  INCLUDE_DIRS "." 
  REQUIRES  lvgl driver 
  PRIV_REQUIRES esp_timer esp_lcd spi_flash
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_CONF_INCLUDE_SIMPLE=1)
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
