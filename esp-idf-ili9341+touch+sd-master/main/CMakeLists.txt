idf_component_register(
  SRCS 	"main.c"      
  INCLUDE_DIRS "." 
  REQUIRES  lvgl driver 
  PRIV_REQUIRES vfs spiffs esp_timer esp_lcd esp_lcd_ili9341 sdmmc fatfs vfs
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_CONF_INCLUDE_SIMPLE=1)
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
