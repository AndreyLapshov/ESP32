#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"

#include <dirent.h>    // opendir/readdir/closedir
/* ==== TFT on SPI2_HOST (existing wiring) ==== */
#define PIN_MOSI       23
#define PIN_MISO       19
#define PIN_CLK        18
#define PIN_CS_TFT      5
#define PIN_DC_TFT     27
#define PIN_RST_TFT    33
#define PIN_BK_TFT     32


/* ==== Touch on SPI3_HOST (new GPIOs) ==== */
#define PIN_MOSI_TP    13
#define PIN_MISO_TP    12
#define PIN_CLK_TP     14
#define PIN_CS_TP      15
#define PIN_IRQ_TP      4

#define PIN_CS_SD      16

#define LCD_V_OFFSET    0  // при необходимости сдвиг по Y

/* Display resolution */
#define HRES          240
#define VRES          320

/* LVGL buffer size (~10 lines) */
#define BUF_PIXELS (HRES * 40)
#define BUF_SIZE   (BUF_PIXELS * sizeof(lv_color_t))
  
#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TICK_PERIOD_US (LVGL_TICK_PERIOD_MS * 1000)


#define TAG "LVGL_DUAL_SPI"


#define MOUNT_POINT "/sdcard"
#define FILE_NAME "/sdcard/ui.json"

FILE *f = NULL;
uint32_t file_size = 0;
unsigned char* file_buf = NULL;



static lv_display_t          *disp;
static esp_lcd_panel_handle_t panel_hdl;
static esp_lcd_touch_handle_t tp_hdl;
static lv_obj_t              *label;

/* LVGL flush for TFT */
static void lvgl_flush(lv_display_t *d, const lv_area_t *area, uint8_t *color_p)
{
    esp_lcd_panel_draw_bitmap(panel_hdl,
                              area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1,
                              color_p);
}

/* TFT transfer-done */
static bool tft_done_cb(esp_lcd_panel_io_handle_t io,
                        esp_lcd_panel_io_event_data_t *edata,
                        void *user_ctx)
{
    lv_display_flush_ready((lv_display_t *)user_ctx);
    return true;
}

/* Touch read for LVGL */
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t x, y; uint8_t cnt = 0;
    esp_lcd_touch_read_data(tp_hdl);
    bool pressed = esp_lcd_touch_get_coordinates(tp_hdl, &x, &y, NULL, &cnt, 1);
    if (pressed && cnt) {
        data->state   = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/* LVGL tick (2 ms) */
static void lv_tick_cb(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

/* Button click */
static void btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        lv_label_set_text(label, "Pressed");
    }
}

/* GUI task */
static void gui_task(void *arg)
{
    while (true) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    //SD card initialization

    esp_err_t ret;

    ret = gpio_set_pull_mode(PIN_CS_SD, GPIO_PULLUP_ONLY); // CS pin for SD card
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pull mode for CS pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Pull mode set for CS pin successfully");
    }
    ret = gpio_set_direction(PIN_CS_SD, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for CS pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for CS pin successfully");
    }
    ret = gpio_set_level(PIN_CS_SD, 1); // Set CS high initially
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set level for CS pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Level set for CS pin successfully");
    }
    ret = gpio_set_direction(PIN_MOSI_TP, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for MOSI pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for MOSI pin successfully");
    }

    ret = gpio_set_direction(PIN_MISO_TP, GPIO_MODE_INPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for MISO pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for MISO pin successfully");
    }

    ret = gpio_set_direction(PIN_CLK_TP, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for CLK pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for CLK pin successfully");
    }

    ret = gpio_set_direction(PIN_CS_TP, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for CS pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for CS pin successfully");
    }

    ret = gpio_set_level(PIN_CS_TP, 1); // Set CS high initially
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set level for CS pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Level set for CS pin successfully");
    }
    ret = gpio_set_direction(PIN_IRQ_TP, GPIO_MODE_INPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for IRQ pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for IRQ pin successfully");
    }
    ret = gpio_set_pull_mode(PIN_IRQ_TP, GPIO_PULLUP_ONLY); // Set pull-up for IRQ pin
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pull mode for IRQ pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Pull mode set for IRQ pin successfully");
    }
    ret = gpio_set_pull_mode(PIN_IRQ_TP, GPIO_PULLUP_ONLY); // Pull-up for IRQ pin

    ret = gpio_set_direction(PIN_MOSI, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for MOSI pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for MOSI pin successfully");
    }
    ret = gpio_set_direction(PIN_MISO, GPIO_MODE_INPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for MISO pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for MISO pin successfully");
    }
    ret = gpio_set_direction(PIN_CLK, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for CLK pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for CLK pin successfully");
    }
    ret = gpio_set_pull_mode(PIN_MOSI, GPIO_PULLUP_ONLY); // Set pull-up for MOSI pin
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pull mode for MOSI pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Pull mode set for MOSI pin successfully");   
    }
    ret = gpio_set_pull_mode(PIN_MISO, GPIO_PULLUP_ONLY); // Set pull-up for MISO pin
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pull mode for MISO pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Pull mode set for MISO pin successfully");
    }
    ret = gpio_set_pull_mode(PIN_CLK, GPIO_PULLUP_ONLY); // Set pull-up for CLK pin
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pull mode for CLK pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Pull mode set for CLK pin successfully");
    }
    ret = gpio_set_direction(PIN_CS_TFT, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for CS pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for CS pin successfully");
    }
    ret = gpio_set_level(PIN_CS_TFT, 1); // Set CS high initially
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set level for CS pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Level set for CS pin successfully");
    }
    ret = gpio_set_direction(PIN_DC_TFT, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for DC pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for DC pin successfully");
    }
    ret = gpio_set_direction(PIN_RST_TFT, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for RST pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for RST pin successfully");
    }
    ret = gpio_set_direction(PIN_BK_TFT, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction for BK pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Direction set for BK pin successfully");
    }
    ret = gpio_set_level(PIN_BK_TFT, 0); // Set backlight on
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set level for BK pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Level set for BK pin successfully");
    }
    ret = gpio_set_level(PIN_RST_TFT, 1); // Set reset low initially
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set level for RST pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Level set for RST pin successfully");
    }
    ret = gpio_set_level(PIN_RST_TFT, 0); // Reset the TFT
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set level for RST pin: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Level set for RST pin successfully");
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset

        // 1) Монтируем SD-карту по SPI
    sdmmc_card_t *card = NULL;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = PIN_MOSI_TP,
        .miso_io_num     = PIN_MISO_TP,
        .sclk_io_num     = PIN_CLK_TP,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 8000,
    };
    sdmmc_host_t host = {
        .flags              = SDMMC_HOST_FLAG_SPI | SDMMC_HOST_FLAG_DEINIT_ARG,
        .slot               = SPI3_HOST,           // нужный SPI-хост
        .max_freq_khz       = 2000,                // 2 MHz
        .io_voltage         = 3.3f,
        .driver_strength    = SDMMC_DRIVER_STRENGTH_B,
        .current_limit      = SDMMC_CURRENT_LIMIT_200MA,
        .init               = sdspi_host_init,
        .set_card_clk       = sdspi_host_set_card_clk,
        .do_transaction     = sdspi_host_do_transaction,
        .deinit_p           = sdspi_host_remove_device,
        .io_int_enable      = sdspi_host_io_int_enable,
        .io_int_wait        = sdspi_host_io_int_wait,
        .get_real_freq      = sdspi_host_get_real_freq,
        .input_delay_phase  = SDMMC_DELAY_PHASE_0,
        .get_dma_info       = sdspi_host_get_dma_info,
        .command_timeout_ms = 0,

    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));        
    }
    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = PIN_CS_SD;
    slot_cfg.host_id = host.slot;
    esp_vfs_fat_sdmmc_mount_config_t mnt_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 5,
    };
    ret =  esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg, &mnt_cfg, &card);
    if (card) {
        if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "SD mounted, cap=%lluGB",
                    (card->csd.capacity * card->csd.sector_size) / (1024ULL*1024ULL*1024ULL*1024ULL));
                ESP_LOGI(TAG, "=== SD Card Info ===");
                sdmmc_card_print_info(stdout, card);
            }
    

        DIR *dir = opendir(MOUNT_POINT);
        if (dir) {
            struct dirent *entry;
            while ((entry = readdir(dir)) != NULL) {
                ESP_LOGI(TAG, "Found: %s", entry->d_name);
            }
            closedir(dir);
        } else {
            ESP_LOGE(TAG, "Failed to open directory: %s", MOUNT_POINT);
        }

        f = fopen(MOUNT_POINT "/ui.json", "r");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open ui.json for reading");
            //return;
        }


        // Определяем размер файла
        if (f) {
            fseek(f, 0, SEEK_END);
            long file_size = ftell(f);
            fseek(f, 0, SEEK_SET);
            ESP_LOGI(TAG, "File size: %ld bytes", file_size);

            // Выделяем буфер (+1 для нуль-терминатора)
            file_buf = malloc(file_size + 1);
            if (!file_buf) {
                ESP_LOGE(TAG, "Failed to allocate buffer (%ld bytes)", file_size + 1);
                //fclose(f);
            
            }
        

            // Читаем весь файл в буфер
            size_t read_len = fread(file_buf, 1, file_size, f);
            file_buf[read_len] = '\0';  // Нуль-терминатор
            if (read_len != file_size) {
                ESP_LOGE(TAG, "Failed to read the entire file: expected %ld bytes, got %zu bytes", file_size, read_len);
                //fclose(f);
                //free(file_buf);
            } else {
                ESP_LOGI(TAG, "Read ui.json successfully: %zu bytes", read_len);
            }
        


            ESP_LOGI(TAG, "FILE:");
            char *line = calloc(1024, sizeof(char)); // Буфер для чтения строк
            if (!line) {
                ESP_LOGE(TAG, "Failed to allocate line buffer");
                //fclose(f);
                //free(file_buf);
                //return;
            } else {
                ESP_LOGI(TAG, "Line buffer allocated successfully");
            }
            
            
            // Читаем файл построчно и выводим в лог
            // fgets сохраняет перевод строки, ESP_LOGI добавляет свой
            
            rewind(f);  // на всякий случай ставим в начало
            if (line) {
                while (fgets(line,255, f)) {
                    // fgets сохраняет перевод строки, ESP_LOGI добавляет свой
                    ESP_LOGI(TAG, "%s", line);
                }
                free(line);
            } else {
                ESP_LOGE(TAG, "Failed to read file line by line");
            }

        }

        if (file_buf)
            free(file_buf);
        // Закрываем файл и освобождаем память
        if (f) 
            fclose(f);

        // Отмонтируем файловую систему и освободим SPI-шину
        ret = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "SD card unmounted successfully");
        }

        
    }
    // //SdCard work finished 
    ret = spi_bus_free(host.slot);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to free SPI bus: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPI bus freed successfully");
    }

    /* LVGL init */
    lv_init();
    static lv_color_t *buf1 = NULL, *buf2 = NULL;
    buf1 = heap_caps_malloc(BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    buf2 = heap_caps_malloc(BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    disp = lv_display_create(HRES, VRES);
    lv_display_set_buffers(disp, buf1, buf2, BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp, lvgl_flush);

    // /* Backlight */
    gpio_config_t bk_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_BK_TFT
    };
    ret = gpio_config(&bk_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure backlight GPIO: %s", esp_err_to_name(ret));
    }else {
        ESP_LOGI(TAG, "Backlight GPIO configured successfully");
    }
    ret = gpio_set_level(PIN_BK_TFT, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set backlight GPIO level: %s", esp_err_to_name(ret));
    } else {
       ESP_LOGI(TAG, "Backlight GPIO level set successfully");
    }

    /* SPI2_HOST for TFT */
    spi_bus_config_t spi2_cfg = {
        .sclk_io_num    = PIN_CLK,
        .mosi_io_num    = PIN_MOSI,
        .miso_io_num    = PIN_MISO,
        .max_transfer_sz= BUF_SIZE
    };
    ret = spi_bus_initialize(SPI2_HOST, &spi2_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus for TFT: %s", esp_err_to_name(ret));
        
    } else {
        ESP_LOGI(TAG, "SPI bus for TFT initialized successfully");
    }

    /* SPI3_HOST for Touch */
    spi_bus_config_t spi3_cfg = {
        .sclk_io_num    = PIN_CLK_TP,
        .mosi_io_num    = PIN_MOSI_TP,
        .miso_io_num    = PIN_MISO_TP,
        .max_transfer_sz= 0
    };
    ret = spi_bus_initialize(SPI3_HOST, &spi3_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus for Touch: %s", esp_err_to_name(ret));
        
    } else {
         ESP_LOGI(TAG, "SPI bus for Touch initialized successfully");
    }

    /* TFT panel I/O */
    esp_lcd_panel_io_handle_t io_tft;
    esp_lcd_panel_io_spi_config_t tft_io_cfg = {
        .dc_gpio_num            = PIN_DC_TFT,
        .cs_gpio_num            = PIN_CS_TFT,
        .pclk_hz                = 40 * 1000 * 1000,
        .lcd_cmd_bits           = 8,
        .lcd_param_bits         = 8,
        .spi_mode               = 0,
        .trans_queue_depth      = 10,
        .on_color_trans_done    = tft_done_cb,
        .user_ctx               = disp,
     };
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,&tft_io_cfg, &io_tft);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TFT panel I/O: %s", esp_err_to_name(ret));
    
    } else {
        ESP_LOGI(TAG, "TFT panel I/O created successfully");
    }

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_RST_TFT,
        .color_space    = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = 16,
    };
    ret = esp_lcd_new_panel_ili9341(io_tft, &panel_cfg, &panel_hdl);
    if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to create TFT panel: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "TFT panel created successfully");
    }
    /* Added user snippet: */
    ret = esp_lcd_panel_reset(panel_hdl);
    if (ret != ESP_OK) {
       ESP_LOGE(TAG, "Failed to reset TFT panel: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "TFT panel reset successfully");
    }
    ret = esp_lcd_panel_init(panel_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TFT panel: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "TFT panel initialized successfully");
    }
    ret = esp_lcd_panel_invert_color(panel_hdl, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to invert TFT panel color: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "TFT panel color inversion set successfully");
    }
    ret = esp_lcd_panel_mirror(panel_hdl, false, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mirror TFT panel: %s", esp_err_to_name(ret));
    } else {    
        ESP_LOGI(TAG, "TFT panel mirroring set successfully");
    }
    ret = esp_lcd_panel_swap_xy(panel_hdl, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TFT panel Failed to swap XY: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "TFT panel swap XY set successfully");
    }
    ret = esp_lcd_panel_set_gap(panel_hdl, 0, LCD_V_OFFSET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set TFT panel gap: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "TFT panel gap set successfully");
    }
    ret = esp_lcd_panel_disp_on_off(panel_hdl, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn on TFT panel: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "TFT panel turned on successfully");
    }
    

    /* Touch panel I/O */
    esp_lcd_panel_io_handle_t io_tp;
    esp_lcd_panel_io_spi_config_t tp_io_cfg = {
        .dc_gpio_num       = -1,
        .cs_gpio_num       = PIN_CS_TP,
        .pclk_hz           = 2 * 1000 * 1000,
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
        .spi_mode          = 0,
        .trans_queue_depth = 4,
    };
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST,
                                            &tp_io_cfg, &io_tp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch panel I/O: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Touch panel I/O created successfully");
    }
    esp_lcd_touch_config_t touch_cfg = {
        .x_max        = HRES,
        .y_max        = VRES,
        .rst_gpio_num = -1,
        .int_gpio_num = PIN_IRQ_TP,
        .levels       = { .reset = 0, .interrupt = 0 },
    };
    ret = esp_lcd_touch_new_spi_xpt2046(io_tp, &touch_cfg, &tp_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch panel: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Touch panel created successfully");
    }

    /* LVGL input device */
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);

    /* LVGL tick timer */
    const esp_timer_create_args_t tick_args = {
        .callback = lv_tick_cb,
        .name     = "lv_tick"
    };
    esp_timer_handle_t tick_timer;
    ret = esp_timer_create(&tick_args, &tick_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LVGL tick timer: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "LVGL tick timer created successfully");
    }

    ret = esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_US);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start LVGL tick timer: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "LVGL tick timer started successfully");
    }


    /* Build UI */
    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello ILI9341");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);

    lv_obj_t *btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 100, 40);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_bg_color(btn, lv_color_make(0,127,255), 0);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "PRESS");
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    /* Start GUI task */
    xTaskCreatePinnedToCore(gui_task, "gui", 4096, NULL, 2, NULL, 1);
    vTaskDelete(NULL);
}
