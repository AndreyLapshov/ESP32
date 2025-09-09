#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"

#include "esp_log.h"
#include "esp_check.h"

#include "esp_lcd_types.h"
#include "esp_lcd_panel_interface.h"   // esp_lcd_panel_t / handle
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_vendor.h"      // ST7789

static const char *TAG = "TFT_PROBE";

/* ----- Пины для двух SPI-хостов ----- */
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define HSPI_MOSI 13
#define HSPI_SCLK 14

/* Управляющие пины */
#define PIN_CS    5      // для режима CS=GND не подключайте этот провод (в коде cs_gpio_num=-1)
#define PIN_DC    27
#define PIN_RST   32
#define PIN_BL    33

/* Геометрия и SPI */
#define LCD_W       240
#define LCD_H       280
#define SPI_SAFE_HZ (80 * 1000 * 1000)  // начните с 12 МГц

/* ===== UART0: мгновенный приём ok/no или y/n, Enter не обязателен ===== */
static void uart0_init_console(void)
{
    const uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &cfg);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
}

static bool uart_wait_ok_or_no(void)
{
    ESP_LOGW(TAG, "Введите: ok/no (или y/n). Enter НЕ обязателен.");
    char buf[4]; size_t pos = 0;

    while (1) {
        uint8_t ch;
        int n = uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY);
        if (n <= 0) continue;

        ch = (uint8_t)tolower((int)ch);

        // Игнорируем перевод строк/пробелы
        if (ch == '\r' || ch == '\n' || ch == ' ') { pos = 0; continue; }

        // Мгновенно принимаем одиночные 'y' / 'n'
        if (ch == 'y') { ESP_LOGW(TAG, "Получил 'y' -> OK"); uart_flush_input(UART_NUM_0); return true; }
        if (ch == 'n') { ESP_LOGW(TAG, "Получил 'n' -> NO"); uart_flush_input(UART_NUM_0); return false; }

        // Копим до 2 символов для 'ok' / 'no'
        if (pos < sizeof(buf) - 1) buf[pos++] = (char)ch;
        buf[pos] = '\0';

        if (strcmp(buf, "ok") == 0) { ESP_LOGW(TAG, "Получил 'ok'"); uart_flush_input(UART_NUM_0); return true; }
        if (strcmp(buf, "no") == 0) { ESP_LOGW(TAG, "Получил 'no'"); uart_flush_input(UART_NUM_0); return false; }

        // Если набралось что-то иное длиной >=2 — сброс и ждём заново
        if (pos >= 2) pos = 0;
    }
}

/* ===== Утилиты ===== */
static inline void gpio_out(int io, int lvl) {
    if (io < 0) return;
    gpio_config_t c = {.pin_bit_mask = 1ULL << io, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&c);
    gpio_set_level(io, lvl);
}
static inline uint16_t bswap16(uint16_t v){ return (uint16_t)((v<<8)|(v>>8)); }

/* Буфер строки — один на всё приложение (в куче) */
static uint16_t *g_line = NULL;
static void draw_fill(esp_lcd_panel_handle_t panel, uint16_t c565, int case_no, int fill_no, const char *name)
{
    if (!g_line) g_line = heap_caps_malloc(LCD_W * sizeof(uint16_t), MALLOC_CAP_DMA);
    uint16_t px = bswap16(c565);
    for (int i=0;i<LCD_W;i++) g_line[i] = px;

    ESP_LOGI(TAG, "CASE #%02d — Fill #%d %s", case_no, fill_no, name);
    for (int y = 0; y < LCD_H; y++) {
        // x_end,y_end — эксклюзивные
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_W, y + 1, g_line));
    }
}

/* ===== Кастомная панель NV3030B ===== */
typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int rst, x_gap, y_gap;
    uint8_t madctl, colmod;
} nv_panel_t;

static inline esp_err_t nv_tx(nv_panel_t *p, uint8_t cmd, const void *d, size_t n){
    return esp_lcd_panel_io_tx_param(p->io, cmd, d, n);
}
static esp_err_t nv_del(esp_lcd_panel_t *panel){ free(__containerof(panel,nv_panel_t,base)); return ESP_OK; }
static esp_err_t nv_reset(esp_lcd_panel_t *panel){
    nv_panel_t *p=__containerof(panel,nv_panel_t,base);
    if (p->rst>=0){ gpio_set_level(p->rst,0); vTaskDelay(pdMS_TO_TICKS(10)); gpio_set_level(p->rst,1); vTaskDelay(pdMS_TO_TICKS(120)); }
    return ESP_OK;
}
static esp_err_t nv_init(esp_lcd_panel_t *panel){
    nv_panel_t *p=__containerof(panel,nv_panel_t,base);
    ESP_RETURN_ON_ERROR(nv_tx(p,0x11,NULL,0),TAG,"SLPOUT"); vTaskDelay(pdMS_TO_TICKS(120));
    p->colmod=0x55; ESP_RETURN_ON_ERROR(nv_tx(p,0x3A,&p->colmod,1),TAG,"COLMOD");           // 16bpp
    ESP_RETURN_ON_ERROR(nv_tx(p,0xFE,(uint8_t[]){0x61},1),TAG,"FE61");
    ESP_RETURN_ON_ERROR(nv_tx(p,0xFE,(uint8_t[]){0x70},1),TAG,"FE70");
    ESP_RETURN_ON_ERROR(nv_tx(p,0xB6,(uint8_t[]){0x04,0x00,0x9F,0x00,0x02},5),TAG,"B6");
    ESP_RETURN_ON_ERROR(nv_tx(p,0x36,&p->madctl,1),TAG,"MADCTL");                            // RGB/BGR + ориентация
    ESP_RETURN_ON_ERROR(nv_tx(p,0x29,NULL,0),TAG,"DISPON"); vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}
static esp_err_t nv_set_gap(esp_lcd_panel_t *panel,int x,int y){
    nv_panel_t *p=__containerof(panel,nv_panel_t,base); p->x_gap=x; p->y_gap=y; return ESP_OK;
}
static esp_err_t nv_draw(esp_lcd_panel_t *panel,int xs,int ys,int xe,int ye,const void *color)
{
    nv_panel_t *p = __containerof(panel, nv_panel_t, base);
    if (xe <= xs || ye <= ys) return ESP_OK;  // пустая область — ок

    // ESP-IDF: x_end,y_end — эксклюзивные; контроллер — инклюзивные
    uint16_t x0 = xs + p->x_gap;
    uint16_t y0 = ys + p->y_gap;
    uint16_t x1 = (xe - 1) + p->x_gap;
    uint16_t y1 = (ye - 1) + p->y_gap;

    uint8_t caset[4] = { x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF };
    uint8_t raset[4] = { y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF };
    ESP_RETURN_ON_ERROR(nv_tx(p, 0x2A, caset, 4), TAG, "CASET");
    ESP_RETURN_ON_ERROR(nv_tx(p, 0x2B, raset, 4), TAG, "RASET");

    size_t w = (size_t)(xe - xs);
    size_t h = (size_t)(ye - ys);
    return esp_lcd_panel_io_tx_color(p->io, 0x2C, color, w * h * 2);
}
static esp_err_t nv_inv(esp_lcd_panel_t *panel,bool inv){ return nv_tx(__containerof(panel,nv_panel_t,base), inv?0x21:0x20, NULL, 0); }
static esp_err_t nv_on (esp_lcd_panel_t *panel,bool on ){ return nv_tx(__containerof(panel,nv_panel_t,base), on ?0x29:0x28, NULL, 0); }

static esp_err_t new_nv3030b(esp_lcd_panel_io_handle_t io, int rst, bool use_bgr, esp_lcd_panel_handle_t *out)
{
    nv_panel_t *p = calloc(1,sizeof(*p));
    ESP_RETURN_ON_FALSE(p, ESP_ERR_NO_MEM, TAG, "oom");
    p->io=io; p->rst=rst;
    p->madctl = use_bgr ? 0x08 : 0x00;   // BGR/RGB
    p->base.del=nv_del; p->base.reset=nv_reset; p->base.init=nv_init;
    p->base.draw_bitmap=nv_draw; p->base.set_gap=nv_set_gap;
    p->base.invert_color=nv_inv; p->base.disp_on_off=nv_on;
    *out=&p->base; return ESP_OK;
}

/* ===== Один «тест-кейс» ===== */
typedef enum { DRV_NV3030B=0, DRV_ST7789=1 } drv_t;

static bool try_one(int case_no,
                    spi_host_device_t host, int mosi, int sclk,
                    bool cs_via_pin, drv_t drv, bool bgr, bool inv, int yoff)
{
    ESP_LOGW(TAG, "CASE #%02d: host=%s MOSI=%d SCLK=%d  CS=%s  drv=%s  %s  INV=%s  YOFF=%d",
             case_no, (host==SPI2_HOST)?"SPI2":"SPI3", mosi, sclk,
             cs_via_pin?"PIN":"GND(-1)", drv==DRV_NV3030B?"NV3030B":"ST7789",
             bgr?"BGR":"RGB", inv?"ON":"OFF", yoff);

    // BUS
    spi_bus_config_t bus = {
        .sclk_io_num = sclk, .mosi_io_num = mosi, .miso_io_num = -1,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = LCD_W*80*2
    };
    if (spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        ESP_LOGE(TAG, "bus init failed"); return false;
    }

    // IO
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_DC,
        .cs_gpio_num = cs_via_pin ? PIN_CS : -1,
        .pclk_hz = SPI_SAFE_HZ,
        .spi_mode = 0,
        .lcd_cmd_bits = 8, .lcd_param_bits = 8,
        .trans_queue_depth = 6,
        .flags = {.lsb_first=0, .cs_high_active=0, .dc_low_on_data=0},
    };
    if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)host, &io_cfg, &io) != ESP_OK) {
        ESP_LOGE(TAG, "io create failed");
        spi_bus_free(host);
        return false;
    }

    // PANEL
    esp_lcd_panel_handle_t panel = NULL;
    esp_err_t err;
    if (drv == DRV_NV3030B) {
        err = new_nv3030b(io, PIN_RST, bgr, &panel);
        if (err==ESP_OK) err = esp_lcd_panel_reset(panel);
        if (err==ESP_OK) err = esp_lcd_panel_init(panel);
        if (err==ESP_OK) err = esp_lcd_panel_invert_color(panel, inv);
        if (err==ESP_OK) err = esp_lcd_panel_set_gap(panel, 0, yoff);
        if (err==ESP_OK) err = esp_lcd_panel_disp_on_off(panel, true);
    } else {
        esp_lcd_panel_dev_config_t cfg = {
            .reset_gpio_num = PIN_RST,
            .rgb_ele_order  = bgr ? LCD_RGB_ELEMENT_ORDER_BGR : LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = 16,
        };
        err = esp_lcd_new_panel_st7789(io, &cfg, &panel);
        if (err==ESP_OK) err = esp_lcd_panel_reset(panel);
        if (err==ESP_OK) vTaskDelay(pdMS_TO_TICKS(120));
        if (err==ESP_OK) err = esp_lcd_panel_init(panel);
        if (err==ESP_OK) err = esp_lcd_panel_invert_color(panel, inv);
        if (err==ESP_OK) err = esp_lcd_panel_set_gap(panel, 0, yoff);
        if (err==ESP_OK) err = esp_lcd_panel_disp_on_off(panel, true);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "panel init failed: 0x%x", err);
        esp_lcd_panel_io_del(io);
        spi_bus_free(host);
        return false;
    }

    // BL ON
    gpio_set_level(PIN_BL, 1);

    // ТЕСТ: R->G->B (пронумерованы)
    draw_fill(panel, 0xF800, case_no, 1, "RED");   vTaskDelay(pdMS_TO_TICKS(150));
    draw_fill(panel, 0x07E0, case_no, 2, "GREEN"); vTaskDelay(pdMS_TO_TICKS(150));
    draw_fill(panel, 0x001F, case_no, 3, "BLUE");  vTaskDelay(pdMS_TO_TICKS(200));

    // Ждём ok/no
    bool selected = uart_wait_ok_or_no();

    // Освобождение ресурсов кейса
    esp_lcd_panel_del(panel);
    esp_lcd_panel_io_del(io);
    spi_bus_free(host);

    if (selected) {
        ESP_LOGW(TAG, "=== SELECTED CASE #%02d ===", case_no);
        ESP_LOGW(TAG, "host:        %s", (host==SPI2_HOST)?"SPI2_HOST":"SPI3_HOST");
        ESP_LOGW(TAG, "MOSI (DIN):  %d", mosi);
        ESP_LOGW(TAG, "SCLK (CLK):  %d", sclk);
        ESP_LOGW(TAG, "CS mode:     %s", cs_via_pin? "GPIO pin (PIN_CS)" : "GND (cs_gpio_num=-1)");
        ESP_LOGW(TAG, "driver:      %s", (drv==DRV_NV3030B)?"NV3030B (custom)":"ST7789 (IDF built-in)");
        ESP_LOGW(TAG, "color order: %s", bgr? "BGR" : "RGB");
        ESP_LOGW(TAG, "invert:      %s", inv? "ON" : "OFF");
        ESP_LOGW(TAG, "Y offset:    %d", yoff);
        ESP_LOGW(TAG, "=====================================");
    }

    return selected; // true => остановиться
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== TFT auto-probe 240x280 (UART ok/no) ===");

    uart0_init_console();

    // Подсветка выкл. на старте, DC/RST в дефолт
    gpio_out(PIN_BL, 0);
    gpio_out(PIN_DC, 1);
    gpio_out(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_BL, 1); // включим BL

    const spi_host_device_t hosts[2] = { SPI2_HOST, SPI3_HOST };
    const int mosi[2] = { VSPI_MOSI, HSPI_MOSI };
    const int sclk[2] = { VSPI_SCLK, HSPI_SCLK };
    const bool cs_pin[2] = { true, false };      // true=CS на GPIO5, false=CS=-1 (GND)
    const drv_t drvs[2] = { DRV_NV3030B, DRV_ST7789 };
    const bool bgrs[2] = { true, false };
    const bool invs[2] = { true, false };
    const int yoffs[3] = { 20, 0, 40 };

    int case_no = 1;
    bool done = false;

    for (int h=0; h<2 && !done; ++h)
    for (int cs=0; cs<2 && !done; ++cs)
    for (int d=0; d<2 && !done; ++d)
    for (int c=0; c<2 && !done; ++c)
    for (int iv=0; iv<2 && !done; ++iv)
    for (int yo=0; yo<3 && !done; ++yo) {
        bool selected = try_one(case_no++,
                                hosts[h], mosi[h], sclk[h],
                                cs_pin[cs], drvs[d], bgrs[c], invs[iv], yoffs[yo]);
        if (selected) done = true;
    }

    if (!done) {
        ESP_LOGE(TAG, "Ни один профиль не был подтверждён ('ok'). Проверьте распайку: VCC, GND, DIN(MOSI), CLK, CS, DC, RST, BL.");
    } else {
        ESP_LOGW(TAG, "Автопоиск завершён по команде 'ok'.");
    }

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
