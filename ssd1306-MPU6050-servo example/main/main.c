#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"

#include "ssd1306.h"
#include "mpu6050.h"

#define TAG               "app"
#define I2C_MASTER_NUM    I2C_NUM_0
#define MPU6050_ADDR      0x68
#define CALIB_SAMPLES     1000
#define SAMPLE_PERIOD_MS  100

#define SERVO_PIN         19
#define SERVO_MIN_ANGLE  -60.0f
#define SERVO_MAX_ANGLE   60.0f
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_FREQ_HZ     50

typedef struct {
    float roll, pitch, yaw, temp;
} mpu_out_t;

static QueueHandle_t mpu_queue;
static mpu6050_handle_t mpu;
static float gyro_off[3];

/** Калибровка гироскопа */
static void calibrate_gyro(void) {
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    mpu6050_gyro_value_t g;

    ESP_LOGI(TAG, "Calibrating gyroscope (%d samples)...", CALIB_SAMPLES);
    for (int i = 0; i < CALIB_SAMPLES; ++i) {
        ESP_ERROR_CHECK(mpu6050_get_gyro(mpu, &g));
        sum_gx += g.gyro_x;
        sum_gy += g.gyro_y;
        sum_gz += g.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    gyro_off[0] = sum_gx / (float)CALIB_SAMPLES;
    gyro_off[1] = sum_gy / (float)CALIB_SAMPLES;
    gyro_off[2] = sum_gz / (float)CALIB_SAMPLES;

    ESP_LOGI(TAG, "Gyro offsets: [%.2f, %.2f, %.2f]", 
             gyro_off[0], gyro_off[1], gyro_off[2]);
}

static void mpu_task(void *pv) {
    const float dt = SAMPLE_PERIOD_MS / 1000.0f;
    mpu_out_t out = {0};
    float yaw = 0.0f;
    mpu6050_acce_value_t a;
    mpu6050_gyro_value_t g;
    mpu6050_temp_value_t t;

    while (1) {
        ESP_ERROR_CHECK(mpu6050_get_acce(mpu, &a));
        ESP_ERROR_CHECK(mpu6050_get_gyro(mpu, &g));
        ESP_ERROR_CHECK(mpu6050_get_temp(mpu, &t));

        float ax = a.acce_x / 16384.0f;
        float ay = a.acce_y / 16384.0f;
        float az = a.acce_z / 16384.0f;

        float gz = (g.gyro_z - gyro_off[2]) / 131.0f;

        out.roll  = atan2f(ay, az) * 180.0f / M_PI;
        out.pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
        yaw += gz * dt;
        out.yaw = yaw;
        out.temp = t.temp;

        ESP_LOGI(TAG, "Angles: Roll=%.2f Pitch=%.2f Yaw=%.2f Temp=%.2f°C",
                 out.roll, out.pitch, out.yaw, out.temp);

        xQueueOverwrite(mpu_queue, &out);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

static void display_task(void *pv) {
    SSD1306_t *oled = (SSD1306_t *)pv;
    mpu_out_t d;
    char buf[32];

    while (1) {
        if (xQueuePeek(mpu_queue, &d, portMAX_DELAY)) {
            //ssd1306_clear_screen(oled, false);
			memset(buf,0,sizeof(buf));
            snprintf(buf, sizeof(buf), "Roll: %.1f", d.roll);
            ssd1306_display_text(oled, 0, buf, 12, false);
			memset(buf,0,sizeof(buf));
            snprintf(buf, sizeof(buf), "Pitch: %.1f", d.pitch);
            ssd1306_display_text(oled, 1, buf, 12, false);
			memset(buf,0,sizeof(buf));
            snprintf(buf, sizeof(buf), "Yaw: %.1f", d.yaw);
            ssd1306_display_text(oled, 2, buf, 12, false);
			memset(buf,0,sizeof(buf));
            snprintf(buf, sizeof(buf), "T: %.1f C", d.temp);
            ssd1306_display_text(oled, 3, buf, 12, false);

            ssd1306_show_buffer(oled);
        }
		vTaskDelay(pdMS_TO_TICKS(10));  // вместо 10
    }
	
}

static void servo_set_angle(float angle_deg) {
    if (angle_deg < SERVO_MIN_ANGLE) angle_deg = SERVO_MIN_ANGLE;
    if (angle_deg > SERVO_MAX_ANGLE) angle_deg = SERVO_MAX_ANGLE;

    float ratio = (angle_deg - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
    int pulse_us = SERVO_MIN_PULSE_US + (int)((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * ratio);

    uint32_t duty = (pulse_us * (uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000) / (1000000 / SERVO_FREQ_HZ) / 2;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void servo_task(void *pv) {
    mpu_out_t d;
    while (1) {
        if (xQueuePeek(mpu_queue, &d, portMAX_DELAY)) {
            servo_set_angle(d.pitch);
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

void app_main(void) {
    SSD1306_t oled;

    ESP_LOGI(TAG, "Initializing SSD1306 and MPU6050...");
    i2c_master_init(&oled, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

#if CONFIG_SSD1306_128x64
    ssd1306_init(&oled, 128, 64);
#elif CONFIG_SSD1306_128x32
    ssd1306_init(&oled, 128, 32);
#else
#error "Select SSD1306 size in menuconfig"
#endif

    mpu = mpu6050_create(I2C_MASTER_NUM, MPU6050_ADDR);
    ESP_ERROR_CHECK(mpu6050_config(mpu, ACCE_FS_2G, GYRO_FS_250DPS));
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu));
    vTaskDelay(pdMS_TO_TICKS(100));

    calibrate_gyro();

    ESP_LOGI(TAG, "Configuring LEDC for servo...");
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_channel_config_t ledc_channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&ledc_channel);

    mpu_queue = xQueueCreate(1, sizeof(mpu_out_t));

    xTaskCreate(display_task, "display_task", 4096, &oled, 4, NULL);
    xTaskCreate(servo_task, "servo_task", 4096, NULL, 4, NULL);
    xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 5, NULL);
	
	while (1)
		vTaskDelay(pdMS_TO_TICKS(100));
		
}
