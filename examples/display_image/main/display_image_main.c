#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_vfs_dev.h"
#include "esp_spiffs.h"
#include "esp_jpeg.h"
#include "periph_ilidriver.h"

static const char *TAG = "APP_MAIN";
static esp_jpeg_handle_t jpeg_handle = NULL;
esp_periph_handle_t ilidriver_handle = NULL;

static esp_err_t periph_event_handle(audio_event_iface_msg_t *event, void *context)
{
    return ESP_OK;
}

void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 20,
        .format_if_mount_failed = true
    };
    ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    esp_periph_config_t config = {
        .event_handle = periph_event_handle,
        .max_parallel_connections = 9,
    };
    esp_periph_init(&config);

    periph_ilidriver_cfg_t periph_ilidriver_cfg = {
        .host = HSPI_HOST,
        .dma_chnl = 2,
        .miso = 25,
        .mosi = 23,
        .clk = 19,
        .rst = 18,
        .cs = 22,
        .dc = 21,
        .bckl = 26,
        .size = ILIDRIVER_SIZE_320_240,
        .rot = ILIDRIVER_ROT_0_DEG,
    };
    ilidriver_handle = periph_ilidriver_init(&periph_ilidriver_cfg);
    esp_periph_start(ilidriver_handle);

    jpeg_handle = esp_jpeg_init(periph_ilidriver_get_buffer(ilidriver_handle));
    
    while (1) {
        esp_jpeg_set_file(jpeg_handle, (const char*)"/spiffs/flower1.jpg");
        vTaskDelay(2000 / portTICK_RATE_MS);
        esp_jpeg_set_file(jpeg_handle, (const char*)"/spiffs/flower2.jpg");
        vTaskDelay(2000 / portTICK_RATE_MS);
        esp_jpeg_set_file(jpeg_handle, (const char*)"/spiffs/flower3.jpg");
        vTaskDelay(2000 / portTICK_RATE_MS);
        esp_jpeg_set_file(jpeg_handle, (const char*)"/spiffs/flower4.jpg");
        vTaskDelay(2000 / portTICK_RATE_MS);
        esp_jpeg_set_file(jpeg_handle, (const char*)"/spiffs/flower5.jpg");
        vTaskDelay(2000 / portTICK_RATE_MS);
        esp_jpeg_set_file(jpeg_handle, (const char*)"/spiffs/flower6.jpg");
        vTaskDelay(2000 / portTICK_RATE_MS);
        esp_jpeg_set_file(jpeg_handle, (const char*)"/spiffs/flower7.jpg");
        vTaskDelay(2000 / portTICK_RATE_MS);
        esp_jpeg_set_file(jpeg_handle, (const char*)"/spiffs/flower8.jpg");
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}
