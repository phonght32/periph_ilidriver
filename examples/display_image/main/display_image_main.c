#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "periph_ilidriver.h"
#include "esp_img.h"

esp_periph_handle_t ilidriver_handle;
esp_img_handle_t img_handle;

static esp_err_t periph_event_handle(audio_event_iface_msg_t *event, void *context)
{
    return ESP_OK;
}

void app_main(void)
{
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
    
    img_handle = esp_img_init(periph_ilidriver_get_buffer(ilidriver_handle));
    esp_img_set_data(img_handle, 320, 240);

    periph_ilidriver_set_position(ilidriver_handle, 40, 215);
    periph_ilidriver_write_string(ilidriver_handle, FONT_SIZE_12x16, (uint8_t *)"Tp.HCM, 28/11/2020", COLOR_WHITE);

    while (1) {
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}
