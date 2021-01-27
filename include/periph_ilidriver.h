#ifndef _PERIPH_ILIDRIVER_H_
#define _PERIPH_ILIDRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_peripherals.h"

#include "fonts.h"
#include "colors.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

typedef enum {
	ILIDRIVER_SIZE_320_240 = 0,
	ILIDRIVER_SIZE_480_320,
	ILIDRIVER_SIZE_MAX,
} ilidriver_size_t;

typedef enum {
	ILIDRIVER_ROT_0_DEG = 0,
	ILIDRIVER_ROT_90_DEG,
	ILIDRIVER_ROT_180_DEG,
	ILIDRIVER_ROT_270_DEG,
	ILIDRIVER_ROT_MAX,
} ilidriver_rot_t;

typedef struct {
	const char 				*tag;
	spi_host_device_t 		host;
	int 					dma_chnl;
	int 					miso;
	int 					mosi;
	int 					clk;
	int 					rst;
	int 					cs;
	int 					dc;
	int 					bckl;
	ilidriver_size_t 		size;
	ilidriver_rot_t 		rot;
} periph_ilidriver_cfg_t;

esp_periph_handle_t periph_ilidriver_init(periph_ilidriver_cfg_t *config);
esp_err_t periph_ilidriver_fill(esp_periph_handle_t periph, uint32_t color);
esp_err_t periph_ilidriver_draw_pixel(esp_periph_handle_t periph, uint16_t x, uint16_t y, uint32_t color);
esp_err_t periph_ilidriver_draw_line(esp_periph_handle_t periph, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color);
esp_err_t periph_ilidriver_draw_rectangle(esp_periph_handle_t periph, uint16_t x_origin, uint16_t y_origin, uint16_t width, uint16_t height, uint32_t color);
esp_err_t periph_ilidriver_draw_circle(esp_periph_handle_t periph, uint16_t x_origin, uint16_t y_origin, uint16_t radius, uint32_t color);
esp_err_t periph_ilidriver_set_position(esp_periph_handle_t periph, uint16_t x, uint16_t y);
esp_err_t periph_ilidriver_get_position(esp_periph_handle_t periph, uint16_t *x, uint16_t *y);

#ifdef __cplusplus
}
#endif

#endif /* _PERIPH_ILIDRIVER_H_ */