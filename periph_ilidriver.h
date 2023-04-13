/**
 * @file periph_ilidriver.h
 *
 * ESP-IDF's component controls the display of TFT LCD ILI9341. The RGB565 format
 * is used to decrease the capacity of data sent on the SPI line.
 *
 * MIT License
 *
 * Copyright (c) 2023 phonght32
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _PERIPH_ILIDRIVER_H_
#define _PERIPH_ILIDRIVER_H_

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_peripherals.h"
#include "fonts.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @enum 	ILI driver size enum.
 */
typedef enum {
	ILIDRIVER_SIZE_320_240 = 0,
	ILIDRIVER_SIZE_480_320,
	ILIDRIVER_SIZE_MAX,
} ilidriver_size_t;

/**
 * @enum   	ILI driver rotation enum.
 */
typedef enum {
	ILIDRIVER_ROT_0_DEG = 0,
	ILIDRIVER_ROT_90_DEG,
	ILIDRIVER_ROT_180_DEG,
	ILIDRIVER_ROT_270_DEG,
	ILIDRIVER_ROT_MAX,
} ilidriver_rot_t;

/**
 * @struct  Peripheral ilidriver configuration structure.
 *
 * @param 	tag Module tag that is displayed in ESP_LOG.
 * @param 	host SPI host.
 * @param 	dma_chnl DMA channel.
 * @param 	miso Screen MISO pin.
 * @param 	mosi Screen MOSI pin.
 * @param 	clk Screen clock pin.
 * @param 	rst Screen reset pin.
 * @param 	cs Screen chip select pin.
 * @param 	dc Screen DC pin.
 * @param 	bckl Screen LED backlight pin.
 * @param 	size Screen size.
 * @param 	rot Screen rotation.
 */
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

/**
 * @brief   Initialize ilidriver peripheral.
 * 
 * @param   config Pointer references to the configuration structure.
 *
 * @return  
 * 		- Pointer to the ilidriver handle structure.
 * 		- NULL: Failed.
 */
esp_periph_handle_t periph_ilidriver_init(periph_ilidriver_cfg_t *config);

/**
 * @brief   Fill screen with color.
 * 
 * @param   periph ILI driver handle structure.
 * @param 	color Color.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_fill(esp_periph_handle_t periph, uint32_t color);

/**
 * @brief   Write character.
 * 
 * @param   periph ilidriver handle structure.
 * @param 	font_size Font size.
 * @param 	chr Character.
 * @param 	color Color.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_write_char(esp_periph_handle_t periph, font_size_t font_size, uint8_t chr, uint32_t color);

/**
 * @brief   Write string.
 * 
 * @param   periph ilidriver handle structure.
 * @param 	font_size Font size.
 * @param 	str Pointer references to the data.
 * @param 	color Color.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_write_string(esp_periph_handle_t periph, font_size_t font_size, uint8_t *str, uint32_t color);

/**
 * @brief   Draw pixel.
 * 
 * @param   periph ilidriver handle structure.
 * @param 	x Horizontal position.
 * @param 	y Vertical position.
 * @param 	color Color.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_draw_pixel(esp_periph_handle_t periph, uint16_t x, uint16_t y, uint32_t color);

/**
 * @brief   Draw line.
 * 
 * @param   periph ilidriver handle structure.
 * @param 	x1 The first horizontal position.
 * @param 	y1 The first vertical postion.
 * @param 	x2 The second horizontal position.
 * @param 	y2 The second vertical position.
 * @param 	color Color.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_draw_line(esp_periph_handle_t periph, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color);

/**
 * @brief   Draw rectangle.
 * 
 * @param   periph ilidriver handle structure.
 * @param 	x_origin Origin horizontal position.
 * @param   y_origin Origin vertical position.
 * @param   width Width in pixel.
 * @param   height Height in pixel.
 * @param   color Color.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_draw_rectangle(esp_periph_handle_t periph, uint16_t x_origin, uint16_t y_origin, uint16_t width, uint16_t height, uint32_t color);

/**
 * @brief   Draw Circle.
 * 
 * @param   periph ilidriver handle structure.
 * @param   x_origin Origin horizontal position.
 * @param   y_origin Origin vertical position.
 * @param 	radius Radius in pixel.
 * @param   color Color.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_draw_circle(esp_periph_handle_t periph, uint16_t x_origin, uint16_t y_origin, uint16_t radius, uint32_t color);

/**
 * @brief   Set current position.
 * 
 * @param   periph ilidriver handle structure.
 * @param 	x Horizontal position.
 * @param   y Vertical position.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_set_position(esp_periph_handle_t periph, uint16_t x, uint16_t y);

/**
 * @brief   Get current position.
 * 
 * @param   periph ilidriver handle structure.
 * @param 	x Pointer references to the horizontal position.
 * @param   y Pointer references to the vertical position.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_get_position(esp_periph_handle_t periph, uint16_t *x, uint16_t *y);

/**
 * @brief   Pause screen.
 * 
 * @param   periph ilidriver handle structure.	
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_pause(esp_periph_handle_t periph);

/**
 * @brief   Start screen.
 * 
 * @param   periph ilidriver handle structure. 	
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_ilidriver_start(esp_periph_handle_t periph);

/**
 * @brief   Get the screen data buffer.
 * 
 * @param   periph ilidriver handle structure. 	
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
uint8_t *periph_ilidriver_get_buffer(esp_periph_handle_t periph);

#ifdef __cplusplus
}
#endif

#endif /* _PERIPH_ILIDRIVER_H_ */