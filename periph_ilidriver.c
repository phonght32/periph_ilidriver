#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "periph_ilidriver.h"

#define SPI_PARALLEL_LINES 				16

#define ILIDRIVER_TASK_SIZE 			4*1024

#define mutex_lock(x)					while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) 				xSemaphoreGive(x)
#define mutex_create()					xSemaphoreCreateMutex()
#define mutex_destroy(x) 				vQueueDelete(x)

#define VALIDATE_ILIDRIVER(periph, ret) if(!esp_periph_validate(periph, PERIPH_ID_ILIDRIVER)) {		\
	ESP_LOGE(TAG, "Invalid PERIPH_ID_ILIDRIVER");													\
	return ret;																						\
}

#define ILIDRIVER_CHECK(a, str, action) if(!(a)) {                          \
    ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);  \
    action;                                                                 \
}

typedef struct {
	uint8_t cmd;
	uint8_t data[16];
	uint8_t databytes; 		/*!< No of data in data; bit 7 = delay after set; 0xFF = end of cmds. */
} lcd_init_cmd_t;

typedef enum {
	SELECT_ENABLE = 0,
	SELECT_DISABLE
} select_t;

typedef struct {
	uint8_t cmd;
	uint8_t data[16];
	uint8_t databytes;
} ilidriver_init_cmd_t;

typedef struct {
	uint16_t *data;
} lines_t;

typedef struct {
	spi_host_device_t	host;
	int 				dma_chnl;
	int 				miso;
	int 				mosi;
	int 				clk;
	int 				rst;
	int 				cs;
	int 				dc;
	int 				bckl;
	spi_device_handle_t spi;
	ilidriver_size_t 	size;
	ilidriver_rot_t 	rot;
	uint16_t 			width;
	uint16_t 			height;
	uint16_t 			pos_x;
	uint16_t 			pos_y;
	uint8_t 			*buf;
	lines_t 			lines[2];
	uint16_t 			lines_idx;
	bool 				pause;
	bool 				is_started;
} periph_ilidriver_t;

static const char *TAG = "PERIPH_ILIDRIVER";
static esp_periph_handle_t g_ilidriver;

DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[] = {
	/* Power contorl B, power control = 0, DC_ENA = 1 */
	{0xCF, {0x00, 0x83, 0X30}, 3},
	/* Power on sequence control,
	 * cp1 keeps 1 frame, 1st frame enable
	 * vcl = 0, ddvdh=3, vgh=1, vgl=2
	 * DDVDH_ENH=1
	 */
	{0xED, {0x64, 0x03, 0X12, 0X81}, 4},
	/* Driver timing control A,
	 * non-overlap=default +1
	 * EQ=default - 1, CR=default
	 * pre-charge=default - 1
	 */
	{0xE8, {0x85, 0x01, 0x79}, 3},
	/* Power control A, Vcore=1.6V, DDVDH=5.6V */
	{0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
	/* Pump ratio control, DDVDH=2xVCl */
	{0xF7, {0x20}, 1},
	/* Driver timing control, all=0 unit */
	{0xEA, {0x00, 0x00}, 2},
	/* Power control 1, GVDD=4.75V */
	{0xC0, {0x26}, 1},
	/* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
	{0xC1, {0x11}, 1},
	/* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
	{0xC5, {0x35, 0x3E}, 2},
	/* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
	{0xC7, {0xBE}, 1},
	/* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
	{0x36, {0x28}, 1},
	/* Pixel format, 16bits/pixel for RGB/MCU interface */
	{0x3A, {0x55}, 1},
	/* Frame rate control, f=fosc, 70Hz fps */
	{0xB1, {0x00, 0x1B}, 2},
	/* Enable 3G, disabled */
	{0xF2, {0x08}, 1},
	/* Gamma set, curve 1 */
	{0x26, {0x01}, 1},
	/* Positive gamma correction */
	{0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
	/* Negative gamma correction */
	{0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
	/* Column address set, SC=0, EC=0xEF */
	{0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
	/* Page address set, SP=0, EP=0x013F */
	{0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
	/* Memory write */
	{0x2C, {0}, 0},
	/* Entry mode set, Low vol detect disabled, normal display */
	{0xB7, {0x07}, 1},
	/* Display function control */
	{0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
	/* Sleep out */
	{0x11, {0}, 0x80},
	/* Display on */
	{0x29, {0}, 0x80},
	{0, {0}, 0xff},
};

static uint16_t _get_screen_width(ilidriver_size_t size)
{
	uint16_t width = 320;
	switch (size) {
	case ILIDRIVER_SIZE_320_240:
		width = 320;
		break;
	case ILIDRIVER_SIZE_480_320:
		width = 480;
		break;
	default:
		break;
	}

	return width;
}

static uint16_t _get_screen_height(ilidriver_size_t size)
{
	uint16_t height = 240;
	switch (size) {
	case ILIDRIVER_SIZE_320_240:
		height = 240;
		break;
	case ILIDRIVER_SIZE_480_320:
		height = 320;
		break;
	default:
		break;
	}

	return height;
}

static void _pre_transfer_callback(spi_transaction_t *t)
{
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(g_ilidriver);
	int dc = (int)t->user;
	gpio_set_level(periph_ilidriver->dc, dc);
}

static void _write_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.tx_buffer = &cmd;
	t.user = (void*)0;
	ret = spi_device_polling_transmit(spi, &t);
	assert(ret == ESP_OK);
}

static void _write_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
	esp_err_t ret;
	spi_transaction_t t;
	if (len == 0) return;
	memset(&t, 0, sizeof(t));
	t.length = len * 8;
	t.tx_buffer = data;
	t.user = (void*)1;
	ret = spi_device_polling_transmit(spi, &t);
	assert(ret == ESP_OK);
}

static void _write_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata)
{
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(g_ilidriver);

	esp_err_t ret;
	int x;
	static spi_transaction_t trans[6];

	for (x = 0; x < 6; x++) {
		memset(&trans[x], 0, sizeof(spi_transaction_t));
		if ((x & 1) == 0) {
			//Even transfers are commands
			trans[x].length = 8;
			trans[x].user = (void*)0;
		} else {
			//Odd transfers are data
			trans[x].length = 8 * 4;
			trans[x].user = (void*)1;
		}
		trans[x].flags = SPI_TRANS_USE_TXDATA;
	}
	trans[0].tx_data[0] = 0x2A;         													/*!< Command set column address */
	trans[1].tx_data[0] = 0;            													/*!< Start column high */
	trans[1].tx_data[1] = 0;           														/*!< Start column low  */
	trans[1].tx_data[2] = (periph_ilidriver->width) >> 8;   								/*!< End column high */
	trans[1].tx_data[3] = (periph_ilidriver->width) & 0xff; 								/*!< End column low */
	trans[2].tx_data[0] = 0x2B;         													/*!< Command set page address */
	trans[3].tx_data[0] = ypos >> 8;    													/*!< Start page high */
	trans[3].tx_data[1] = ypos & 0xff;  													/*!< Start page low */
	trans[3].tx_data[2] = (ypos + SPI_PARALLEL_LINES) >> 8;  								/*!< End page high */
	trans[3].tx_data[3] = (ypos + SPI_PARALLEL_LINES) & 0xff; 								/*!< End page low */
	trans[4].tx_data[0] = 0x2C;         													/*!< Command set data */
	trans[5].tx_buffer = linedata;      													/*!< Data transfer */
	trans[5].length = periph_ilidriver->width * 8 * sizeof(uint16_t) * SPI_PARALLEL_LINES;  /*!< Data length in bits */
	trans[5].flags = 0; 																	/*!< Undo SPI_TRANS_USE_TXDATA flag */

	for (x = 0; x < 6; x++) {
		ret = spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
		assert(ret == ESP_OK);
	}
}

static void _write_line_finish(spi_device_handle_t spi)
{
	spi_transaction_t *rtrans;
	esp_err_t ret;
	for (int x = 0; x < 6; x++) {
		ret = spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
		assert(ret == ESP_OK);
	}
}

static void _draw_pixel(uint16_t x, uint16_t y, uint32_t color)
{
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(g_ilidriver);

	uint8_t *p = periph_ilidriver->buf + (x + y * periph_ilidriver->width) * 3;
	p[0] = (color >> 16) & 0xFF;
	p[1] = (color >> 8) & 0xFF;
	p[2] = (color >> 0) & 0xFF;
}

static void _draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color)
{
	int32_t deltaX = abs(x2 - x1);
	int32_t deltaY = abs(y2 - y1);
	int32_t signX = ((x1 < x2) ? 1 : -1);
	int32_t signY = ((y1 < y2) ? 1 : -1);
	int32_t error = deltaX - deltaY;
	int32_t error2;

	_draw_pixel(x2, y2, color);

	while ((x1 != x2) || (y1 != y2))
	{
		_draw_pixel(x1, y1, color);

		error2 = error * 2;
		if (error2 > -deltaY) {
			error -= deltaY;
			x1 += signX;
		} else {
			/*nothing to do*/
		}

		if (error2 < deltaX) {
			error += deltaX;
			y1 += signY;
		} else {
			/*nothing to do*/
		}
	}
}

static void _convert_pixel_to_lines(int height_idx)
{
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(g_ilidriver);

	for (int idx = 0; idx < (periph_ilidriver->width * SPI_PARALLEL_LINES); idx++) {
		uint8_t *p_src = periph_ilidriver->buf + periph_ilidriver->width * height_idx * 3 + idx * 3;
		uint16_t *p_desc = periph_ilidriver->lines[periph_ilidriver->lines_idx].data + idx;

		uint32_t rgb = (p_src[0] << 16) | (p_src[1] << 8) | (p_src[2]);
		uint16_t swap565;
		rgb_2_swap565(rgb, &swap565);
		*p_desc = swap565;
	}
}

static void _ilidriver_task(void *pv)
{
	esp_periph_handle_t periph = (esp_periph_handle_t)pv;
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(periph);

	int sending_line = -1;

	while (periph_ilidriver->is_started) {
		if (!periph_ilidriver->pause) {
			for (int y = 0; y < periph_ilidriver->height; y += SPI_PARALLEL_LINES) {
				if (sending_line != -1) {
					_write_line_finish(periph_ilidriver->spi);
				}
				_convert_pixel_to_lines(y);
				sending_line = periph_ilidriver->lines_idx;
				_write_lines(periph_ilidriver->spi, y, periph_ilidriver->lines[sending_line].data);
				periph_ilidriver->lines_idx ^= 1;
			}
		}
		vTaskDelay(10 / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

static esp_err_t _ilidriver_init(esp_periph_handle_t self)
{
	VALIDATE_ILIDRIVER(self, ESP_FAIL);
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(self);
	if (!periph_ilidriver->is_started) {
		spi_bus_config_t buscfg = {
			.miso_io_num = periph_ilidriver->miso,
			.mosi_io_num = periph_ilidriver->mosi,
			.sclk_io_num = periph_ilidriver->clk,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = SPI_PARALLEL_LINES * periph_ilidriver->width * sizeof(uint16_t) + 8
		};
		spi_device_interface_config_t devcfg = {
			.clock_speed_hz = 10 * 1000 * 1000,
			.mode = 0,
			.spics_io_num = periph_ilidriver->cs,
			.queue_size = 7,
			.pre_cb = _pre_transfer_callback,
		};

		spi_bus_initialize(periph_ilidriver->host, &buscfg, periph_ilidriver->dma_chnl);
		spi_bus_add_device(periph_ilidriver->host, &devcfg, &periph_ilidriver->spi);

		gpio_set_direction(periph_ilidriver->dc, GPIO_MODE_OUTPUT);
		gpio_set_direction(periph_ilidriver->rst, GPIO_MODE_OUTPUT);
		gpio_set_direction(periph_ilidriver->bckl, GPIO_MODE_OUTPUT);

		gpio_set_level(periph_ilidriver->rst, 0);
		vTaskDelay(100 / portTICK_RATE_MS);
		gpio_set_level(periph_ilidriver->rst, 1);
		vTaskDelay(100 / portTICK_RATE_MS);

		int cmd = 0;
		const lcd_init_cmd_t* lcd_init_cmds;

		lcd_init_cmds = ili_init_cmds;

		while (lcd_init_cmds[cmd].databytes != 0xff) {
			_write_cmd(periph_ilidriver->spi, lcd_init_cmds[cmd].cmd);
			_write_data(periph_ilidriver->spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);
			if (lcd_init_cmds[cmd].databytes & 0x80) {
				vTaskDelay(100 / portTICK_RATE_MS);
			}
			cmd++;
		}
		gpio_set_level(periph_ilidriver->bckl, 1);

		periph_ilidriver->is_started = true;
		xTaskCreatePinnedToCore(_ilidriver_task, "ilidriver_task", ILIDRIVER_TASK_SIZE, self, 5, NULL, 1);

	}

	return ESP_OK;
}

static esp_err_t _ilidriver_run(esp_periph_handle_t self, audio_event_iface_msg_t *msg)
{
	return ESP_OK;
}

static esp_err_t _ilidriver_destroy(esp_periph_handle_t self)
{
	VALIDATE_ILIDRIVER(self, ESP_FAIL);
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(self);

	free(periph_ilidriver->lines[0].data);
	free(periph_ilidriver->lines[1].data);
	free(periph_ilidriver->buf);
	free(periph_ilidriver);

	return ESP_OK;
}

esp_periph_handle_t periph_ilidriver_init(periph_ilidriver_cfg_t *config)
{
	ILIDRIVER_CHECK(config, "error config null", return NULL);

	esp_periph_handle_t periph = esp_periph_create(PERIPH_ID_ILIDRIVER, config->tag ? config->tag : "periph_ilidriver");
	periph_ilidriver_t *periph_ilidriver = calloc(1, sizeof(periph_ilidriver_t));
	PERIPH_MEM_CHECK(TAG, periph_ilidriver, {free(periph); return NULL;});

	uint16_t width = _get_screen_width(config->size);
	uint16_t height = _get_screen_height(config->size);

	periph_ilidriver->buf = heap_caps_malloc(width * height * 3 * sizeof(uint8_t), MALLOC_CAP_SPIRAM);
	ILIDRIVER_CHECK(periph_ilidriver->buf, "error malloc periph_ilidriver->buf", {
		free(periph_ilidriver);
		free(periph);
		return NULL;
	});

	memset(periph_ilidriver->buf, 0xFF, width * height * 3 * sizeof(uint8_t));

	periph_ilidriver->lines[0].data = heap_caps_malloc(width * SPI_PARALLEL_LINES * sizeof(uint16_t), MALLOC_CAP_DMA);
	periph_ilidriver->lines[1].data = heap_caps_malloc(width * SPI_PARALLEL_LINES * sizeof(uint16_t), MALLOC_CAP_DMA);

	periph_ilidriver->host = config->host;
	periph_ilidriver->dma_chnl = config->dma_chnl;
	periph_ilidriver->miso = config->miso;
	periph_ilidriver->mosi = config->mosi;
	periph_ilidriver->clk = config->clk;
	periph_ilidriver->rst = config->rst;
	periph_ilidriver->cs = config->cs;
	periph_ilidriver->dc = config->dc;
	periph_ilidriver->bckl = config->bckl;
	periph_ilidriver->size = config->size;
	periph_ilidriver->rot = config->rot;
	periph_ilidriver->width = width;
	periph_ilidriver->height = height;
	periph_ilidriver->pos_x = 0;
	periph_ilidriver->pos_y = 0;
	periph_ilidriver->lines_idx = 0;
	periph_ilidriver->pause = false;
	periph_ilidriver->is_started = false;

	esp_periph_set_data(periph, periph_ilidriver);
	esp_periph_set_function(periph, _ilidriver_init, _ilidriver_run, _ilidriver_destroy);
	g_ilidriver = periph;

	return periph;
}

esp_err_t periph_ilidriver_fill(esp_periph_handle_t periph, uint32_t color)
{
	VALIDATE_ILIDRIVER(periph, ESP_FAIL);
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(periph);

	uint16_t width = periph_ilidriver->width;
	uint16_t height = periph_ilidriver->height;

	for (int idx = 0; idx < (width * height); idx++) {
		uint8_t *p = periph_ilidriver->buf + idx * 3;

		p[0] = (color >> 16) & 0xFF;
		p[1] = (color >> 8) & 0xFF;
		p[2] = (color >> 0) & 0xFF;
	}

	return ESP_OK;
}

esp_err_t periph_ilidriver_write_char(esp_periph_handle_t periph, font_size_t font_size, uint8_t chr, uint32_t color)
{
	VALIDATE_ILIDRIVER(periph, ESP_FAIL);
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(periph);

	font_t font;
	ILIDRIVER_CHECK(get_font(chr, font_size, &font) > 0, "get font error", return ESP_FAIL);

	uint16_t num_byte_per_row = font.data_len / font.height;
	for (uint16_t height_idx = 0; height_idx < font.height; height_idx ++) {
		for ( uint8_t byte_idx = 0; byte_idx < num_byte_per_row; byte_idx++) {
			for (uint16_t width_idx = 0; width_idx < 8; width_idx++) {
				uint16_t x = periph_ilidriver->pos_x + width_idx + byte_idx * 8;
				uint16_t y = periph_ilidriver->pos_y + height_idx;
				if (((font.data[height_idx * num_byte_per_row + byte_idx] << width_idx) & 0x80) == 0x80) {
					_draw_pixel(x, y, color);
				}
			}
		}
	}
	periph_ilidriver->pos_x += font.width + num_byte_per_row;

	return ESP_OK;
}

esp_err_t periph_ilidriver_write_string(esp_periph_handle_t periph, font_size_t font_size, uint8_t *str, uint32_t color)
{
	VALIDATE_ILIDRIVER(periph, ESP_FAIL);
	ILIDRIVER_CHECK(str, "error string null", return ESP_FAIL);
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(periph);

	while (*str) {
		font_t font;
		ILIDRIVER_CHECK(get_font(*str, font_size, &font) > 0, "get font error", return ESP_FAIL);

		uint16_t num_byte_per_row = font.data_len / font.height;
		for (uint16_t height_idx = 0; height_idx < font.height; height_idx ++) {
			for ( uint16_t byte_idx = 0; byte_idx < num_byte_per_row; byte_idx++) {
				for (uint16_t width_idx = 0; width_idx < 8; width_idx++) {
					uint16_t x = periph_ilidriver->pos_x + width_idx + byte_idx * 8;
					uint16_t y = periph_ilidriver->pos_y + height_idx;
					if (((font.data[height_idx * num_byte_per_row + byte_idx] << width_idx) & 0x80) == 0x80) {
						_draw_pixel(x, y, color);
					}
				}
			}
		}
		periph_ilidriver->pos_x += font.width + num_byte_per_row;
		str++;
	}

	return ESP_OK;
}

esp_err_t periph_ilidriver_draw_pixel(esp_periph_handle_t periph, uint16_t x, uint16_t y, uint32_t color)
{
	VALIDATE_ILIDRIVER(periph, ESP_FAIL);
	_draw_pixel(x, y, color);

	return ESP_OK;
}

esp_err_t periph_ilidriver_draw_line(esp_periph_handle_t periph, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color)
{
	VALIDATE_ILIDRIVER(periph, ESP_FAIL);
	_draw_line(x1, y1, x2, y2, color);

	return ESP_OK;
}

esp_err_t periph_ilidriver_draw_rectangle(esp_periph_handle_t periph, uint16_t x_origin, uint16_t y_origin, uint16_t width, uint16_t height, uint32_t color)
{
	VALIDATE_ILIDRIVER(periph, ESP_FAIL);

	_draw_line(x_origin, y_origin, x_origin + width, y_origin, color);
	_draw_line(x_origin + width, y_origin, x_origin + width, y_origin + height, color);
	_draw_line(x_origin + width, y_origin + height, x_origin, y_origin + height, color);
	_draw_line(x_origin, y_origin + height, x_origin, y_origin, color);

	return ESP_OK;
}

esp_err_t periph_ilidriver_draw_circle(esp_periph_handle_t periph, uint16_t x_origin, uint16_t y_origin, uint16_t radius, uint32_t color)
{
	VALIDATE_ILIDRIVER(periph, ESP_FAIL);

	int32_t x = -radius;
	int32_t y = 0;
	int32_t err = 2 - 2 * radius;
	int32_t e2;

	do {
		_draw_pixel(x_origin - x, y_origin + y, color);
		_draw_pixel(x_origin + x, y_origin + y, color);
		_draw_pixel(x_origin + x, y_origin - y, color);
		_draw_pixel(x_origin - x, y_origin - y, color);

		e2 = err;
		if (e2 <= y) {
			y++;
			err = err + (y * 2 + 1);
			if (-x == y && e2 <= x) {
				e2 = 0;
			}
			else {
				/*nothing to do*/
			}
		} else {
			/*nothing to do*/
		}

		if (e2 > x) {
			x++;
			err = err + (x * 2 + 1);
		} else {
			/*nothing to do*/
		}
	} while (x <= 0);

	return ESP_OK;
}

esp_err_t periph_ilidriver_set_position(esp_periph_handle_t periph, uint16_t x, uint16_t y)
{
	VALIDATE_ILIDRIVER(periph, ESP_FAIL);

	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(g_ilidriver);
	periph_ilidriver->pos_x = x;
	periph_ilidriver->pos_y = y;

	return ESP_OK;
}

esp_err_t periph_ilidriver_get_position(esp_periph_handle_t periph, uint16_t *x, uint16_t *y)
{
	VALIDATE_ILIDRIVER(periph, ESP_FAIL);

	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(g_ilidriver);
	*x = periph_ilidriver->pos_x;
	*y = periph_ilidriver->pos_y;

	return ESP_OK;
}

uint8_t *periph_ilidriver_get_buffer(esp_periph_handle_t periph)
{
	VALIDATE_ILIDRIVER(periph, NULL);
	periph_ilidriver_t *periph_ilidriver = esp_periph_get_data(g_ilidriver);
	return periph_ilidriver->buf;
}