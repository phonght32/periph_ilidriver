#ifndef _PERIPH_ILIDRIVER_H_
#define _PERIPH_ILIDRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_peripherals.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "fonts.h"

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

#ifdef __cplusplus
}
#endif

#endif /* _PERIPH_ILIDRIVER_H_ */