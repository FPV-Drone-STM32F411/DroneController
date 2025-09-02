#ifndef W25Q16_H
#define W25Q16_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

// Edit these to match your CubeMX labels/handles:
#define W25_SPI_HANDLE      hspi1
#define W25_CS_GPIO_Port    FLASH_CS_GPIO_Port
#define W25_CS_Pin          FLASH_CS_Pin

HAL_StatusTypeDef W25_ReadJEDEC(uint8_t id[3]);

#endif
