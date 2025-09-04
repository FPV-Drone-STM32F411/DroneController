#ifndef W25Q16_H
#define W25Q16_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stddef.h>

// Edit these to match your CubeMX labels/handles:
// SPI handle and CS pin
#define W25_SPI_HANDLE      hspi1
#define W25_CS_GPIO_Port    GPIOA   // Adjust if your CS is on another port
#define W25_CS_Pin          NSS1_Pin

// Memory geometry
#define W25_PAGE_SIZE       256u       // 256 bytes per page
#define W25_SECTOR_SIZE     4096u      // 4 KB per sector
#define W25_BLOCK_SIZE      65536u     // 64 KB block size
#define W25_TOTAL_SIZE      (2u * 1024u * 1024u) // 2 MB (16 Mbit)



HAL_StatusTypeDef W25_ReadJEDEC(uint8_t id[3]);





#endif
