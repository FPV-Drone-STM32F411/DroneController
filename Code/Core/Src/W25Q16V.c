#include "w25q16v.h"

static inline void CS_L(void){ HAL_GPIO_WritePin(GPIOB, NSS5_Pin, GPIO_PIN_RESET); }
static inline void CS_H(void){ HAL_GPIO_WritePin(GPIOB, NSS5_Pin, GPIO_PIN_SET); }

HAL_StatusTypeDef W25_ReadJEDEC(uint8_t id[3]){
  uint8_t tx[4] = { 0x9F, 0,0,0 };  // 0x9F = JEDEC ID
  uint8_t rx[4] = { 0 };

  CS_L();
  HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(&W25_SPI_HANDLE, tx, rx, sizeof tx, 100);
  CS_H();

  if (s != HAL_OK) return s;
  id[0] = rx[1];   // manufacturer (Winbond = 0xEF)
  id[1] = rx[2];   // memory type
  id[2] = rx[3];   // capacity code (e.g., 0x15 for 16 Mbit)
  return HAL_OK;
}

// --- helper to pick prescaler nearest to target_hz and (re)init SPI ---
static uint32_t SPI_Reinit_To_Baud(SPI_HandleTypeDef *h, uint32_t target_hz) {
    uint32_t pclk =
        (h->Instance == SPI1) ? HAL_RCC_GetPCLK2Freq() :
        (h->Instance == SPI4) ? HAL_RCC_GetPCLK2Freq() :        // if you ever use SPI4
                                 HAL_RCC_GetPCLK1Freq();         // SPI2/3
    struct { uint32_t br; uint32_t div; } opt[] = {
        {SPI_BAUDRATEPRESCALER_2,   2}, {SPI_BAUDRATEPRESCALER_4,   4},
        {SPI_BAUDRATEPRESCALER_8,   8}, {SPI_BAUDRATEPRESCALER_16, 16},
        {SPI_BAUDRATEPRESCALER_32, 32}, {SPI_BAUDRATEPRESCALER_64, 64},
        {SPI_BAUDRATEPRESCALER_128,128},{SPI_BAUDRATEPRESCALER_256,256},
    };
    uint32_t best_i = 0, best_err = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < sizeof(opt)/sizeof(opt[0]); ++i) {
        uint32_t f = pclk / opt[i].div;
        uint32_t err = (f > target_hz) ? (f - target_hz) : (target_hz - f);
        if (err < best_err) { best_err = err; best_i = i; }
    }
    if (h->Init.BaudRatePrescaler != opt[best_i].br) {
        HAL_SPI_DeInit(h);
        h->Init.BaudRatePrescaler = opt[best_i].br;
        HAL_SPI_Init(h);
    }
    return pclk / opt[best_i].div;   // actual SCK
}

// Output a visible SPI clock on SCK for 'duration_ms' (CS stays HIGH)
void W25_OutputClock(uint32_t target_hz, uint32_t duration_ms) {
    uint32_t actual = SPI_Reinit_To_Baud(&W25_SPI_HANDLE, target_hz);

    // Make sure CS is high so the flash ignores clocks
    CS_H();

    static uint8_t dummy[64] = {0xFF};   // contents don't matter
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < duration_ms) {
        HAL_SPI_Transmit(&W25_SPI_HANDLE, dummy, sizeof dummy, HAL_MAX_DELAY);
    }
    printf("[W25] SCK ~= %lu Hz\r\n", (unsigned long)actual);
}

/// Global Unlock
// Removes Write Protection
// Ran after JEDEC
//tells the w25 i want to write (mandatory before all comm)
void W25_SendWrite(void){
	uint8_t cmd = 0x06;
	CS_L();
	//handle, size (byte), timeout
	HAL_SPI_Transmit(&W25_SPI_HANDLE,&cmd, 1, 50);
	CS_H();
}

void W25_BusySense(void){
	//read if busy
	uint8_t cmd = 0x05;
	uint8_t status;
	do {
		CS_L();
		//handle, size (byte), timeout
		HAL_SPI_Transmit(&W25_SPI_HANDLE, &cmd, 1, 50);
		HAL_SPI_Recieve(&W25_SPI_HANDLE, &status, 1, 50);
		CS_H();
	}
	while (status == 0x01);
	printf("DEBUG: not busy");
}

void W25_GlobalUnlock(void){
  W25_SendWrite();
  //clear wp command
  CS_L();
  uint8_t cmd = 0x98;
  CS_H();
  W25_BusySense();
}

// Safe write
HAL_StatusTypeDef W25_Write(uint32_t addr, const uint8_t* data, uint32_t len){
  while (len){
    uint16_t page_off = addr & 0xFF;
    uint16_t chunk = W25_PAGE_SIZE - page_off;
    if (chunk > len) chunk = len;
    HAL_StatusTypeDef s = W25_PageProgram(addr, data, chunk);
    if (s != HAL_OK) return s;
    addr += chunk; data += chunk; len -= chunk;
  }
  return HAL_OK;
}

//Logging


