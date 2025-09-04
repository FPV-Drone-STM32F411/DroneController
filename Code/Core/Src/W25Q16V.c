#include "w25q16v.h"

static inline void CS_L(void){ HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); }
static inline void CS_H(void){ HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); }

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


