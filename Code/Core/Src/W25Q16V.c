#include "w25q16v.h"

extern SPI_HandleTypeDef W25_SPI_HANDLE;

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
