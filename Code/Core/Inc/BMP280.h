#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- Registers & constants (from datasheet) ---- */
#define BMP280_REG_ID          0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7  // F7..F9 (20-bit)
#define BMP280_REG_TEMP_MSB    0xFA  // FA..FC (20-bit)
#define BMP280_CHIP_ID         0x58
#define BMP280_RESET_VALUE     0xB6

/* Status bits */
#define BMP280_STATUS_MEASURING_Msk  (1U << 3)
#define BMP280_STATUS_IM_UPDATE_Msk  (1U << 0)

/* ctrl_meas fields */
typedef enum {
  BMP280_OSRS_SKIPPED = 0,
  BMP280_OSRS_X1      = 1,
  BMP280_OSRS_X2      = 2,
  BMP280_OSRS_X4      = 3,
  BMP280_OSRS_X8      = 4,
  BMP280_OSRS_X16     = 5,
} bmp280_osrs_t;

typedef enum {
  BMP280_MODE_SLEEP  = 0x00,
  BMP280_MODE_FORCED = 0x01,     // (0x01 or 0x02 are both “forced”)
  BMP280_MODE_NORMAL = 0x03,
} bmp280_mode_t;

/* config fields */
typedef enum {
  BMP280_FILTER_OFF = 0,  // coeff 0 (off)
  BMP280_FILTER_2   = 1,
  BMP280_FILTER_4   = 2,
  BMP280_FILTER_8   = 3,
  BMP280_FILTER_16  = 4,
} bmp280_filter_t;

typedef enum {
  BMP280_TSB_0_5_MS  = 0,
  BMP280_TSB_62_5_MS = 1,
  BMP280_TSB_125_MS  = 2,
  BMP280_TSB_250_MS  = 3,
  BMP280_TSB_500_MS  = 4,
  BMP280_TSB_1000_MS = 5,
  BMP280_TSB_2000_MS = 6,
  BMP280_TSB_4000_MS = 7,
} bmp280_tstandby_t;

/* ---- Device handle ---- */
typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;

  /* calibration (from NVM) */
  uint16_t dig_T1;
  int16_t  dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

  int32_t t_fine; // carries temperature fine resolution for pressure compensation
} BMP280_HandleTypeDef;

/* ---- API ---- */
HAL_StatusTypeDef bmp280_init_spi(BMP280_HandleTypeDef *dev,
                                  SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin);

HAL_StatusTypeDef bmp280_soft_reset(BMP280_HandleTypeDef *dev);
HAL_StatusTypeDef bmp280_read_id(BMP280_HandleTypeDef *dev, uint8_t *id_out);
HAL_StatusTypeDef bmp280_read_calibration(BMP280_HandleTypeDef *dev);

/* Configuration helpers */
HAL_StatusTypeDef bmp280_set_config(BMP280_HandleTypeDef *dev,
                                    bmp280_tstandby_t t_sb,
                                    bmp280_filter_t filter,
                                    bool spi3w_enable);

HAL_StatusTypeDef bmp280_set_ctrl_meas(BMP280_HandleTypeDef *dev,
                                       bmp280_osrs_t osrs_t,
                                       bmp280_osrs_t osrs_p,
                                       bmp280_mode_t mode);

/* Data path */
HAL_StatusTypeDef bmp280_force_measurement(BMP280_HandleTypeDef *dev, uint32_t timeout_ms);
HAL_StatusTypeDef bmp280_read_raw(BMP280_HandleTypeDef *dev, int32_t *adc_T, int32_t *adc_P);

/* Compensation (fixed-point per Bosch) */
int32_t  bmp280_compensate_T_int32(BMP280_HandleTypeDef *dev, int32_t adc_T); // 0.01 °C
uint32_t bmp280_compensate_P_int64(BMP280_HandleTypeDef *dev, int32_t adc_P); // Q24.8 Pa

/* Convenience wrappers */
HAL_StatusTypeDef bmp280_read_temperature_c(BMP280_HandleTypeDef *dev, float *temp_c);
HAL_StatusTypeDef bmp280_read_pressure_pa(BMP280_HandleTypeDef *dev, float *press_pa);
HAL_StatusTypeDef bmp280_read_temp_press(BMP280_HandleTypeDef *dev, float *temp_c, float *press_pa);

/* Altitude helper (simple barometric formula) */
static inline float bmp280_altitude_m_from_pa(float pressure_pa, float sea_level_pa) {
  /*  standard atmosphere: h = 44330*(1 - (P/P0)^(1/5.255))  */
  float ratio = pressure_pa / sea_level_pa;
  /* powf needs <math.h> in your C file */
  extern float powf(float, float);
  return 44330.0f * (1.0f - powf(ratio, 0.1903f));
}
