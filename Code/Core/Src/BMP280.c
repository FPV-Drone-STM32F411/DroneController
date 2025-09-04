#include "bmp280.h"
#include <string.h>
#include <math.h>

/* ---- local helpers ---- */
static inline void CS_L(BMP280_HandleTypeDef *dev) { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); }
static inline void CS_H(BMP280_HandleTypeDef *dev) { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET); }

/* SPI: in BMP280, MSB of address is replaced by R/W bit: 1=read, 0=write */
static HAL_StatusTypeDef bmp280_write_reg(BMP280_HandleTypeDef *dev, uint8_t reg, uint8_t val) {
  uint8_t tx[2];
  tx[0] = (reg & 0x7F); // RW=0
  tx[1] = val;
  CS_L(dev);
  HAL_StatusTypeDef st = HAL_SPI_Transmit(dev->hspi, tx, sizeof(tx), HAL_MAX_DELAY);
  CS_H(dev);
  return st;
}

static HAL_StatusTypeDef bmp280_read_regs(BMP280_HandleTypeDef *dev, uint8_t start_reg, uint8_t *dst, uint16_t len) {
  uint8_t cmd = (start_reg | 0x80); // RW=1
  CS_L(dev);
  HAL_StatusTypeDef st = HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
  if (st == HAL_OK) {
    st = HAL_SPI_Receive(dev->hspi, dst, len, HAL_MAX_DELAY); // auto-increments
  }
  CS_H(dev);
  return st;
}

/* ---- public API ---- */
HAL_StatusTypeDef bmp280_init_spi(BMP280_HandleTypeDef *dev,
                                  SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  memset(dev, 0, sizeof(*dev));
  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;

  /* optional: pull CS high */
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

  uint8_t id = 0;
  HAL_StatusTypeDef st = bmp280_read_id(dev, &id);
  if (st != HAL_OK) return st;
  if (id != BMP280_CHIP_ID) return HAL_ERROR;

  /* soft reset and wait for NVM copy to finish */
  st = bmp280_soft_reset(dev);
  if (st != HAL_OK) return st;

  /* read factory calibration */
  return bmp280_read_calibration(dev);
}

HAL_StatusTypeDef bmp280_soft_reset(BMP280_HandleTypeDef *dev) {
  HAL_StatusTypeDef st = bmp280_write_reg(dev, BMP280_REG_RESET, BMP280_RESET_VALUE);
  if (st != HAL_OK) return st;
  HAL_Delay(4); // t_startup is small, but give a few ms
  /* wait for NVM copy (im_update=1 -> 0) */
  uint32_t t0 = HAL_GetTick();
  for (;;) {
    uint8_t status = 0;
    if (bmp280_read_regs(dev, BMP280_REG_STATUS, &status, 1) != HAL_OK) return HAL_ERROR;
    if ((status & BMP280_STATUS_IM_UPDATE_Msk) == 0) break;
    if ((HAL_GetTick() - t0) > 20) break; // small guard timeout
  }
  return HAL_OK;
}

HAL_StatusTypeDef bmp280_read_id(BMP280_HandleTypeDef *dev, uint8_t *id_out) {
  return bmp280_read_regs(dev, BMP280_REG_ID, id_out, 1);
}

HAL_StatusTypeDef bmp280_read_calibration(BMP280_HandleTypeDef *dev) {
  /* dig_T1..dig_P9 occupy 0x88..0x9F (24 bytes) */
  uint8_t buf[24];
  HAL_StatusTypeDef st = bmp280_read_regs(dev, 0x88, buf, sizeof(buf));
  if (st != HAL_OK) return st;

  dev->dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
  dev->dig_T2 = (int16_t) (buf[3] << 8 | buf[2]);
  dev->dig_T3 = (int16_t) (buf[5] << 8 | buf[4]);

  dev->dig_P1 = (uint16_t)(buf[7] << 8 | buf[6]);
  dev->dig_P2 = (int16_t) (buf[9] << 8 | buf[8]);
  dev->dig_P3 = (int16_t) (buf[11] << 8 | buf[10]);
  dev->dig_P4 = (int16_t) (buf[13] << 8 | buf[12]);
  dev->dig_P5 = (int16_t) (buf[15] << 8 | buf[14]);
  dev->dig_P6 = (int16_t) (buf[17] << 8 | buf[16]);
  dev->dig_P7 = (int16_t) (buf[19] << 8 | buf[18]);
  dev->dig_P8 = (int16_t) (buf[21] << 8 | buf[20]);
  dev->dig_P9 = (int16_t) (buf[23] << 8 | buf[22]);

  return HAL_OK;
}

HAL_StatusTypeDef bmp280_set_config(BMP280_HandleTypeDef *dev,
                                    bmp280_tstandby_t t_sb,
                                    bmp280_filter_t filter,
                                    bool spi3w_enable)
{
  uint8_t cfg = ((uint8_t)t_sb & 0x7) << 5
              | ((uint8_t)filter & 0x7) << 2
              | (spi3w_enable ? 1U : 0U);
  return bmp280_write_reg(dev, BMP280_REG_CONFIG, cfg);
}

HAL_StatusTypeDef bmp280_set_ctrl_meas(BMP280_HandleTypeDef *dev,
                                       bmp280_osrs_t osrs_t,
                                       bmp280_osrs_t osrs_p,
                                       bmp280_mode_t mode)
{
  uint8_t v = ((uint8_t)osrs_t & 0x7) << 5
            | ((uint8_t)osrs_p & 0x7) << 2
            | ((uint8_t)mode    & 0x3);
  return bmp280_write_reg(dev, BMP280_REG_CTRL_MEAS, v);
}

HAL_StatusTypeDef bmp280_force_measurement(BMP280_HandleTypeDef *dev, uint32_t timeout_ms) {
  /* set mode to FORCED leaving oversampling as configured */
  uint8_t ctrl;
  if (bmp280_read_regs(dev, BMP280_REG_CTRL_MEAS, &ctrl, 1) != HAL_OK) return HAL_ERROR;
  ctrl = (ctrl & ~0x03) | BMP280_MODE_FORCED;
  if (bmp280_write_reg(dev, BMP280_REG_CTRL_MEAS, ctrl) != HAL_OK) return HAL_ERROR;

  /* wait for measuring bit to clear */
  uint32_t t0 = HAL_GetTick();
  for (;;) {
    uint8_t status = 0;
    if (bmp280_read_regs(dev, BMP280_REG_STATUS, &status, 1) != HAL_OK) return HAL_ERROR;
    if ((status & BMP280_STATUS_MEASURING_Msk) == 0) break;
    if ((HAL_GetTick() - t0) > timeout_ms) return HAL_TIMEOUT;
  }
  return HAL_OK;
}

HAL_StatusTypeDef bmp280_read_raw(BMP280_HandleTypeDef *dev, int32_t *adc_T, int32_t *adc_P) {
  uint8_t buf[6];
  HAL_StatusTypeDef st = bmp280_read_regs(dev, BMP280_REG_PRESS_MSB, buf, sizeof(buf));
  if (st != HAL_OK) return st;

  int32_t up = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((buf[2] >> 4) & 0x0F);
  int32_t ut = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((buf[5] >> 4) & 0x0F);

  if (adc_P) *adc_P = up;
  if (adc_T) *adc_T = ut;
  return HAL_OK;
}

/* ---- Bosch fixed-point compensation (exactly as in datasheet) ---- */

int32_t bmp280_compensate_T_int32(BMP280_HandleTypeDef *dev, int32_t adc_T) {
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) *
            ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >> 12) * ((int32_t)dev->dig_T3)) >> 14;
  dev->t_fine = var1 + var2;
  T = (dev->t_fine * 5 + 128) >> 8;  // 0.01 °C
  return T;
}

uint32_t bmp280_compensate_P_int64(BMP280_HandleTypeDef *dev, int32_t adc_P) {
  int64_t var1, var2, p;
  var1 = ((int64_t)dev->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dev->dig_P6;
  var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
  var2 = var2 + (((int64_t)dev->dig_P4) << 35);
  var1 = (((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12));
  var1 = (((((int64_t)1) << 47) + var1) * (int64_t)dev->dig_P1) >> 33;
  if (var1 == 0) return 0;  // avoid div by zero
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = ((int64_t)dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = ((int64_t)dev->dig_P8 * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);
  return (uint32_t)p; // Q24.8 (Pa)
}

/* ---- Convenience wrappers ---- */

HAL_StatusTypeDef bmp280_read_temperature_c(BMP280_HandleTypeDef *dev, float *temp_c) {
  int32_t adc_T;
  if (bmp280_read_raw(dev, &adc_T, NULL) != HAL_OK) return HAL_ERROR;
  int32_t T_01deg = bmp280_compensate_T_int32(dev, adc_T);
  if (temp_c) *temp_c = (float)T_01deg / 100.0f;
  return HAL_OK;
}

HAL_StatusTypeDef bmp280_read_pressure_pa(BMP280_HandleTypeDef *dev, float *press_pa) {
  int32_t adc_T, adc_P;
  if (bmp280_read_raw(dev, &adc_T, &adc_P) != HAL_OK) return HAL_ERROR;
  (void)bmp280_compensate_T_int32(dev, adc_T); // updates t_fine
  uint32_t p_q24_8 = bmp280_compensate_P_int64(dev, adc_P);
  if (press_pa) *press_pa = ((float)p_q24_8) / 256.0f;
  return HAL_OK;
}

HAL_StatusTypeDef bmp280_read_temp_press(BMP280_HandleTypeDef *dev, float *temp_c, float *press_pa) {
  int32_t adc_T, adc_P;
  if (bmp280_read_raw(dev, &adc_T, &adc_P) != HAL_OK) return HAL_ERROR;
  int32_t T_01deg = bmp280_compensate_T_int32(dev, adc_T);
  uint32_t p_q24_8 = bmp280_compensate_P_int64(dev, adc_P);
  if (temp_c)  *temp_c  = (float)T_01deg / 100.0f;
  if (press_pa) *press_pa = (float)p_q24_8 / 256.0f;
  return HAL_OK;
}
