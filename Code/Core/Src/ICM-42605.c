#include "ICM-42605.h"

static inline void CS_L(void){ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); }
static inline void CS_H(void){ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); }
// Combines two bytes into a signed values
static inline int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

//send address first, then change value
// 1) SPI primitives
HAL_StatusTypeDef icm42605_bank_select(uint8_t bankval)
{
	HAL_StatusTypeDef st;
	 uint8_t bank_code;
	switch (bankval) {
	        case 0:
	            bank_code = BANK0;
	            break;
	        case 1:
	            bank_code = BANK1;
	            break;
	        case 2:
	            bank_code = BANK2;
	            break;
	        case 4:
	            bank_code = BANK4;
	            break;
	        default:
	            return HAL_ERROR; // invalid bank number
	    }

	uint8_t addr= 0x76 & 0x7F;
	CS_L();
	st = HAL_SPI_Transmit(&ICM_SPI_HANDLE, &addr, 1, 50);
	if (st == HAL_OK) {
	st = HAL_SPI_Transmit(&ICM_SPI_HANDLE, &bank_code, 1, 50);
		}
	CS_H();
	return st;
}

HAL_StatusTypeDef icm42605_write_reg(uint8_t bank, uint8_t reg, uint8_t val)
{
	// select bank
		HAL_StatusTypeDef st;
		    // 1) select bank
		    st = icm42605_bank_select(bank);
		    if (st != HAL_OK) return st;
	// selects 7 bit address from register

	uint8_t addr = reg & 0x7F; // MSB = 0 so write
	CS_L();
	// send address
	st = HAL_SPI_Transmit(&ICM_SPI_HANDLE, &addr, 1, 50);
	// write new value
	if (st == HAL_OK) {
		st = HAL_SPI_Transmit(&ICM_SPI_HANDLE, &val, 1, 50);
	}
	CS_H();
	return st;
}
HAL_StatusTypeDef icm42605_read_reg (uint8_t bank, uint8_t reg, uint8_t *val)
{
	HAL_StatusTypeDef st;
	    // 1) select bank
	    st = icm42605_bank_select(bank);
	    if (st != HAL_OK) return st;
	uint8_t addr = reg | 0x80;
	CS_L();
		// send address
		st = HAL_SPI_Transmit(&ICM_SPI_HANDLE, &addr, 1, 50);
		// read/reassign value
		if (st == (HAL_OK)) {
			st = HAL_SPI_Receive(&ICM_SPI_HANDLE, val, 1, 50);
		}
	CS_H();
	return st;
}


HAL_StatusTypeDef icm42605_read_multi(uint8_t bank, uint8_t start_reg, uint8_t *buf, size_t n)
{
    if (n == 0 || buf == NULL) return HAL_OK;

    HAL_StatusTypeDef st = icm42605_bank_select(bank);
    if (st != HAL_OK) return st;

    uint8_t cmd = (uint8_t)(start_reg | 0x80); // MSB=1 → read
    CS_L();
    st = HAL_SPI_Transmit(&ICM_SPI_HANDLE, &cmd, 1, 50);
    if (st == HAL_OK) {
        st = HAL_SPI_Receive(&ICM_SPI_HANDLE, buf, (uint16_t)n, 50);
    }
    CS_H();
    return st;
}


// 2. Core Functions
HAL_StatusTypeDef icm42605_reset(void)
{
    HAL_StatusTypeDef st;

    // Write reset command
    CS_L();
    st = icm42605_write_reg(0, REG_DEVICE_CONFIG, DEVICE_CONFIG_RESET);
    if (st != HAL_OK) return st;

    // Wait for reset to complete (datasheet recommends ~10ms)
    HAL_Delay(10);
    CS_H();
    return HAL_OK;
}

// 3. Sensor Config/setup
HAL_StatusTypeDef icm42605_config_accel(uint8_t fs_sel, uint8_t odr_sel)
{
    HAL_StatusTypeDef st;
    uint8_t v;

    // 1) Set FS + ODR (ACCEL_CONFIG0: FS [7:5], ODR [3:0])
    v = (uint8_t)((fs_sel << ACCEL_FS_SHIFT) | (odr_sel & ACCEL_ODR_MASK));
    st = icm42605_write_reg(0, REG_ACCEL_CONFIG0, v);
    if (st != HAL_OK) return st;   // ACCEL_CONFIG0 @ 0x50. :contentReference[oaicite:3]{index=3}

    // 2) Power accel ON in PWR_MGMT0 → Low-Noise (LN)
    st = icm42605_read_reg(0, REG_PWR_MGMT0, &v); if (st != HAL_OK) return st;
    v = (uint8_t)((v & ~PWRM_ACCEL_MODE_MASK) | PWRM_ACCEL_MODE_LN);
    st = icm42605_write_reg(0, REG_PWR_MGMT0, v); if (st != HAL_OK) return st;

    // Datasheet: after OFF→other modes, avoid register writes for ~200 µs.
    HAL_Delay(1);  // 1 ms > 200 µs, safe. :contentReference[oaicite:4]{index=4}

    // 3) Set ACCEL UI filter ORDER = 2nd (ACCEL_CONFIG1, bits [4:3] = 01)
    st = icm42605_read_reg(0, REG_ACCEL_CONFIG1, &v); if (st != HAL_OK) return st;
    v = (uint8_t)((v & ~(0b11 << 3)) | (1u << 3));  // 01 = 2nd order
    st = icm42605_write_reg(0, REG_ACCEL_CONFIG1, v); if (st != HAL_OK) return st;
    // (ACCEL_CONFIG1 @ 0x53; “00=1st, 01=2nd, 10=3rd”). :contentReference[oaicite:5]{index=5}

    // 4) Set LPF bandwidths (GYRO_ACCEL_CONFIG0 @ 0x52)
    //    Put accel BW code in [7:4] and (optionally) leave gyro BW as-is.
    st = icm42605_read_reg(0, REG_GYRO_ACCEL_CONFIG0, &v); if (st != HAL_OK) return st;
    v = (uint8_t)((v & 0x0F) | (uint8_t)(6u << 4));    // accel BW code = 6 → ODR/20
    st = icm42605_write_reg(0, REG_GYRO_ACCEL_CONFIG0, v); if (st != HAL_OK) return st;
    // (BW code 6 gives ≈ ODR/20 in LN mode; at 1 kHz → ~50 Hz cutoff). :contentReference[oaicite:6]{index=6}

    // small settle (optional)
    HAL_Delay(5);

    return HAL_OK;
}

HAL_StatusTypeDef icm42605_config_gyro(uint8_t fs_sel, uint8_t odr_sel)
{
    HAL_StatusTypeDef st;
    uint8_t v;

    // 1) Set FS + ODR (GYRO_CONFIG0: FS[7:5], ODR[3:0])
    v = (uint8_t)((fs_sel << GYRO_FS_SHIFT) | (odr_sel & GYRO_ODR_MASK));
    st = icm42605_write_reg(0, REG_GYRO_CONFIG0, v);
    if (st != HAL_OK) return st;

    // 2) Power gyro ON in PWR_MGMT0 → Low-Noise (LN)
    st = icm42605_read_reg(0, REG_PWR_MGMT0, &v);
    if (st != HAL_OK) return st;
    v = (uint8_t)((v & ~PWRM_GYRO_MODE_MASK) | PWRM_GYRO_MODE_LN);
    st = icm42605_write_reg(0, REG_PWR_MGMT0, v);
    if (st != HAL_OK) return st;

    // datasheet: wait at least 200 µs after mode change before writes
    HAL_Delay(1);

    // 3) Set GYRO filter order = 2nd (GYRO_CONFIG1, bits [3:2] = 01)
    st = icm42605_read_reg(0, REG_GYRO_CONFIG1, &v);
    if (st != HAL_OK) return st;
    v = (uint8_t)((v & ~(0b11 << 2)) | (1u << 2));
    st = icm42605_write_reg(0, REG_GYRO_CONFIG1, v);
    if (st != HAL_OK) return st;

    // 4) Set GYRO LPF bandwidth = BW code 6 (~ODR/20)
    st = icm42605_read_reg(0, REG_GYRO_ACCEL_CONFIG0, &v);
    if (st != HAL_OK) return st;
    v = (uint8_t)((v & 0xF0) | (6u & 0x0F));   // lower nibble = gyro BW
    st = icm42605_write_reg(0, REG_GYRO_ACCEL_CONFIG0, v);
    if (st != HAL_OK) return st;

    HAL_Delay(5);

    return HAL_OK;
}

// 3) Data

HAL_StatusTypeDef icm42605_read_accel(int16_t xyz[3])
{

    uint8_t raw[6];
    HAL_StatusTypeDef st = icm42605_read_multi(0, REG_ACCEL_DATA_X_H, raw, 6);
    if (st != HAL_OK) return st;
    // casts two bytes into one
    xyz[0] = be16(&raw[0]);
    xyz[1] = be16(&raw[2]);
    xyz[2] = be16(&raw[4]);
    return HAL_OK;

}

HAL_StatusTypeDef icm42605_read_gyro (int16_t xyz[3])
{

uint8_t raw[6];
    HAL_StatusTypeDef st = icm42605_read_multi(0, REG_GYRO_DATA_X_H, raw, 6);
    if (st != HAL_OK) return st;
    xyz[0] = be16(&raw[0]);
    xyz[1] = be16(&raw[2]);
    xyz[2] = be16(&raw[4]);
    return HAL_OK;

}

