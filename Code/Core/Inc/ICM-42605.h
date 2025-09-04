// ic2_helpers.h
#pragma once
#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stddef.h>

#define ICM_SPI_HANDLE      hspi2
#define ICM_CS_GPIO_Port    NSS2_GPIO_Port   // Adjust if your CS is on another port
#define ICM_CS_Pin          NSS2_Pin

// ---- register addresses (Bank 0) ----
#define REG_WHO_AM_I       0x75
#define REG_PWR_MGMT0      0x4E
#define REG_INTF_CONFIG0   0x4C
#define REG_REG_BANK_SEL   0x76  // Bank select register lives in Bank 0
#define REG_DEVICE_CONFIG   0x11 // Bank 0
#define DEVICE_CONFIG_RESET 0x01 // Bit 0 triggers soft reset
// If not already in your header:
#define REG_PWR_MGMT0        0x4E
#define REG_GYRO_CONFIG0     0x4F
#define REG_ACCEL_CONFIG0    0x50

// Accel data registers (6 bytes total: XH, XL, YH, YL, ZH, ZL)
#define REG_ACCEL_DATA_X_H   0x1F  // adjust if your map uses different names
// (Then Y_H = 0x21, Z_H = 0x23, but we’ll burst from X_H for 6 bytes)

// ---- Bit masks (fill with the exact values from your datasheet if you have them) ----
// PWR_MGMT0: accel mode bits [3:2], gyro mode bits [1:0] (common pattern on ICM-426xx)
#define PWRM_ACCEL_MODE_MASK  0x03      // bits 1:0
#define PWRM_ACCEL_MODE_OFF   0x00      // 00
#define PWRM_ACCEL_MODE_LP    0x02      // 10
#define PWRM_ACCEL_MODE_LN    0x03      // 11

#define PWRM_GYRO_MODE_MASK   0x0C      // bits 3:2
#define PWRM_GYRO_MODE_OFF    0x00      // 00
#define PWRM_GYRO_MODE_STBY   0x04      // 01
#define PWRM_GYRO_MODE_LN     0x0C      // 11



// ACCEL_CONFIG0: FS in [7:5], ODR in [4:0] (typical layout)
#define ACCEL_FS_SHIFT        5
#define ACCEL_ODR_MASK        0x0F
// Gyro FS and ODR bit positions
#define GYRO_FS_SHIFT   5      // FS bits [7:5] in GYRO_CONFIG0
#define GYRO_ODR_MASK   0x0F   // ODR bits [3:0] in GYRO_CONFIG0

#define REG_ACCEL_DATA_X_H   0x1F   // XH, XL, YH, YL, ZH, ZL (6 bytes total)
#define REG_GYRO_DATA_X_H    0x25   // XH, XL, YH, YL, ZH, ZL (6 bytes total)

// ---- bank codes (datasheet shows 0,1,2,4 as valid banks) ----
#define BANK0 0x00
#define BANK1 0x10
#define BANK2 0x20
#define BANK4 0x40

// Config
#define GYRO_FS_2000DPS   0x00  // ±2000 dps
#define ACCEL_FS_16G      0x0
#define ACCEL_FS_8G       0x1
#define ACCEL_FS_4G       0x2
#define ACCEL_FS_2G       0x3

#define ACCEL_ODR_1KHZ    0x06  // <-- fix
#define GYRO_ODR_1KHZ     0x06  // <-- fix

#define REG_GYRO_CONFIG1        0x51  // also holds GYRO_UI_FILT_ORD
#define REG_GYRO_ACCEL_CONFIG0  0x52  // accel[7:4] BW, gyro[3:0] BW
#define REG_ACCEL_CONFIG1       0x53  // ACCEL_UI_FILT_ORD in bits [4:3]


// Bit positions for filter order
#define GYRO_UI_FILT_ORD_SHIFT   2
#define GYRO_UI_FILT_ORD_MASK    (0x3 << GYRO_UI_FILT_ORD_SHIFT)
#define ACCEL_UI_FILT_ORD_SHIFT  3
#define ACCEL_UI_FILT_ORD_MASK   (0x3 << ACCEL_UI_FILT_ORD_SHIFT)

// Bandwidth fields
#define ACCEL_UI_FILT_BW_SHIFT   4
#define ACCEL_UI_FILT_BW_MASK    (0xF << ACCEL_UI_FILT_BW_SHIFT)
#define GYRO_UI_FILT_BW_SHIFT    0
#define GYRO_UI_FILT_BW_MASK     (0xF << GYRO_UI_FILT_BW_SHIFT)


// Accelerometer conversion (±2 g, 16384 LSB/g)
#define ACCEL_LSB_PER_G    16384.0f
#define G_IN_MS2           9.80665f

// Gyro conversion (±2000 dps, 16.4 LSB/°/s)
#define GYRO_LSB_PER_DPS   16.4f


