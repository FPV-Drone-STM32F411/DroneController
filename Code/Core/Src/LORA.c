// sx1276_min.c — minimal SPI read/write + a couple of sanity checks
// Target: STM32F411 (CubeIDE, HAL) + NiceRF LoRa1276‑C1‑915 (SX1276)
// Wiring per user: SPI1, NSS1 (GPIOA), all DIOx NC, RESET optional
// Use: drop this single file into Core/Src (or split as you like). Call demo() from main.
//
// What this provides:
//  - chip‑select control (software NSS)
//  - single‑byte register read/write, burst read/write
//  - read RegVersion (0x42) == 0x12 check
//  - enter LoRa mode correctly (toggle LongRangeMode in SLEEP)
//  - set RF frequency to 915 MHz using FRF formula (RegFrfMsb/Mid/Lsb)
//
// References (SX1276 datasheet):
//  - SPI: CPOL=0, CPHA=0; address MSB = 1 for write, 0 for read (wnr bit)
//  - RegVersion (0x42) reads 0x12 on SX1276/7/8/9 silicon
//  - RegOpMode.LongRangeMode bit writable only in SLEEP mode
//  - FRF = Freq * 2^19 / FXOSC (FXOSC = 32 MHz). Write LSB last to latch
//
// NOTE: This is only to verify SPI wiring and basic register access. No TX/RX; antenna not required.

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ---- Choose your SPI and NSS pin here ----
extern SPI_HandleTypeDef hspi1; // CubeMX provides this for SPI1
#define SX_SPI         (&hspi1)
#define SX_NSS_PORT    GPIOA  // change if your CS pin macro differs
#define SX_NSS_PIN     NSS1_Pin        // change if your CS pin macro differs

// ---- Key SX1276 registers we touch ----
#define REG_FIFO           0x00
#define REG_OPMODE         0x01
#define REG_FRF_MSB        0x06
#define REG_FRF_MID        0x07
#define REG_FRF_LSB        0x08
#define REG_VERSION        0x42

// OpMode bits
#define OPMODE_LONG_RANGE  0x80 // bit7
#define OPMODE_MODE_MASK   0x07
#define OPMODE_SLEEP       0x00
#define OPMODE_STDBY       0x01

static inline void sx_cs_low(void){ HAL_GPIO_WritePin(GPIOA, NSS1_Pin, GPIO_PIN_RESET); }
static inline void sx_cs_high(void){ HAL_GPIO_WritePin(GPIOA, NSS1_Pin, GPIO_PIN_SET); }
#include <stdio.h>
#include <stdarg.h>

// Unbuffer stdout so prints appear immediately
static void log_init(void){
    setvbuf(stdout, NULL, _IONBF, 0);
}

// Our logger used by sx_*
void sx_log(const char *fmt, ...) {
    char buf[160];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) _write(1, buf, n);
}

// SPI single register read (wnr=0)
uint8_t sx_read(uint8_t addr){
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0x00 };
    uint8_t rx[2] = {0};
    sx_cs_low();
    HAL_SPI_TransmitReceive(SX_SPI, tx, rx, 2, HAL_MAX_DELAY);
    sx_cs_high();
    return rx[1];
}

// SPI single register write (wnr=1)
void sx_write(uint8_t addr, uint8_t val){
    uint8_t tx[2] = { (uint8_t)(addr | 0x80), val };
    sx_cs_low();
    HAL_SPI_Transmit(SX_SPI, tx, 2, HAL_MAX_DELAY);
    sx_cs_high();
}

// Burst write/read (auto‑inc addressing for non‑FIFO)
void sx_burst_write(uint8_t addr, const uint8_t *buf, uint8_t len){
    uint8_t a = addr | 0x80;
    sx_cs_low();
    HAL_SPI_Transmit(SX_SPI, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(SX_SPI, (uint8_t*)buf, len, HAL_MAX_DELAY);
    sx_cs_high();
}

void sx_burst_read(uint8_t addr, uint8_t *buf, uint8_t len){
    uint8_t a = addr & 0x7F;
    sx_cs_low();
    HAL_SPI_Transmit(SX_SPI, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(SX_SPI, buf, len, HAL_MAX_DELAY);
    sx_cs_high();
}

// ---- Small helpers for bring‑up ----
// Put device into LoRa mode correctly: must toggle in SLEEP
void sx_enter_lora_mode(void){
    // go to FSK sleep first
    sx_write(REG_OPMODE, OPMODE_SLEEP);
    // set LoRa bit while staying in sleep
    sx_write(REG_OPMODE, OPMODE_LONG_RANGE | OPMODE_SLEEP);
    // standby in LoRa
    uint8_t v = sx_read(REG_OPMODE);
    v = (uint8_t)((v & ~OPMODE_MODE_MASK) | OPMODE_STDBY);
    sx_write(REG_OPMODE, v);
}

// Set RF center frequency using FRF = Freq * 2^19 / 32e6
void sx_set_freq_hz(uint32_t freq_hz){
    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000UL; // FXOSC=32MHz
    sx_write(REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx_write(REG_FRF_MID, (uint8_t)(frf >> 8));
    sx_write(REG_FRF_LSB, (uint8_t)(frf)); // write LSB last to latch
}

// Quick sanity: read RegVersion (expect 0x12)
bool sx_detect(void){
    uint8_t v = sx_read(REG_VERSION);
    return (v == 0x12);
}


void sx_dump(uint8_t start, uint8_t end){
    for(uint8_t a = start; a <= end; ++a){
        uint8_t v = sx_read(a);
        if (sx_log) sx_log("0x%02X : 0x%02X\r\n", a, v);
    }
}

// ---- Minimal demo you can call from main() ----
void sx1276_min_demo(void){
	log_init();
	// Probe with CS forced HIGH (should NOT return 0x12)
	HAL_GPIO_WritePin(SX_NSS_PORT, SX_NSS_PIN, GPIO_PIN_SET);
	uint8_t v_cs_high = sx_read(REG_VERSION);
	sx_log("[SX] RegVersion with CS HIGH = 0x%02X (expect junk)\r\n", v_cs_high);

	// Now normal read with CS toggled in sx_read()
	uint8_t v = sx_read(REG_VERSION);
	sx_log("[SX] RegVersion with CS toggled = 0x%02X (expect 0x12)\r\n", v);

	// ensure CS idle high before any SPI frames
    HAL_GPIO_WritePin(SX_NSS_PORT, SX_NSS_PIN, GPIO_PIN_SET);

    // 1) version check
    uint8_t ver = sx_read(REG_VERSION);
    if (sx_log) sx_log("SX1276 RegVersion=0x%02X (expect 0x12)\r\n", ver);

    // 2) enter LoRa mode and standby
    sx_enter_lora_mode();

    // 3) set 915 MHz and read back FRF regs
    sx_set_freq_hz(915000000UL);
    uint8_t msb = sx_read(REG_FRF_MSB);
    uint8_t mid = sx_read(REG_FRF_MID);
    uint8_t lsb = sx_read(REG_FRF_LSB);
    if (sx_log) sx_log("FRF set -> MSB=0x%02X MID=0x%02X LSB=0x%02X\r\n", msb, mid, lsb);

    // 4) optional: dump a small window around key regs
    sx_dump(0x01, 0x1F);
}

// Example of a tiny logger to printf (uncomment if you want UART logging)
// int _write(int file, char *ptr, int len); // if you use retargeted printf
// void sx_log(const char *fmt, ...){
//     char buf[128];
//     va_list ap; va_start(ap, fmt);
//     int n = vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
//     if (n < 0) return; if (n > (int)sizeof(buf)) n = sizeof(buf);
//     // send via ITM/SWO or UART; below uses semihosting/ITM if enabled
//     for (int i=0;i<n;i++){ ITM_SendChar(buf[i]); }
// }
