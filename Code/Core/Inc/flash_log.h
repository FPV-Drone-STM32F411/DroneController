#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "w25q16v.h"   // <- your flash driver header (rename if needed)

// --- Choose a region in flash for logs (must be sector-aligned) ---
#define LOG_BASE        0x00000u
#define LOG_SIZE        (4u * W25_SECTOR_SIZE)   // 16 KB -> 4 sectors
#define LOG_END         (LOG_BASE + LOG_SIZE)

// user record “type” tags (make more as you wish)
enum { LOG_TYPE_IMU = 0x01, LOG_TYPE_GPS = 0x02, LOG_TYPE_EVENT = 0x10 };

// API
bool     log_init(void);                                               // find/prepare write pointer
bool     log_write(uint8_t type, const uint8_t* payload, uint16_t len); // append one record
typedef void (*log_iter_cb)(uint8_t type, const uint8_t* p, uint16_t n, uint32_t ts);
void     log_iterate(log_iter_cb cb);                                  // read all valid records

// (optional) expose write pointer for debugging
uint32_t log_get_wp(void);
