#include "GPS.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Debug control
#ifndef GPS_DEBUG_ENABLED
#define GPS_DEBUG_ENABLED 1
#endif

#if GPS_DEBUG_ENABLED
  #define GPS_DEBUG(fmt, ...)  printf("[GPS] " fmt, ##__VA_ARGS__)
#else
  #define GPS_DEBUG(fmt, ...)
#endif

// Global statistics for debugging
volatile uint32_t gps_irq_events = 0;
volatile uint32_t gps_rx_total = 0;

// Single instance pointer for ISR callbacks
static GPS_Handle_t *s_gps = NULL;

// ===== Private Helper Functions =====

static inline uint16_t rb_next(uint16_t i) {
    return (uint16_t)((i + 1u) % GPS_RX_BUFFER_SIZE);
}

static void rb_put(GPS_Handle_t *g, uint8_t b) {
    uint16_t n = rb_next(g->rb_head);
    if (n != g->rb_tail) {
        g->rb[g->rb_head] = b;
        g->rb_head = n;
    }
    // Note: Silently drops on overflow
}

static bool rb_get(GPS_Handle_t *g, uint8_t *b) {
    if (g->rb_tail == g->rb_head) return false;  // empty
    *b = g->rb[g->rb_tail];
    g->rb_tail = rb_next(g->rb_tail);
    return true;
}

static int hex_nib(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
}

// Calculate NMEA checksum (XOR between '$' and '*')
static uint8_t nmea_checksum(const char *sentence) {
    uint8_t checksum = 0;
    // Start after '$', stop at '*' or end
    for (const char *p = sentence + 1; *p && *p != '*'; ++p) {
        checksum ^= *p;
    }
    return checksum;
}

// Parse common NMEA fields
static bool parse_nmea_field(const char **ptr, char *buffer, size_t buf_size) {
    if (!ptr || !*ptr || !buffer) return false;

    const char *start = *ptr;
    const char *end = strchr(start, ',');
    if (!end) end = strchr(start, '*');  // Last field before checksum
    if (!end) end = start + strlen(start);  // End of string

    size_t len = (size_t)(end - start);
    if (len >= buf_size) len = buf_size - 1;

    memcpy(buffer, start, len);
    buffer[len] = '\0';

    *ptr = (*end == ',') ? end + 1 : end;
    return len > 0;
}

static double parse_coord(const char *coord_str, const char *dir_str) {
    if (!coord_str || !dir_str || strlen(coord_str) < 4) return 0.0;

    double coord = atof(coord_str);
    if (coord == 0.0) return 0.0;

    // Convert DDMM.MMMM to decimal degrees
    int degrees = (int)(coord / 100);
    double minutes = coord - (degrees * 100);
    double result = degrees + (minutes / 60.0);

    // Apply direction
    if (dir_str[0] == 'S' || dir_str[0] == 'W') result = -result;

    return result;
}

// Parse GPRMC sentence: $GPRMC,time,status,lat,latdir,lon,londir,speed,course,date,magvar,magvardir*checksum
static bool parse_gprmc(GPS_Handle_t *gps, const char *sentence) {
    const char *ptr = strchr(sentence, ',');
    if (!ptr) return false;
    ptr++; // Skip past first comma after "GPRMC"

    char field[32];
    GPS_Fix_t *fix = &gps->fix;

    // Time (HHMMSS.sss)
    if (parse_nmea_field(&ptr, field, sizeof(field)) && strlen(field) >= 6) {
        fix->hour = (field[0] - '0') * 10 + (field[1] - '0');
        fix->minute = (field[2] - '0') * 10 + (field[3] - '0');
        fix->second = (field[4] - '0') * 10 + (field[5] - '0');
    }

    // Status (A=valid, V=invalid)
    parse_nmea_field(&ptr, field, sizeof(field));
    bool valid = (field[0] == 'A');

    // Latitude
    char lat_str[16], lat_dir[4];
    parse_nmea_field(&ptr, lat_str, sizeof(lat_str));
    parse_nmea_field(&ptr, lat_dir, sizeof(lat_dir));
    if (valid) fix->latitude_deg = parse_coord(lat_str, lat_dir);

    // Longitude
    char lon_str[16], lon_dir[4];
    parse_nmea_field(&ptr, lon_str, sizeof(lon_str));
    parse_nmea_field(&ptr, lon_dir, sizeof(lon_dir));
    if (valid) fix->longitude_deg = parse_coord(lon_str, lon_dir);

    // Speed (knots)
    parse_nmea_field(&ptr, field, sizeof(field));
    if (valid) fix->speed_knots = (float)atof(field);

    // Course (degrees)
    parse_nmea_field(&ptr, field, sizeof(field));
    if (valid) fix->course_deg = (float)atof(field);

    // Date (DDMMYY)
    if (parse_nmea_field(&ptr, field, sizeof(field)) && strlen(field) >= 6) {
        fix->day = (field[0] - '0') * 10 + (field[1] - '0');
        fix->month = (field[2] - '0') * 10 + (field[3] - '0');
        fix->year = (field[4] - '0') * 10 + (field[5] - '0');
        if (fix->year < 80) fix->year += 100; // Y2K handling: 00-79 -> 2000-2079
    }

    fix->valid = valid;
    return true;
}

// Parse GPGGA sentence for additional fix info
static bool parse_gpgga(GPS_Handle_t *gps, const char *sentence) {
    const char *ptr = strchr(sentence, ',');
    if (!ptr) return false;
    ptr++; // Skip past first comma

    char field[32];
    GPS_Fix_t *fix = &gps->fix;

    // Skip time field (already parsed from RMC)
    parse_nmea_field(&ptr, field, sizeof(field));

    // Skip lat/lon (already from RMC)
    parse_nmea_field(&ptr, field, sizeof(field)); // lat
    parse_nmea_field(&ptr, field, sizeof(field)); // lat dir
    parse_nmea_field(&ptr, field, sizeof(field)); // lon
    parse_nmea_field(&ptr, field, sizeof(field)); // lon dir

    // Fix quality (0=invalid, 1=GPS, 2=DGPS)
    parse_nmea_field(&ptr, field, sizeof(field));
    fix->fix_quality = (uint8_t)atoi(field);

    // Number of satellites
    parse_nmea_field(&ptr, field, sizeof(field));
    fix->sats_in_use = (uint8_t)atoi(field);

    // HDOP
    parse_nmea_field(&ptr, field, sizeof(field));
    fix->hdop = (float)atof(field);

    // Altitude
    parse_nmea_field(&ptr, field, sizeof(field));
    fix->altitude_m = (float)atof(field);

    return true;
}

// ===== HAL Callbacks =====

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (!s_gps || huart != s_gps->huart || !s_gps->started || Size == 0) return;

    gps_irq_events++;
    GPS_OnRxChunk(s_gps, s_gps->dma_buf, Size);

    // Re-arm DMA reception
    HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(huart, s_gps->dma_buf, GPS_DMA_BUFFER_SIZE);
    if (st != HAL_OK) {
        GPS_DEBUG("RxEvent: re-arm failed st=%d err=0x%08lx\r\n", st, huart->ErrorCode);
    } else {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // Disable half-transfer interrupt
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (!s_gps || huart != s_gps->huart) return;

    GPS_DEBUG("UART error: 0x%08lx (state=%lu)\r\n",
              huart->ErrorCode, (unsigned long)HAL_UART_GetState(huart));

    // Try to recover by re-arming DMA
    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_gps->dma_buf, GPS_DMA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

// ===== Public API Implementation =====

void GPS_Init(GPS_Handle_t *gps, UART_HandleTypeDef *huart, GPS_RxMode_t mode, const GPS_OptionalPins_t *pins) {
    if (!gps) return;

    // Clear entire structure
    memset(gps, 0, sizeof(*gps));

    // Set hardware handles
    gps->huart = huart;
    gps->rx_mode = mode;

    // Copy optional pins if provided
    if (pins) {
        gps->pins = *pins;

        // Initialize optional pins to safe states
        if (gps->pins.RESET_Port) {
            HAL_GPIO_WritePin(gps->pins.RESET_Port, gps->pins.RESET_Pin, GPIO_PIN_SET); // Not reset
        }
        if (gps->pins.FORCE_Port) {
            HAL_GPIO_WritePin(gps->pins.FORCE_Port, gps->pins.FORCE_Pin, GPIO_PIN_RESET); // Not forced
        }
    }

    // Initialize buffers
    gps->rb_head = 0;
    gps->rb_tail = 0;
    gps->line_len = 0;

    // Initialize fix data
    gps->fix.valid = false;
    gps->fix.fix_quality = 0;
    gps->fix.sats_in_use = 0;
    gps->fix.hdop = 0.0f;
    gps->fix.latitude_deg = 0.0;
    gps->fix.longitude_deg = 0.0;
    gps->fix.altitude_m = 0.0f;
    gps->fix.speed_knots = 0.0f;
    gps->fix.course_deg = 0.0f;

    gps->started = false;
}

HAL_StatusTypeDef GPS_Start(GPS_Handle_t *gps) {
    if (!gps || !gps->huart) {
        GPS_DEBUG("Start: Invalid GPS handle or UART\r\n");
        return HAL_ERROR;
    }

    if (!gps->huart->hdmarx) {
        GPS_DEBUG("Start: No DMA configured - Check CubeMX USART1 RX DMA setup\r\n");
        return HAL_ERROR;
    }

    GPS_DEBUG("Start: UART=%p, DMA=%p\r\n", (void*)gps->huart, (void*)gps->huart->hdmarx);

    // Reset buffers
    gps->rb_head = 0;
    gps->rb_tail = 0;
    gps->line_len = 0;
    gps->started = true;

    // Set global pointer for callbacks
    s_gps = gps;

    // Clear any pending UART flags
    __HAL_UART_CLEAR_IDLEFLAG(gps->huart);
    gps->huart->ErrorCode = HAL_UART_ERROR_NONE;

    // Start DMA reception
    HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(gps->huart, gps->dma_buf, GPS_DMA_BUFFER_SIZE);
    if (st != HAL_OK) {
        GPS_DEBUG("Start: DMA start failed: %d, err=0x%08lx\r\n", st, gps->huart->ErrorCode);
        gps->started = false;
        return st;
    }

    // Disable half-transfer interrupt (we only need complete/idle)
    __HAL_DMA_DISABLE_IT(gps->huart->hdmarx, DMA_IT_HT);

    GPS_DEBUG("Start: OK - DMA armed for %d bytes\r\n", GPS_DMA_BUFFER_SIZE);
    return HAL_OK;
}

void GPS_OnRxChunk(GPS_Handle_t *gps, const uint8_t *data, uint16_t len) {
    if (!gps || !data) return;

    gps_rx_total += len;
    for (uint16_t i = 0; i < len; i++) {
        rb_put(gps, data[i]);
    }
}

void GPS_Process(GPS_Handle_t *gps) {
    if (!gps) return;

    uint8_t byte;

    // Process all available bytes
    while (rb_get(gps, &byte)) {
        // Skip carriage returns, use only line feeds
        if (byte == '\r') continue;

        // Guard against buffer overflow
        if (gps->line_len >= (GPS_LINE_BUFFER_SIZE - 1)) {
            gps->line_len = 0; // Reset on overflow
        }

        if (byte != '\n') {
            // Accumulate line
            gps->line[gps->line_len++] = byte;
            continue;
        }

        // End of line - process sentence
        if (gps->line_len == 0) continue; // Skip empty lines

        gps->line[gps->line_len] = '\0'; // Null terminate
        char *sentence = (char*)gps->line;

        // Validate sentence format
        if (sentence[0] != '$') {
            gps->line_len = 0;
            continue; // Not an NMEA sentence
        }

        // Find checksum
        char *asterisk = strrchr(sentence, '*');
        if (!asterisk || strlen(asterisk) < 3) {
            gps->line_len = 0;
            continue; // No valid checksum
        }

        // Parse checksum
        int h1 = hex_nib(asterisk[1]);
        int h2 = hex_nib(asterisk[2]);
        if (h1 < 0 || h2 < 0) {
            gps->line_len = 0;
            continue; // Invalid hex digits
        }

        uint8_t expected_checksum = (uint8_t)((h1 << 4) | h2);
        uint8_t actual_checksum = nmea_checksum(sentence);

        if (expected_checksum != actual_checksum) {
            GPS_DEBUG("Checksum error: exp=0x%02X, got=0x%02X\r\n", expected_checksum, actual_checksum);
            gps->line_len = 0;
            continue;
        }

        // Valid NMEA sentence - notify callback
        if (gps->on_sentence) {
            gps->on_sentence(sentence);
        }

        // Parse specific sentence types
        if (strncmp(sentence + 3, "RMC", 3) == 0) { // $GPRMC or $GNRMC
            if (parse_gprmc(gps, sentence) && gps->on_fix) {
                gps->on_fix(&gps->fix);
            }
        } else if (strncmp(sentence + 3, "GGA", 3) == 0) { // $GPGGA or $GNGGA
            parse_gpgga(gps, sentence);
        }

        // Ready for next sentence
        gps->line_len = 0;
    }
}

// ===== Optional Helper Functions =====

HAL_StatusTypeDef GPS_SendRaw(GPS_Handle_t *gps, const void *data, uint16_t len) {
    if (!gps || !gps->huart || !data) return HAL_ERROR;
    return HAL_UART_Transmit(gps->huart, (uint8_t*)data, len, 1000);
}

HAL_StatusTypeDef GPS_SendPMTK(GPS_Handle_t *gps, const char *pmtk) {
    if (!gps || !pmtk) return HAL_ERROR;

    // Calculate checksum for PMTK command
    uint8_t checksum = 0;
    for (const char *p = pmtk + 1; *p; p++) { // Skip initial '$'
        checksum ^= *p;
    }

    // Send command with checksum
    char cmd[128];
    int len = snprintf(cmd, sizeof(cmd), "%s*%02X\r\n", pmtk, checksum);
    if (len > 0 && len < sizeof(cmd)) {
        return GPS_SendRaw(gps, cmd, (uint16_t)len);
    }
    return HAL_ERROR;
}

void GPS_ResetPulse(GPS_Handle_t *gps, uint32_t low_ms) {
    if (!gps || !gps->pins.RESET_Port) return;

    HAL_GPIO_WritePin(gps->pins.RESET_Port, gps->pins.RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(low_ms);
    HAL_GPIO_WritePin(gps->pins.RESET_Port, gps->pins.RESET_Pin, GPIO_PIN_SET);
}

void GPS_ForceOn(GPS_Handle_t *gps, bool level) {
    if (!gps || !gps->pins.FORCE_Port) return;
    HAL_GPIO_WritePin(gps->pins.FORCE_Port, gps->pins.FORCE_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool GPS_ActiveAntennaPresent(GPS_Handle_t *gps) {
    if (!gps || !gps->pins.AADET_Port) return false;
    return HAL_GPIO_ReadPin(gps->pins.AADET_Port, gps->pins.AADET_Pin) == GPIO_PIN_RESET; // Active low
}
