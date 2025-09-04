#ifndef GPS_H
#define GPS_H

#include "stm32f4xx_hal.h"   // keep if you’re on F4; otherwise swap to your family
#include <stdint.h>
#include <stdbool.h>

// Defaults
#define GPS_DEFAULT_BAUD     9600U
#define GPS_UART_WORDLEN     UART_WORDLENGTH_8B
#define GPS_UART_STOPBITS    UART_STOPBITS_1
#define GPS_UART_PARITY      UART_PARITY_NONE

// RX buffers (tune as needed)
#ifndef GPS_RX_BUFFER_SIZE
#define GPS_RX_BUFFER_SIZE   512
#endif
#ifndef GPS_DMA_BUFFER_SIZE
#define GPS_DMA_BUFFER_SIZE  256
#endif
#ifndef GPS_LINE_BUFFER_SIZE
#define GPS_LINE_BUFFER_SIZE 128
#endif

// Optional pins
typedef struct {
    GPIO_TypeDef *RESET_Port; uint16_t RESET_Pin;  // active-low
    GPIO_TypeDef *FORCE_Port; uint16_t FORCE_Pin;  // high = wake from backup
    GPIO_TypeDef *AADET_Port; uint16_t AADET_Pin;  // low = active antenna present
    GPIO_TypeDef *PPS_Port;   uint16_t PPS_Pin;    // 1PPS
} GPS_OptionalPins_t;

// Parsed fix
typedef struct {
    bool     valid;
    uint8_t  fix_quality;
    uint8_t  sats_in_use;
    float    hdop;
    double   latitude_deg;
    double   longitude_deg;
    float    altitude_m;
    float    speed_knots;
    float    course_deg;
    uint8_t  hour, minute, second;
    uint8_t  day, month, year;
} GPS_Fix_t;

typedef enum { GPS_RXMODE_IT = 0, GPS_RXMODE_DMA } GPS_RxMode_t;

typedef struct {
    UART_HandleTypeDef *huart;
    GPS_RxMode_t        rx_mode;

    // DMA staging buffer (hardware fills this)
    uint8_t             dma_buf[GPS_DMA_BUFFER_SIZE];

    // Software ring buffer (ISR writes, Process reads)
    volatile uint16_t   rb_head, rb_tail;
    uint8_t             rb[GPS_RX_BUFFER_SIZE];

    // Line assembly
    uint8_t             line[GPS_LINE_BUFFER_SIZE];
    uint16_t            line_len;

    // Last fix
    GPS_Fix_t           fix;

    // Callbacks (optional)
    void (*on_sentence)(const char *nmea);
    void (*on_fix)(const GPS_Fix_t *fix);

    // Optional GPIOs
    GPS_OptionalPins_t  pins;

    // State
    bool                started;
} GPS_Handle_t;

/* ===== Public API ===== */
void GPS_Init(GPS_Handle_t *gps, UART_HandleTypeDef *huart, GPS_RxMode_t mode, const GPS_OptionalPins_t *pins);
HAL_StatusTypeDef GPS_Start(GPS_Handle_t *gps);
void GPS_Process(GPS_Handle_t *gps);

// RX chunk helper the ISR calls
void GPS_OnRxChunk(GPS_Handle_t *g, const uint8_t *data, uint16_t len);

// TX helpers
HAL_StatusTypeDef GPS_SendRaw(GPS_Handle_t *gps, const void *data, uint16_t len);
HAL_StatusTypeDef GPS_SendPMTK(GPS_Handle_t *gps, const char *pmtk);

// Optional pin helpers
void GPS_ResetPulse(GPS_Handle_t *gps, uint32_t low_ms);
void GPS_ForceOn(GPS_Handle_t *gps, bool level);
bool GPS_ActiveAntennaPresent(GPS_Handle_t *gps);

#endif // GPS_H
