#ifndef GPS_H
#define GPS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// GPS Configuration
#define GPS_UART_HANDLE        huart1
#define GPS_RX_BUFFER_SIZE     512
#define GPS_MAX_SENTENCE_LEN   100
#define GPS_MAX_FIELDS         20

// GPS Fix Structure
typedef struct {
    bool valid;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float speed_knots;
    float course_deg;
    float hdop;
    uint8_t sats_in_use;
    uint8_t fix_quality;
} GPS_Fix_t;

// GPS State Structure
typedef struct {
    uint8_t rx_buffer[GPS_RX_BUFFER_SIZE];
    uint16_t rx_index;
    bool sentence_ready;
    char current_sentence[GPS_MAX_SENTENCE_LEN];
    GPS_Fix_t fix;
    uint32_t sentences_received;
    uint32_t valid_sentences;
    uint32_t checksum_errors;
} GPS_State_t;

// Function Declarations
HAL_StatusTypeDef GPS_Init(void);
void GPS_IRQHandler(void);
bool GPS_IsSentenceReady(void);
char* GPS_GetSentence(void);
bool GPS_ParseSentence(const char* sentence);
GPS_Fix_t* GPS_GetFix(void);
void GPS_PrintDebugInfo(void);
HAL_StatusTypeDef GPS_SendCommand(const char* command);
void GPS_AutobaudScan(void);

// NMEA Parser Functions
bool GPS_ParseGGA(const char* sentence);
bool GPS_ParseRMC(const char* sentence);
bool GPS_ParseGSA(const char* sentence);
uint8_t GPS_CalculateChecksum(const char* sentence);
bool GPS_ValidateChecksum(const char* sentence);
int GPS_ParseFields(const char* sentence, char fields[][20], int max_fields);
double GPS_ConvertToDecimalDegrees(const char* coord, const char* direction);

// External variables (defined in GPS.c)
extern GPS_State_t gps_state;
extern volatile uint32_t gps_irq_events;
extern volatile uint32_t gps_rx_total;

#endif // GPS_H
