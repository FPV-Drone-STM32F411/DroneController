#include "GPS.h"

// Global variables
GPS_State_t gps_state = {0};
volatile uint32_t gps_irq_events = 0;
volatile uint32_t gps_rx_total = 0;

// External UART handle (from main.c)
extern UART_HandleTypeDef huart1;

/**
 * @brief Initialize GPS module
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef GPS_Init(void)
{
    // Clear GPS state
    memset(&gps_state, 0, sizeof(gps_state));

    printf("[GPS] Initializing...\r\n");

    // Enable default NMEA sentences (RMC, VTG, GGA, GSA, GSV, GLL, and GPTXT)
    GPS_SendCommand("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
    HAL_Delay(100);

    // Set update rate to 1Hz
    GPS_SendCommand("PMTK220,1000");
    HAL_Delay(100);

    // Enable AIC (Active Interference Cancellation)
    GPS_SendCommand("PMTK286,1");
    HAL_Delay(100);

    // Enable EASY (default on)
    GPS_SendCommand("PMTK869,1,1");
    HAL_Delay(100);

    // Enable UART interrupt for receiving data
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(&GPS_UART_HANDLE, UART_IT_RXNE);

    printf("[GPS] Initialized successfully\r\n");
    return HAL_OK;
}

/**
 * @brief GPS UART interrupt handler - call this from USART1_IRQHandler
 */
void GPS_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&GPS_UART_HANDLE, UART_FLAG_RXNE))
    {
        gps_irq_events++;

        // Read the received byte
        uint8_t received_byte = (uint8_t)(GPS_UART_HANDLE.Instance->DR & 0xFF);
        gps_rx_total++;

        // Handle received byte
        if (received_byte == '$')
        {
            // Start of new NMEA sentence
            gps_state.rx_index = 0;
            gps_state.rx_buffer[gps_state.rx_index++] = received_byte;
        }
        else if (received_byte == '\n')
        {
            // End of NMEA sentence
            if (gps_state.rx_index > 0 && gps_state.rx_index < GPS_RX_BUFFER_SIZE - 1)
            {
                gps_state.rx_buffer[gps_state.rx_index] = '\0';

                // Copy to current sentence buffer
                if (gps_state.rx_index < GPS_MAX_SENTENCE_LEN - 1)
                {
                    strcpy(gps_state.current_sentence, (char*)gps_state.rx_buffer);
                    gps_state.sentence_ready = true;
                    gps_state.sentences_received++;
                }
            }
            gps_state.rx_index = 0;
        }
        else if (received_byte != '\r')
        {
            // Add to buffer (ignore carriage return)
            if (gps_state.rx_index < GPS_RX_BUFFER_SIZE - 1)
            {
                gps_state.rx_buffer[gps_state.rx_index++] = received_byte;
            }
        }

        // Clear the interrupt flag
        __HAL_UART_CLEAR_FLAG(&GPS_UART_HANDLE, UART_FLAG_RXNE);
    }

    // Handle overrun error
    if (__HAL_UART_GET_FLAG(&GPS_UART_HANDLE, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(&GPS_UART_HANDLE);
    }
}

/**
 * @brief Check if a complete NMEA sentence is ready
 * @retval true if sentence is ready
 */
bool GPS_IsSentenceReady(void)
{
    return gps_state.sentence_ready;
}

/**
 * @brief Get the current NMEA sentence and mark as processed
 * @retval Pointer to sentence string
 */
char* GPS_GetSentence(void)
{
    if (gps_state.sentence_ready)
    {
        gps_state.sentence_ready = false;
        return gps_state.current_sentence;
    }
    return NULL;
}

/**
 * @brief Parse NMEA sentence and update GPS fix
 * @param sentence NMEA sentence string
 * @retval true if sentence was parsed successfully
 */
bool GPS_ParseSentence(const char* sentence)
{
    if (!sentence || strlen(sentence) < 6)
        return false;

    // Validate checksum
    if (!GPS_ValidateChecksum(sentence))
    {
        gps_state.checksum_errors++;
        return false;
    }

    gps_state.valid_sentences++;

    // Parse different sentence types
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0)
    {
        return GPS_ParseGGA(sentence);
    }
    else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0)
    {
        return GPS_ParseRMC(sentence);
    }
    else if (strncmp(sentence, "$GPGSA", 6) == 0 || strncmp(sentence, "$GNGSA", 6) == 0)
    {
        return GPS_ParseGSA(sentence);
    }

    return false;
}

/**
 * @brief Parse GGA sentence (Global Positioning System Fix Data)
 */
bool GPS_ParseGGA(const char* sentence)
{
    char fields[GPS_MAX_FIELDS][20];
    int field_count = GPS_ParseFields(sentence, fields, GPS_MAX_FIELDS);

    if (field_count < 14)
        return false;

    // Parse time (field 1: HHMMSS.SS)
    if (strlen(fields[1]) >= 6)
    {
        char time_str[7];
        strncpy(time_str, fields[1], 6);
        time_str[6] = '\0';

        gps_state.fix.hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
        gps_state.fix.minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
        gps_state.fix.second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
    }

    // Parse coordinates
    if (strlen(fields[2]) > 0 && strlen(fields[4]) > 0)
    {
        gps_state.fix.latitude_deg = GPS_ConvertToDecimalDegrees(fields[2], fields[3]);
        gps_state.fix.longitude_deg = GPS_ConvertToDecimalDegrees(fields[4], fields[5]);
    }

    // Parse fix quality (field 6)
    if (strlen(fields[6]) > 0)
    {
        gps_state.fix.fix_quality = atoi(fields[6]);
        gps_state.fix.valid = (gps_state.fix.fix_quality > 0);
    }

    // Parse number of satellites (field 7)
    if (strlen(fields[7]) > 0)
    {
        gps_state.fix.sats_in_use = atoi(fields[7]);
    }

    // Parse HDOP (field 8)
    if (strlen(fields[8]) > 0)
    {
        gps_state.fix.hdop = atof(fields[8]);
    }

    // Parse altitude (field 9)
    if (strlen(fields[9]) > 0)
    {
        gps_state.fix.altitude_m = atof(fields[9]);
    }

    return true;
}

/**
 * @brief Parse RMC sentence (Recommended Minimum Specific GPS/Transit Data)
 */
bool GPS_ParseRMC(const char* sentence)
{
    char fields[GPS_MAX_FIELDS][20];
    int field_count = GPS_ParseFields(sentence, fields, GPS_MAX_FIELDS);

    if (field_count < 12)
        return false;

    // Parse status (field 2: A=Active, V=Void)
    bool rmc_valid = (fields[2][0] == 'A');

    // Parse date (field 9: DDMMYY)
    if (strlen(fields[9]) >= 6)
    {
        char date_str[7];
        strncpy(date_str, fields[9], 6);
        date_str[6] = '\0';

        gps_state.fix.day = (date_str[0] - '0') * 10 + (date_str[1] - '0');
        gps_state.fix.month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
        gps_state.fix.year = 2000 + (date_str[4] - '0') * 10 + (date_str[5] - '0');
    }

    // Parse speed (field 7: knots)
    if (strlen(fields[7]) > 0)
    {
        gps_state.fix.speed_knots = atof(fields[7]);
    }

    // Parse course (field 8: degrees)
    if (strlen(fields[8]) > 0)
    {
        gps_state.fix.course_deg = atof(fields[8]);
    }

    // Update validity based on RMC status
    if (rmc_valid && gps_state.fix.fix_quality > 0)
    {
        gps_state.fix.valid = true;
    }

    return true;
}

/**
 * @brief Parse GSA sentence (GPS DOP and active satellites)
 */
bool GPS_ParseGSA(const char* sentence)
{
    char fields[GPS_MAX_FIELDS][20];
    int field_count = GPS_ParseFields(sentence, fields, GPS_MAX_FIELDS);

    if (field_count < 18)
        return false;

    // Parse fix type (field 2: 1=No fix, 2=2D, 3=3D)
    if (strlen(fields[2]) > 0)
    {
        int fix_type = atoi(fields[2]);
        // Update validity based on fix type
        if (fix_type >= 2)
        {
            gps_state.fix.valid = true;
        }
    }

    return true;
}

/**
 * @brief Calculate NMEA checksum
 */
uint8_t GPS_CalculateChecksum(const char* sentence)
{
    uint8_t checksum = 0;

    // Start after '$' and calculate until '*'
    for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++)
    {
        checksum ^= sentence[i];
    }

    return checksum;
}

/**
 * @brief Validate NMEA sentence checksum
 */
bool GPS_ValidateChecksum(const char* sentence)
{
    // Find the '*' character
    char* star_pos = strchr(sentence, '*');
    if (!star_pos || strlen(star_pos) < 3)
        return false;

    // Calculate checksum
    uint8_t calculated = GPS_CalculateChecksum(sentence);

    // Parse received checksum (hex)
    uint8_t received = 0;
    char hex_str[3] = {star_pos[1], star_pos[2], '\0'};
    received = (uint8_t)strtol(hex_str, NULL, 16);

    return (calculated == received);
}

/**
 * @brief Parse NMEA sentence fields
 */
int GPS_ParseFields(const char* sentence, char fields[][20], int max_fields)
{
    int field_count = 0;
    int field_index = 0;
    int start_pos = 0;

    // Find first comma to start parsing fields
    for (int i = 0; sentence[i] != '\0' && sentence[i] != '*'; i++)
    {
        if (sentence[i] == ',')
        {
            if (start_pos == 0)  // First comma found, start field parsing
            {
                start_pos = i + 1;
                field_index = 0;
                continue;
            }

            // End of current field
            fields[field_count][field_index] = '\0';
            field_count++;
            field_index = 0;

            if (field_count >= max_fields)
                break;
        }
        else if (start_pos > 0)  // Only process characters after first comma
        {
            if (field_index < 19) // Prevent buffer overflow
            {
                fields[field_count][field_index++] = sentence[i];
            }
        }
    }

    // Close last field
    if (field_count < max_fields && start_pos > 0)
    {
        fields[field_count][field_index] = '\0';
        field_count++;
    }

    return field_count;
}

/**
 * @brief Convert GPS coordinates to decimal degrees
 */
double GPS_ConvertToDecimalDegrees(const char* coord, const char* direction)
{
    if (!coord || !direction || strlen(coord) < 4)
        return 0.0;

    double coordinate = atof(coord);

    // For latitude: DDMM.MMMM, for longitude: DDDMM.MMMM
    int degrees;
    double minutes;

    if (strlen(coord) > 8) // Longitude format DDDMM.MMMM
    {
        degrees = (int)(coordinate / 100);
        minutes = coordinate - (degrees * 100);
    }
    else // Latitude format DDMM.MMMM
    {
        degrees = (int)(coordinate / 100);
        minutes = coordinate - (degrees * 100);
    }

    double decimal_degrees = degrees + (minutes / 60.0);

    // Apply direction
    if (direction[0] == 'S' || direction[0] == 'W')
    {
        decimal_degrees = -decimal_degrees;
    }

    return decimal_degrees;
}

/**
 * @brief Get current GPS fix
 * @retval Pointer to GPS fix structure
 */
GPS_Fix_t* GPS_GetFix(void)
{
    return &gps_state.fix;
}

/**
 * @brief Print GPS debug information
 */
void GPS_PrintDebugInfo(void)
{
    printf("[GPS DEBUG] IRQ Events: %lu, RX Total: %lu\r\n",
           gps_irq_events, gps_rx_total);
    printf("[GPS DEBUG] Sentences: %lu, Valid: %lu, Checksum Errors: %lu\r\n",
           gps_state.sentences_received, gps_state.valid_sentences, gps_state.checksum_errors);
}

/**
 * @brief Send command to GPS module
 * @param command Command string (without $ and checksum)
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef GPS_SendCommand(const char* command)
{
    char full_command[100];
    uint8_t checksum = 0;

    // Calculate checksum
    for (int i = 0; command[i] != '\0'; i++)
    {
        checksum ^= command[i];
    }

    // Format complete command
    sprintf(full_command, "$%s*%02X\r\n", command, checksum);

    // Send command
    printf("[GPS CMD] %s", full_command);  // Debug output
    return HAL_UART_Transmit(&GPS_UART_HANDLE, (uint8_t*)full_command,
                            strlen(full_command), 1000);
}

/**
 * @brief Auto-detect GPS baud rate
 */
void GPS_AutobaudScan(void)
{
    uint32_t baud_rates[] = {9600, 38400, 57600, 115200};
    int num_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);

    printf("[GPS] Starting autobaud scan...\r\n");

    for (int i = 0; i < num_rates; i++)
    {
        printf("[GPS] Trying %lu baud...\r\n", baud_rates[i]);

        // Change baud rate
        GPS_UART_HANDLE.Init.BaudRate = baud_rates[i];
        if (HAL_UART_Init(&GPS_UART_HANDLE) != HAL_OK)
        {
            continue;
        }

        // Re-enable interrupt
        __HAL_UART_ENABLE_IT(&GPS_UART_HANDLE, UART_IT_RXNE);

        // Wait for data
        uint32_t start_count = gps_rx_total;
        HAL_Delay(2000);

        if (gps_rx_total > start_count + 10)
        {
            printf("[GPS] Found GPS at %lu baud\r\n", baud_rates[i]);
            GPS_Init();
            return;
        }
    }

    printf("[GPS] Autobaud failed, using 9600\r\n");
    GPS_UART_HANDLE.Init.BaudRate = 9600;
    HAL_UART_Init(&GPS_UART_HANDLE);
    GPS_Init();
}
