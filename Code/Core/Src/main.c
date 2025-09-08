/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ICM-42605.h"
#include "BMP280.h"
#include "GPS.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define SEA_LEVEL_PA     101325.0f

// Simple GPS test variables
volatile uint32_t uart_rx_count = 0;
volatile uint8_t last_byte = 0;

// --- IMU calibration settings ---
#define IMU_CAL_SAMPLES     512    // number of samples to average
#define IMU_CAL_SETTLE_MS   200    // small settle before sampling
#define EMA_ALPHA           0.12f  // optional smoothing (0..1). Lower = smoother

// Biases stored in RAW COUNTS (not m/s² or dps)
static float acc_bias_counts[3] = {0};
static float gyr_bias_counts[3] = {0};

// Optional EMA filtered values (in 'g' and dps)
static float ax_g_f=0, ay_g_f=0, az_g_f=0;
static float gx_dps_f=0, gy_dps_f=0, gz_dps_f=0;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BMP280_CS_GPIO   GPIOA
#define BMP280_CS_PIN    NSS3_Pin

static BMP280_HandleTypeDef bmp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void BMP280_AppInit(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void IMU_Calibrate(void)
{
    // Hold the board still, Z+ pointing up
    HAL_Delay(IMU_CAL_SETTLE_MS);

    int64_t sax = 0, say = 0, saz = 0;
    int64_t sgx = 0, sgy = 0, sgz = 0;
    int16_t a[3], g[3];

    for (int i = 0; i < IMU_CAL_SAMPLES; i++) {
        if (icm42605_read_accel(a) == HAL_OK) {
            sax += a[0]; say += a[1]; saz += a[2];
        }
        if (icm42605_read_gyro(g) == HAL_OK) {
            sgx += g[0]; sgy += g[1]; sgz += g[2];
        }
        HAL_Delay(2); // ~500 Hz sampling during calibration
    }

    const float invN = 1.0f / (float)IMU_CAL_SAMPLES;
    const float mean_ax = (float)sax * invN;
    const float mean_ay = (float)say * invN;
    const float mean_az = (float)saz * invN;
    const float mean_gx = (float)sgx * invN;
    const float mean_gy = (float)sgy * invN;
    const float mean_gz = (float)sgz * invN;

    // Accel biases in COUNTS: zero X/Y; make Z read exactly +1g at rest
    acc_bias_counts[0] = mean_ax;
    acc_bias_counts[1] = mean_ay;
    acc_bias_counts[2] = mean_az - ACCEL_LSB_PER_G;  // assumes Z+ up

    // Gyro biases in COUNTS (should be near 0 at rest)
    gyr_bias_counts[0] = mean_gx;
    gyr_bias_counts[1] = mean_gy;
    gyr_bias_counts[2] = mean_gz;

    // Prime EMAs with first values post-cal
    ax_g_f = 0.0f; ay_g_f = 0.0f; az_g_f = 1.0f;
    gx_dps_f = gy_dps_f = gz_dps_f = 0.0f;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("Starting GPS and sensor test\r\n");
  // Initialize GPS module
    if (GPS_Init() == HAL_OK) {
        printf("GPS module initialized successfully\r\n");
    } else {
        printf("GPS module initialization failed\r\n");
    }
  // Make sure BMP280 CS idles HIGH before first SPI transaction
  HAL_GPIO_WritePin(BMP280_CS_GPIO, BMP280_CS_PIN, GPIO_PIN_SET);

  // Initialize/configure BMP280
  BMP280_AppInit();

  printf("USB CDC up!\n");

  uint8_t id[3];
  if (W25_ReadJEDEC(id) == HAL_OK){
     printf("JEDEC: %02X %02X %02X\r\n", id[0], id[1], id[2]);
     HAL_GPIO_TogglePin (GPIOA, LED_Pin);
     HAL_Delay(1000);
     HAL_GPIO_TogglePin (GPIOA, LED_Pin);
    } else {
     printf("JEDEC read failed\r\n");
    }


  uint8_t whoami;
  HAL_StatusTypeDef whoami_test;

  whoami_test = icm42605_read_reg(0,0x75, &whoami);
  if (whoami_test == HAL_OK) {
      if (whoami == 0x42) {
          printf("Confirmed ICM WHOAMI: 0x%02X\r\n", whoami);
          HAL_Delay(500);
          HAL_GPIO_TogglePin(GPIOA, LED_Pin);
          HAL_Delay(1000);
          HAL_GPIO_TogglePin(GPIOA, LED_Pin);
      } else {
          printf("ICM WHOAMI mismatch: got 0x%02X (expected 0x42)\r\n", whoami);
      }
  } else {
      printf("Failed to read WHOAMI register (status: %d)\r\n", whoami_test);
  }

  // Reset chip
  icm42605_reset();

  // Configure accelerometer and gyro
  icm42605_config_accel(ACCEL_FS_2G, ACCEL_ODR_1KHZ);
  icm42605_config_gyro(GYRO_FS_2000DPS, GYRO_ODR_1KHZ);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Calibrating IMU... keep board still for ~1s\r\n");
  IMU_Calibrate();
  printf("Accel bias (cts): [%.1f, %.1f, %.1f], Gyro bias (cts): [%.1f, %.1f, %.1f]\r\n",
         acc_bias_counts[0], acc_bias_counts[1], acc_bias_counts[2],
         gyr_bias_counts[0], gyr_bias_counts[1], gyr_bias_counts[2]);
  int16_t accel[3];
  int16_t gyro[3];

  float ax_ms2, ay_ms2, az_ms2;
  float gx_dps, gy_dps, gz_dps;
  float pitch_deg, roll_deg;

  uint32_t next_bmp = HAL_GetTick();   // 5 Hz BMP280
  uint32_t next_gps = HAL_GetTick();   // 1 Hz GPS

  printf("Enter while(1)\r\n");
  extern void sx1276_min_demo(void);
  sx1276_min_demo();

  while (1)
   {

	  // Add this code to your main while loop for GPS simulation
	  // GPS Simulation (1 Hz) - replace your existing GPS processing section
	  if ((int32_t)(HAL_GetTick() - next_gps) >= 0) {
	      next_gps += 1000;

	      // Simulate GPS fix data with realistic changing values
	      static float sim_lat = 43.6532;      // Starting latitude (Toronto, Ontario)
	      static float sim_lon = -79.3832;     // Starting longitude (Toronto, Ontario)
	      static float sim_alt = 10.0;         // Starting altitude in meters
	      static float sim_speed = 0.0;        // Starting speed in knots
	      static float sim_course = 0.0;       // Starting course in degrees
	      static uint8_t sim_sats = 8;         // Number of satellites
	      static uint32_t sim_time_s = 0;      // Elapsed seconds for time simulation
	      static bool gps_fixed = false;
	      static uint32_t startup_counter = 0;

	      startup_counter++;

	      // Simulate GPS startup sequence (takes ~10 seconds to get fix)
	      if (startup_counter < 10) {
	          printf("[GPS] Searching for satellites... (%d/4 satellites)\r\n", startup_counter / 3);
	      } else if (!gps_fixed) {
	          gps_fixed = true;
	          printf("[GPS] GPS Fix acquired! Starting navigation...\r\n");
	      } else {
	          // Simulate realistic GPS movement and variations
	          sim_time_s++;

	          // Simulate slow movement (walking pace)
	          static float movement_angle = 0.0;
	          movement_angle += 0.05f;  // Slow rotation

	          // Add small realistic variations to position (GPS drift simulation)
	          sim_lat += (sinf(movement_angle * 0.1f) * 0.000005f);  // ~0.5m variation
	          sim_lon += (cosf(movement_angle * 0.1f) * 0.000005f);  // ~0.5m variation

	          // Simulate altitude changes (small variations)
	          sim_alt += sinf(movement_angle * 0.02f) * 0.5f + (rand() % 20 - 10) * 0.1f;
	          if (sim_alt < 0) sim_alt = 0;

	          // Simulate speed (walking/stationary with variations)
	          sim_speed = 0.5f + sinf(movement_angle * 0.3f) * 0.3f + (rand() % 10) * 0.05f;
	          if (sim_speed < 0) sim_speed = 0;

	          // Simulate course changes
	          sim_course = fmodf(sim_course + (rand() % 10 - 5) * 2.0f, 360.0f);
	          if (sim_course < 0) sim_course += 360.0f;

	          // Simulate satellite count variations (7-12 satellites)
	          if (sim_time_s % 10 == 0) {
	              sim_sats = 7 + (rand() % 6);
	          }

	          // Calculate time (simulate UTC time)
	          uint8_t hours = (sim_time_s / 3600) % 24;
	          uint8_t minutes = (sim_time_s / 60) % 60;
	          uint8_t seconds = sim_time_s % 60;

	          // Simulate date (fixed date for demo)
	          uint8_t day = 7;
	          uint8_t month = 9;
	          uint16_t year = 2025;

	          // Display GPS information
	          printf("[GPS] Lat: %.6f° Lon: %.6f°\r\n", sim_lat, sim_lon);
	          printf("[GPS] Satellites: %d Quality: DGPS\r\n", sim_sats);

	          // Simulate some raw NMEA-like output every few seconds
	          if (sim_time_s % 5 == 0) {
	              printf("[GPS RAW] $GPRMC,%02d%02d%02d.00,A,%.4f,N,%.4f,W,%.1f,%.1f,%02d%02d%02d,,,A*XX\r\n",
	                     hours, minutes, seconds,
	                     sim_lat, -sim_lon, sim_speed, sim_course,
	                     day, month, year % 100);
	          }

	          // Simulate HDOP and signal quality variations
	          if (sim_time_s % 3 == 0) {
	              float hdop = 1.0f + (rand() % 50) * 0.02f; // HDOP between 1.0-2.0
	              printf("[GPS] HDOP: %.2f Signal Quality: %s\r\n",
	                     hdop, (hdop < 1.5f) ? "Excellent" : (hdop < 2.0f) ? "Good" : "Fair");
	          }

	          // Simulate antenna status occasionally
	          if (sim_time_s % 15 == 0) {
	              printf("[GPS] Antenna Status: OK (Internal patch antenna)\r\n");
	          }
	      }

	      // Debug counter for simulation
	      static uint32_t debug_counter = 0;
	      debug_counter++;
	      if (debug_counter >= 30) {  // Every 30 seconds
	          debug_counter = 0;
	          printf("[GPS DEBUG] Simulation running for %lu seconds\r\n", sim_time_s);
	          printf("[GPS DEBUG] Position accuracy: ±3m, Time accuracy: ±50ms\r\n");
	      }
	  }

    // --- Read and convert accelerometer ---
     // --- Read raw accel/gyro ---
     if (icm42605_read_accel(accel) == HAL_OK) {
         // Bias-correct in COUNTS
         const float ax_c = (float)accel[0] - acc_bias_counts[0];
         const float ay_c = (float)accel[1] - acc_bias_counts[1];
         const float az_c = (float)accel[2] - acc_bias_counts[2];

         // Convert to g and m/s²
         float ax_g = ax_c / ACCEL_LSB_PER_G;
         float ay_g = ay_c / ACCEL_LSB_PER_G;
         float az_g = az_c / ACCEL_LSB_PER_G;

         // Optional smoothing for prettier printing
         ax_g_f = EMA_ALPHA*ax_g + (1.0f-EMA_ALPHA)*ax_g_f;
         ay_g_f = EMA_ALPHA*ay_g + (1.0f-EMA_ALPHA)*ay_g_f;
         az_g_f = EMA_ALPHA*az_g + (1.0f-EMA_ALPHA)*az_g_f;

         ax_ms2 = ax_g_f * G_IN_MS2;
         ay_ms2 = ay_g_f * G_IN_MS2;
         az_ms2 = az_g_f * G_IN_MS2;
     }

     if (icm42605_read_gyro(gyro) == HAL_OK) {
         // Bias-correct in COUNTS, then convert to dps
         const float gx_c = (float)gyro[0] - gyr_bias_counts[0];
         const float gy_c = (float)gyro[1] - gyr_bias_counts[1];
         const float gz_c = (float)gyro[2] - gyr_bias_counts[2];

         const float gx_dps_raw = gx_c / GYRO_LSB_PER_DPS;
         const float gy_dps_raw = gy_c / GYRO_LSB_PER_DPS;
         const float gz_dps_raw = gz_c / GYRO_LSB_PER_DPS;

         // Optional smoothing
         gx_dps_f = EMA_ALPHA*gx_dps_raw + (1.0f-EMA_ALPHA)*gx_dps_f;
         gy_dps_f = EMA_ALPHA*gy_dps_raw + (1.0f-EMA_ALPHA)*gy_dps_f;
         gz_dps_f = EMA_ALPHA*gz_dps_raw + (1.0f-EMA_ALPHA)*gz_dps_f;

         gx_dps = gx_dps_f; gy_dps = gy_dps_f; gz_dps = gz_dps_f;
     }

     // --- Angles from corrected accel (in g) ---
     float ax_g_use = ax_ms2 / G_IN_MS2;
     float ay_g_use = ay_ms2 / G_IN_MS2;
     float az_g_use = az_ms2 / G_IN_MS2;

     pitch_deg = atan2f(-ax_g_use, sqrtf(ay_g_use*ay_g_use + az_g_use*az_g_use)) * (180.0f / M_PI);
     roll_deg  = atan2f( ay_g_use, az_g_use ) * (180.0f / M_PI);

     // Print
     printf("[IMU: Accel] (m/s²): X=%.2f Y=%.2f Z=%.2f\r\n", ax_ms2, ay_ms2, az_ms2);
     printf("[IMU: Gyro]  (°/s):  X=%.2f Y=%.2f Z=%.2f\r\n", gx_dps, gy_dps, gz_dps);
     printf("[IMU: Angles] (deg):  Pitch=%.2f Roll=%.2f\r\n",  pitch_deg, roll_deg);

    HAL_GPIO_TogglePin(GPIOA, LED_Pin);
    HAL_Delay(100); // ~1 Hz refresh for readability

    // --- BMP280 read (5 Hz) ---
    if ((int32_t)(HAL_GetTick() - next_bmp) >= 0) {
        next_bmp += 200;

        float t_c, p_pa;
        if (bmp280_read_temp_press(&bmp, &t_c, &p_pa) == HAL_OK) {
            float alt_m = bmp280_altitude_m_from_pa(p_pa, SEA_LEVEL_PA);
            printf("[BMP280] T=%.2f C  P=%.1f Pa  Alt=%.1f m\r\n", t_c, p_pa, alt_m);
        }
    }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS5_Pin|NSS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin NSS1_Pin NSS3_Pin */
  GPIO_InitStruct.Pin = LED_Pin|NSS1_Pin|NSS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS5_Pin NSS2_Pin */
  GPIO_InitStruct.Pin = NSS5_Pin|NSS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOA, NSS1_Pin|NSS3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, NSS5_Pin|NSS2_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void BMP280_AppInit(void)
{
    if (bmp280_init_spi(&bmp, &hspi3, BMP280_CS_GPIO, BMP280_CS_PIN) != HAL_OK) {
        Error_Handler();
    }
    // Write CONFIG while device is in sleep (after reset)
    bmp280_set_config(&bmp, BMP280_TSB_125_MS, BMP280_FILTER_4, false);
    // Start continuous conversions
    bmp280_set_ctrl_meas(&bmp, BMP280_OSRS_X1, BMP280_OSRS_X4, BMP280_MODE_NORMAL);

    // Sanity read
    float t, p;
    if (bmp280_read_temp_press(&bmp, &t, &p) == HAL_OK) {
        float alt = bmp280_altitude_m_from_pa(p, SEA_LEVEL_PA);
        (void)alt;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
