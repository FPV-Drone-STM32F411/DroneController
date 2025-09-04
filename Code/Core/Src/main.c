/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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

extern volatile uint32_t gps_irq_events;
extern volatile uint32_t gps_rx_total;

static void gps_print_sentence(const char *nmea) {
    printf("%s\r\n", nmea);   // prints full, checksum-valid NMEA lines
}

static void gps_print_fix(const GPS_Fix_t *fix) {
    if (fix->valid) {
        printf("[GPS FIX] %02d:%02d:%02d %02d/%02d/%02d\r\n",
               fix->hour, fix->minute, fix->second,
               fix->day, fix->month, fix->year);
        printf("          Lat=%.6f Lon=%.6f Alt=%.1fm\r\n",
               fix->latitude_deg, fix->longitude_deg, fix->altitude_m);
        printf("          Speed=%.1fkn Course=%.1f° HDOP=%.1f Sats=%d\r\n",
               fix->speed_knots, fix->course_deg, fix->hdop, fix->sats_in_use);
    } else {
        printf("[GPS] No valid fix\r\n");
    }
}




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
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */

// ===== choose the BMP280 CS you actually wired =====
#define BMP280_CS_GPIO   GPIOA         // e.g., GPIOA
#define BMP280_CS_PIN    NSS3_Pin      // e.g., your NSS3 pin
#define SEA_LEVEL_PA     101325.0f

static BMP280_HandleTypeDef bmp;

static void BMP280_AppInit(void);      // forward decl

// (optional) keep these here so you don't redeclare inside the loop
static uint32_t last_irq = 0, last_bytes = 0;

/* USER CODE END PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//printf redirection
//int _write(int file, char *ptr, int len) {
//    HAL_UART_Transmit(&husart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
///    return len;
//}
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Make sure BMP280 CS idles HIGH before first SPI transaction
  HAL_GPIO_WritePin(BMP280_CS_GPIO, BMP280_CS_PIN, GPIO_PIN_SET);

  // Initialize/configure BMP280
  BMP280_AppInit();

#define BMP280_CS_GPIO   GPIOA      // I’m assuming your BMP280 CS is on GPIOA
#define BMP280_CS_PIN    NSS3_Pin   // and uses your NSS3 pin
  __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);  // <-- MUST exist
  // after MX_DMA_Init() and MX_USART1_UART_Init()
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

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
 	        HAL_GPIO_TogglePin(GPIOA, LED_Pin);
 	        HAL_Delay(500);
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


 	//gps init
 	static GPS_Handle_t gps;
 	  GPS_Init(&gps, &huart1, GPS_RXMODE_DMA, NULL);

 	  // Set GPS callbacks
 	  gps.on_sentence = gps_print_sentence;
 	  gps.on_fix = gps_print_fix;

 	  // Start GPS
 	  HAL_StatusTypeDef st = GPS_Start(&gps);
 	  printf("GPS Start: status=%d, huart1.hdmarx=%p, err=0x%08lx\r\n",
 	         st, (void*)huart1.hdmarx, huart1.ErrorCode);

 	  if (st != HAL_OK) {
 	      printf("GPS_Start failed: %d\r\n", st);
 	      printf("Check: 1) USART1 RX DMA enabled in CubeMX\r\n");
 	      printf("       2) DMA2_Stream2 IRQ enabled\r\n");
 	      printf("       3) Correct GPIO pins assigned\r\n");
 	  } else {
 	      printf("GPS started successfully - waiting for data...\r\n");
 	  }




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 	int16_t accel[3];
 	int16_t gyro[3];

 	float ax_ms2, ay_ms2, az_ms2;
 	float gx_dps, gy_dps, gz_dps;
 	float pitch_deg, roll_deg;

 	uint32_t last_gps_debug = 0;
 	uint32_t last_imu_print = 0;
 	uint32_t last_irq = 0, last_bytes = 0;

 	uint32_t next_bmp      = HAL_GetTick();   // 5 Hz BMP280
 	uint32_t next_imu_print= HAL_GetTick();   // 1 Hz IMU
 	uint32_t next_gps_dbg  = HAL_GetTick();   // 1 Hz GPS debug

 	printf("Enter while(1)\r\n");
 	BMP280_AppInit();
 	while (1)
 	{

 		GPS_Process(&gps);
 		static uint32_t last_irq=0, last_bytes=0;
 		if (HAL_GetTick() - last_gps_debug > 1000) {
 		      last_gps_debug = HAL_GetTick();
 		      printf("[GPS Debug] IRQ events=%lu(+%lu) RX bytes=%lu(+%lu)\r\n",
 		             gps_irq_events, gps_irq_events - last_irq,
 		             gps_rx_total, gps_rx_total - last_bytes);
 		      last_irq = gps_irq_events;
 		      last_bytes = gps_rx_total;

 		      // Show current fix status
 		      if (gps.fix.valid) {
 		          printf("[GPS Status] Valid fix with %d satellites\r\n", gps.fix.sats_in_use);
 		      } else {
 		          printf("[GPS Status] Searching for satellites...\r\n");
 		      }
 		    }
 	    // --- Read and convert accelerometer ---
 	    if (icm42605_read_accel(accel) == HAL_OK)
 	    {
 	        // Convert to m/s²
 	        ax_ms2 = (accel[0] / ACCEL_LSB_PER_G) * G_IN_MS2;
 	        ay_ms2 = (accel[1] / ACCEL_LSB_PER_G) * G_IN_MS2;
 	        az_ms2 = (accel[2] / ACCEL_LSB_PER_G) * G_IN_MS2;
 	    }

 	    // --- Read and convert gyroscope ---
 	    if (icm42605_read_gyro(gyro) == HAL_OK)
 	    {
 	        // Convert to °/s
 	        gx_dps = gyro[0] / GYRO_LSB_PER_DPS;
 	        gy_dps = gyro[1] / GYRO_LSB_PER_DPS;
 	        gz_dps = gyro[2] / GYRO_LSB_PER_DPS;
 	    }

 	    // --- Calculate pitch and roll angles ---
 	    float ax_g = accel[0] / ACCEL_LSB_PER_G;
 	    float ay_g = accel[1] / ACCEL_LSB_PER_G;
 	    float az_g = accel[2] / ACCEL_LSB_PER_G;

 	    pitch_deg = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * (180.0f / M_PI);
 	    roll_deg  = atan2f( ay_g, az_g ) * (180.0f / M_PI);

 	    // --- Print formatted, readable values ---
 	    printf("[IMU: Accel] (m/s²): X=%.2f Y=%.2f Z=%.2f\r\n", ax_ms2, ay_ms2, az_ms2);
 	    printf("[IMU: Gyro] (°/s): X=%.2f Y=%.2f Z=%.2f\r\n", gx_dps, gy_dps, gz_dps);
 	    printf("[IMU: Angles] (deg): Pitch=%.2f Roll=%.2f\r\n", pitch_deg, roll_deg);

 	    HAL_Delay(1000); // ~20 Hz refresh for readability

 	    // --- BMP280 read (5 Hz) ---
 	      if ((int32_t)(HAL_GetTick() - next_bmp) >= 0) {
 	          next_bmp += 200;

 	          float t_c, p_pa;
 	          if (bmp280_read_temp_press(&bmp, &t_c, &p_pa) == HAL_OK) {
 	              float alt_m = bmp280_altitude_m_from_pa(p_pa, SEA_LEVEL_PA);
 	              printf("[BMP280] T=%.2f C  P=%.1f Pa  Alt=%.1f m\r\n", t_c, p_pa, alt_m);
 	          }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 	      }
 	}
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  	HAL_GPIO_WritePin(GPIOA, NSS1_Pin|NSS3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NSS2_GPIO_Port, NSS2_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|NSS1_Pin|NSS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS2_GPIO_Port, NSS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin NSS1_Pin NSS3_Pin */
  GPIO_InitStruct.Pin = LED_Pin|NSS1_Pin|NSS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS2_Pin */
  GPIO_InitStruct.Pin = NSS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
