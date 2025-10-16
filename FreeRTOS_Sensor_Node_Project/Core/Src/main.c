
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

#include <rtc.h>
#include "main.h"
#include "cmsis_os.h"


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tmp102.h"
#include"packet.h"
#include "mpu9250.h"
#include "semphr.h"
#include "rtc.h"
#include "filter.h"

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;


QueueHandle_t sensorQueue;

// Task function prototypes
void SensorTask(void *pvParameters);
void UartTxTask(void *pvParameters);
static void uart_safe_write(const uint8_t *data, uint16_t len, TickType_t timeoutTicks);
static void uart_safe_print(const char *s);


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void *argument);

SemaphoreHandle_t uartMutex;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_RTC_Init();

  MPU9250_Init(&hi2c1);

  uartMutex = xSemaphoreCreateMutex();
  configASSERT(uartMutex != NULL);

	//Create Queue
	sensorQueue = xQueueCreate(8, sizeof(SensorPacket));
	if (sensorQueue == NULL) {
		char err[] = "Queue not created!\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) err, strlen(err), HAL_MAX_DELAY);
		while (1)
			; 
	}

	//CreateTask
	xTaskCreate(SensorTask, "Sensor", 512, NULL, tskIDLE_PRIORITY+2, NULL);
	xTaskCreate(UartTxTask, "UART",   512, NULL, tskIDLE_PRIORITY+3, NULL);

	//Start Scheduler
	vTaskStartScheduler();

	while (1) {

		// test_json_output();
		//HAL_Delay(2000);
	}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x10;
  sTime.Minutes = 0x15;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_OCTOBER;
  sDate.Date = 0x14;
  sDate.Year = 0x25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
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
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BT1_Pin */
  GPIO_InitStruct.Pin = BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT1_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* ------------------------------------------------------------------ */
/* SensorTask: generates fake sensor values and puts them in the queue*/
/* ------------------------------------------------------------------ */
void SensorTask(void *pvParameters) {
    SensorPacket packet;

    // --- EMA için state (son değer saklama) ---
    static float acc_x_last = 0.0f;

    for (;;) {
        memset(&packet, 0, sizeof(packet));  // tüm alanları temizle

        // --- Temperature (TMP102) ---
        float temp_raw = 0.0f;
        if (TMP102_ReadTemperature(&hi2c1, &temp_raw) == HAL_OK) {
            float temp_filtered = moving_average(temp_raw);  // filtre uygula
            packet.temperature = temp_filtered;
            packet.valid |= V_TEMP;
        } else {
            uart_safe_print("Temperature (TMP102) read failed!\r\n");
        }

        // --- Timestamp ---
        packet.timestamp_ms = HAL_GetTick();
        RTC_GetTimestamp(packet.timestamp_str, sizeof(packet.timestamp_str));
        packet.valid |= V_TIME;

        // --- Accelerometer X (MPU9250) ---
        float acc_x_raw = 0.0f;
        if (MPU9250_ReadAccelX(&hi2c1, &acc_x_raw) == HAL_OK) {
            float acc_x_filtered = ema_filter(acc_x_raw, &acc_x_last, 0.1f);
            packet.acceleration_x = acc_x_filtered;
            packet.valid |= V_ACCEL_X;
        } else {
            uart_safe_print("MPU9250 accel X read failed!\r\n");
        }

        // --- Kuyruğa gönder ---
        if (xQueueSend(sensorQueue, &packet, portMAX_DELAY) != pdPASS) {
            uart_safe_print("SensorTask: Queue send failed!\r\n");
        }

        // --- Delay ---
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2 saniyede bir ölçüm
    }
}


/* ------------------------------------------------------------------ */
/* UartTxTask: reads values from the queue and sends over UART    */
/* ------------------------------------------------------------------ */

static void uart_safe_write(const uint8_t *data, uint16_t len, TickType_t timeoutTicks)
{
	if(uartMutex == NULL){
		// fallback (mutex yoksa yine de yaz)
		HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
		return;
	}

	if(xSemaphoreTake(uartMutex, timeoutTicks) == pdTRUE){
		HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
		xSemaphoreGive(uartMutex);
	}else {
		// İsteğe bağlı: burada bir “missed log” sayacı tutabilirsin
	}
}

static void uart_safe_print(const char *s)
{
	uart_safe_write((const uint8_t*)s, (uint16_t)strlen(s), pdMS_TO_TICKS(20));
}

void UartTxTask(void *pvParameters) {
	SensorPacket packet;
	char buffer[128];
	int len;

	for (;;) {
		// Queue’dan paket bekle
		if (xQueueReceive(sensorQueue, &packet, portMAX_DELAY) == pdTRUE) {
			len = packet_build_json(&packet, buffer, sizeof(buffer));
			uart_safe_write((uint8_t*)buffer, (uint16_t)len, pdMS_TO_TICKS(50));
		}
	}
}


