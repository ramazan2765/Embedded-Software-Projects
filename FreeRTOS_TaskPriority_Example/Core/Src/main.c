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

/**
 * @file    rtos_priority_sensors.c
 * @author  Ramazan
 * @brief   FreeRTOS - Task Priority Example with MPU9250, TMM103 and Buttons
 * @details Demonstrates task scheduling and preemption with real peripherals.
 *
 * Task Priorities:
 *  - Emergency Task: Priority 4 (Highest, triggered by button press)
 *  - IMU Task: Priority 3 (MPU9250 readings)
 *  - Temp Task: Priority 2 (TMM103 readings)
 *  - Logger Task: Priority 1 (System log + button state)
 */

#include "main.h"
#include "cmsis_os.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* ---- Task Prototypes ---- */
void vLoggerTask(void *pvParameters);
void vSensorTask(void *pvParameters);
void vCommTask(void *pvParameters);
void vEmergencyTask(void *pvParameters);

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
	
      xTaskCreate(vLoggerTask, "Logger", 128, NULL, 1, NULL);         // Priority 1
      xTaskCreate(vSensorTask, "Sensor", 128, NULL, 2, NULL);         // Priority 2
      xTaskCreate(vCommTask, "Comm", 128, NULL, 3, NULL);             // Priority 3
      xTaskCreate(vEmergencyTask, "Emergency", 128, NULL, 4, NULL);   // Priority 4

      /* Start the scheduler */
      vTaskStartScheduler();

  while (1)
  {

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
 * @brief Lowest priority task. It only runs when CPU is idle.
 */
void vLoggerTask(void *pvParameters)
{
	char msg[] = "[Logger] System is running normally...\r\n";
    for(;;)
    {
        HAL_UART_Transmit(&huart1,(uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/**
 * @brief Reads fake sensor data (simulated with random numbers).
 */
void vSensorTask(void *pvParameters)
{
	char msg[50];
	int sensorValue;

    for(;;)
    {
        sensorValue = rand() % 100;  // Simulated sensor data
        snprintf(msg, sizeof(msg), "[Sensor] Value = %d\r\n", sensorValue);
        HAL_UART_Transmit(&huart1,(uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

/**
 * @brief Communication task that sends data over UART.
 */
void vCommTask(void *pvParameters)
{
	char msg[] = "[Comm] Sending data over UART...\r\n";
	for(;;)
    {
		HAL_UART_Transmit(&huart1,(uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Highest priority task. Interrupts all others when it runs.
 */
void vEmergencyTask(void *pvParameters)
{
	char msg[] = "!!! [Emergency] Highest priority task is running !!!\r\n";
    for(;;)
    {
    	HAL_UART_Transmit(&huart1,(uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(5000));  // Runs every 5 seconds
    }
}

