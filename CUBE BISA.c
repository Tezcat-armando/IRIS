/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE BEGIN PV */
// Motor Control Variables
uint16_t motor_pwm = 0;
uint8_t motor_direction = 0;
int16_t encoder_count = 0;

// UART Communication Variables
uint8_t uart_rx_data;
uint8_t uart_rx_index = 0;
char uart_buffer[32];
uint8_t new_data_available = 0;

// Debug Variables
uint32_t last_debug_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void ProcessUARTCommand(char* command);
void SetMotorSpeed(int16_t pwm_value);
void SendEncoderData(void);
/* USER CODE END PFP */

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  // Start Encoder Interface
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // Start PWM for Motor
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // Start UART Receive Interrupt
  HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);

  // Initialize motor to stop
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

  // Startup message
  char startup_msg[] = "STM32F401 Motor Controller Ready\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)startup_msg, strlen(startup_msg), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Process new UART commands */
    if (new_data_available)
    {
        ProcessUARTCommand(uart_buffer);
        new_data_available = 0;
    }

    /* Read encoder count */
    encoder_count = __HAL_TIM_GET_COUNTER(&htim2);

    /* Debug output every 500ms */
    if (HAL_GetTick() - last_debug_time > 500)
    {
        char debug_msg[50];
        snprintf(debug_msg, sizeof(debug_msg), "Encoder: %d | PWM: %d | Dir: %d\r\n",
                 encoder_count, motor_pwm, motor_direction);
        HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
        last_debug_time = HAL_GetTick();
    }

    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
  * @brief UART Receive Complete Callback
  * @param huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (uart_rx_data == '\n' || uart_rx_data == '\r')
        {
            if (uart_rx_index > 0)
            {
                uart_buffer[uart_rx_index] = '\0';
                new_data_available = 1;
            }
            uart_rx_index = 0;
        }
        else
        {
            if (uart_rx_index < (32 - 1))
            {
                uart_buffer[uart_rx_index++] = uart_rx_data;
            }
            else
            {
                uart_rx_index = 0;
            }
        }

        HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
    }
}

/**
  * @brief Process UART Command from ESP32
  * @param command: Received command string
  * @retval None
  */
void ProcessUARTCommand(char* command)
{
    if (strncmp(command, "PWM:", 4) == 0)
    {
        int16_t pwm_value = atoi(command + 4);
        SetMotorSpeed(pwm_value);

        // Send acknowledgment back to ESP32
        char response[50];
        snprintf(response, sizeof(response), "ACK:PWM:%d,ENC:%d\n", pwm_value, encoder_count);
        HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
    }
    else if (strcmp(command, "READ") == 0)
    {
        SendEncoderData();
    }
    else
    {
        // Unknown command
        char error_msg[30];
        snprintf(error_msg, sizeof(error_msg), "ERROR:Unknown command\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
    }
}

/**
  * @brief Set Motor Speed and Direction
  * @param pwm_value: PWM value (-255 to 255)
  * @retval None
  */
void SetMotorSpeed(int16_t pwm_value)
{
    // Constrain PWM value
    if (pwm_value > 255) pwm_value = 255;
    if (pwm_value < -255) pwm_value = -255;

    // Set direction and PWM
    if (pwm_value > 0) {
        // Forward
        motor_direction = 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
        motor_pwm = pwm_value;
    }
    else if (pwm_value < 0) {
        // Reverse
        motor_direction = 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        motor_pwm = -pwm_value;
    }
    else {
        // Stop
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        motor_pwm = 0;
    }

    // Set PWM duty cycle (assuming 8-bit resolution)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_pwm);
}

/**
  * @brief Send Encoder Data to ESP32
  * @retval None
  */
void SendEncoderData(void)
{
    char message[30];
    snprintf(message, sizeof(message), "ENC:%d\n", encoder_count);
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
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

#ifdef  USE_FULL_ASSERT
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
