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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Motor & Encoder Variables
uint16_t encoder_count = 0;
uint16_t motor_speed = 0;
uint8_t motor_direction = 0;
int16_t pwm_value = 0;

// UART Communication Variables
uint8_t uart_rx_data;
uint8_t uart_rx_index = 0;
char uart_buffer[UART_BUFFER_SIZE];
int16_t received_pwm = 0;
uint8_t new_data_available = 0;

// System Variables
uint32_t last_debug_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ControlMotorDirection(void);
void SetMotorPWM(void);
void ProcessUARTCommand(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// UART Receive Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Check for newline or carriage return (end of message)
        if (uart_rx_data == '\n' || uart_rx_data == '\r') 
        {
            if (uart_rx_index > 0)  // Only process if we have data
            {
                uart_buffer[uart_rx_index] = '\0'; // Null terminate
                new_data_available = 1;
            }
            uart_rx_index = 0;
        } 
        else 
        {
            // Store received character
            if (uart_rx_index < (UART_BUFFER_SIZE - 1)) 
            {
                uart_buffer[uart_rx_index++] = uart_rx_data;
            }
            else 
            {
                // Buffer overflow, reset
                uart_rx_index = 0;
            }
        }
        
        // Restart UART receive for next character
        HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
    }
}

// Redirect printf to UART for debugging
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  // Start Encoder in Interface Mode
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  
  // Start PWM for Motor Control
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  
  // Start UART Receive Interrupt
  HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
  
  // Initialize motor to stop
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  
  printf("\r\n🚀 STM32F401 Motor Controller Started\r\n");
  printf("📡 UART Ready - Waiting for ESP32 Commands...\r\n");
  printf("🎯 Format: PWM value (0-255) followed by newline\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Process new UART commands from ESP32 */
    if (new_data_available)
    {
        ProcessUARTCommand();
        new_data_available = 0;
    }
    
    /* Update motor control based on received PWM */
    ControlMotorDirection();
    SetMotorPWM();
    
    /* Read encoder count */
    encoder_count = __HAL_TIM_GET_COUNTER(&htim2);
    
    /* Debug output every 500ms */
    if (HAL_GetTick() - last_debug_time > 500)
    {
        printf("📊 Encoder: %6d | PWM: %3d | Dir: %d\r\n", 
               encoder_count, motor_speed, motor_direction);
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

/* USER CODE BEGIN 4 */

/**
  * @brief Process UART command received from ESP32
  * @retval None
  */
void ProcessUARTCommand(void)
{
    // Parse PWM value from ESP32
    received_pwm = atoi(uart_buffer);
    
    // Validate and constrain PWM value
    if (received_pwm > 255) received_pwm = 255;
    if (received_pwm < -255) received_pwm = -255;
    
    // Set direction based on PWM value sign
    if (received_pwm < 0) {
        motor_direction = 1; // Reverse direction
        motor_speed = -received_pwm; // Convert to positive for PWM value
    } else {
        motor_direction = 0; // Forward direction  
        motor_speed = received_pwm;
    }
    
    printf("📥 Received: %s -> PWM: %d, Dir: %d\r\n", 
           uart_buffer, received_pwm, motor_direction);
}

/**
  * @brief Control motor direction based on PWM sign
  * @retval None
  */
void ControlMotorDirection(void)
{
    if (motor_speed > 0) {
        if (motor_direction == 0) {
            // Forward direction
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        } else {
            // Reverse direction  
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
        }
    } else {
        // Stop motor
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    }
}

/**
  * @brief Set PWM value for motor speed control
  * @retval None  
  */
void SetMotorPWM(void)
{
    // Constrain speed to valid PWM range
    if (motor_speed > 999) {
        motor_speed = 999;
    }
    
    // Set PWM compare value (assuming 10-bit resolution: 0-1023)
    // Map 0-255 to 0-1023 for 10-bit PWM
    uint16_t pwm_compare = (motor_speed * 1023) / 255;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_compare);
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
  
  // Blink LED to indicate error
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Built-in LED on Black Pill
    HAL_Delay(200);
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
