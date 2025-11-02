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

// HC-SR04 Sensor Variables
volatile uint32_t echo_start = 0;
volatile uint32_t echo_end = 0;
volatile uint32_t pulse_duration = 0;
volatile uint8_t measurement_done = 0;
volatile uint8_t measurement_timeout = 0;
float distance_cm = 0;

// UART Communication Variables
uint8_t uart_rx_data;
uint8_t uart_rx_index = 0;
char uart_buffer[32];
uint8_t new_data_available = 0;

// System Variables
uint32_t last_sensor_time = 0;
uint32_t last_debug_time = 0;
uint32_t last_trigger_time = 0;
uint8_t operation_mode = 0; // 0=Manual, 1=Auto

// Pin Definitions
#define TRIG_PIN GPIO_PIN_5
#define TRIG_PORT GPIOA

// Sensor Constants
#define SENSOR_TIMEOUT_MS 50
#define MEASUREMENT_INTERVAL_MS 100
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void ProcessUARTCommand(char* command);
void SetMotorSpeed(int16_t pwm_value);
void SendEncoderData(void);
void SendDistanceData(void);
void ReadDistanceSensor(void);
void SensorDiagnostic(void);
void Debug_TIM3_Config(void);
void MicroDelay(uint16_t delay_us);
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
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // Start Encoder Interface
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  
  // Start PWM for Motor
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  
  // Start Input Capture for HC-SR04
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  
  // Start UART Receive Interrupt
  HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
  
  // Initialize motor to stop
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  
  // Debug TIM3 configuration
  Debug_TIM3_Config();
  
  // Startup message
  char startup_msg[] = "STM32F401 Ready - Motor + HC-SR04 (FIXED)\r\n";
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
    
    /* Read distance sensor dengan interval yang tepat */
    if (HAL_GetTick() - last_trigger_time >= MEASUREMENT_INTERVAL_MS)
    {
        ReadDistanceSensor();
        last_trigger_time = HAL_GetTick();
    }
    
    /* Handle sensor timeout */
    if (!measurement_done && (HAL_GetTick() - last_trigger_time > SENSOR_TIMEOUT_MS))
    {
        measurement_timeout = 1;
        measurement_done = 1;
        distance_cm = 0.0f;
        
        char timeout_msg[] = "SENSOR TIMEOUT - No echo detected\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)timeout_msg, strlen(timeout_msg), HAL_MAX_DELAY);
    }
    
    /* Send sensor data to ESP32 every 500ms */
    if (HAL_GetTick() - last_debug_time > 500)
    {
        SendDistanceData();
        
        char debug_msg[100];
        snprintf(debug_msg, sizeof(debug_msg), 
                 "ENC:%d | PWM:%d | DIST:%.1fcm | MODE:%d | Pulse:%lu\r\n", 
                 encoder_count, motor_pwm, distance_cm, operation_mode, pulse_duration);
        HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
        last_debug_time = HAL_GetTick();
        
        // Sensor diagnostic setiap 2 detik
        static uint32_t last_diag_time = 0;
        if (HAL_GetTick() - last_diag_time > 2000) {
            SensorDiagnostic();
            last_diag_time = HAL_GetTick();
        }
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 (HC-SR04 Trigger) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 (Motor Direction) */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
  * @brief Microsecond delay function
  * @param delay_us: delay in microseconds
  * @retval None
  */
void MicroDelay(uint16_t delay_us)
{
    volatile uint32_t count = SystemCoreClock / 1000000 * delay_us / 5;
    while(count--);
}

/**
  * @brief Debug TIM3 Configuration
  * @retval None
  */
void Debug_TIM3_Config(void)
{
    char debug_msg[150];
    snprintf(debug_msg, sizeof(debug_msg), 
             "TIM3 Config - PSC:%lu, ARR:%lu, Clock:%lu Hz\r\n", 
             htim3.Instance->PSC, 
             htim3.Instance->ARR,
             HAL_RCC_GetPCLK1Freq() * 2); // TIM3 pada APB1
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
}

/**
  * @brief TIM3 Input Capture Callback for HC-SR04 - FIXED VERSION
  * @param htim: TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            
            // Debug rising/falling edge detection
            char edge_msg[50];
            
            if (__HAL_TIM_GET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1) == TIM_INPUTCHANNELPOLARITY_RISING)
            {
                // Rising edge - start of pulse
                echo_start = capture;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
                
                snprintf(edge_msg, sizeof(edge_msg), "Rising Edge: %lu\r\n", echo_start);
                HAL_UART_Transmit(&huart2, (uint8_t*)edge_msg, strlen(edge_msg), HAL_MAX_DELAY);
            }
            else
            {
                // Falling edge - end of pulse
                echo_end = capture;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
                
                // Calculate pulse duration
                if (echo_end >= echo_start) {
                    pulse_duration = echo_end - echo_start;
                } else {
                    // Handle timer overflow
                    pulse_duration = (0xFFFF - echo_start) + echo_end;
                }
                
                // Calculate distance in cm
                // Timer clock = 1MHz (1 tick = 1us), speed of sound = 0.034 cm/us
                distance_cm = (pulse_duration * 0.034f) / 2.0f;
                
                // Validate distance range (HC-SR04 range: 2cm - 400cm)
                if (distance_cm < 2.0f || distance_cm > 400.0f) {
                    distance_cm = 0.0f;
                }
                
                measurement_done = 1;
                measurement_timeout = 0;
                
                snprintf(edge_msg, sizeof(edge_msg), 
                         "Falling Edge: %lu, Duration: %lu us, Dist: %.1f cm\r\n", 
                         echo_end, pulse_duration, distance_cm);
                HAL_UART_Transmit(&huart2, (uint8_t*)edge_msg, strlen(edge_msg), HAL_MAX_DELAY);
            }
        }
    }
}

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
        char response[80];
        snprintf(response, sizeof(response), "ACK:PWM:%d,ENC:%d,DIST:%.1f\n", 
                 pwm_value, encoder_count, distance_cm);
        HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
    }
    else if (strcmp(command, "READ") == 0)
    {
        SendEncoderData();
    }
    else if (strcmp(command, "DIST") == 0)
    {
        SendDistanceData();
    }
    else if (strncmp(command, "MODE:", 5) == 0)
    {
        operation_mode = atoi(command + 5);
        char mode_msg[30];
        snprintf(mode_msg, sizeof(mode_msg), "MODE:%d\n", operation_mode);
        HAL_UART_Transmit(&huart2, (uint8_t*)mode_msg, strlen(mode_msg), HAL_MAX_DELAY);
    }
    else if (strcmp(command, "DIAG") == 0)
    {
        SensorDiagnostic();
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
    
    // Set PWM duty cycle
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_pwm);
}

/**
  * @brief Read Distance from HC-SR04 Sensor - FIXED VERSION
  * @retval None
  */
void ReadDistanceSensor(void)
{
    // Reset measurement flags
    measurement_done = 0;
    measurement_timeout = 0;
    
    // Pastikan trigger LOW terlebih dahulu
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    MicroDelay(2);
    
    // Generate 10us trigger pulse
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    MicroDelay(10);  // 10us delay yang akurat
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    
    // Reset timer counter sebelum memulai pengukuran
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    echo_start = 0;
    echo_end = 0;
    
    char trigger_msg[] = "Trigger sent\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)trigger_msg, strlen(trigger_msg), HAL_MAX_DELAY);
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

/**
  * @brief Send Distance Data to ESP32
  * @retval None
  */
void SendDistanceData(void)
{
    char message[30];
    snprintf(message, sizeof(message), "DIST:%.1f\n", distance_cm);
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

/**
  * @brief Sensor Diagnostic Function
  * @retval None
  */
void SensorDiagnostic(void)
{
    char diag_msg[200];
    snprintf(diag_msg, sizeof(diag_msg),
             "=== SENSOR DIAGNOSTIC ===\r\n"
             "Start: %lu | End: %lu | Duration: %lu\r\n"
             "MeasDone: %d | Timeout: %d | Distance: %.1f cm\r\n"
             "Timer CNT: %lu | ARR: %lu\r\n"
             "=========================\r\n",
             echo_start, echo_end, pulse_duration,
             measurement_done, measurement_timeout, distance_cm,
             __HAL_TIM_GET_COUNTER(&htim3), __HAL_TIM_GET_AUTORELOAD(&htim3));
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), HAL_MAX_DELAY);
}

/* USER CODE END 4 */
