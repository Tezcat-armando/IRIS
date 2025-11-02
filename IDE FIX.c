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
#include "gpio.h"
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

// HC-SR04 Sensor Variables - FIXED
volatile uint32_t echo_start = 0;
volatile uint32_t echo_end = 0;
volatile uint32_t pulse_duration = 0;
volatile uint8_t measurement_done = 0;
volatile uint8_t measurement_in_progress = 0;
volatile uint8_t measurement_timeout = 0;
float distance_cm = 0.0f;

// UART Communication Variables
uint8_t uart_rx_data;
uint8_t uart_rx_index = 0;
char uart_buffer[32];
uint8_t new_data_available = 0;

// System Variables
uint32_t last_debug_time = 0;
uint32_t last_measurement_time = 0;
uint32_t measurement_start_time = 0;
uint8_t operation_mode = 0; // 0=Manual, 1=Auto

// Pin Definitions
#define TRIG_PIN GPIO_PIN_5
#define TRIG_PORT GPIOA

// Sensor Constants
#define MEASUREMENT_INTERVAL_MS 100
#define SENSOR_TIMEOUT_MS 30
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ProcessUARTCommand(char* command);
void SetMotorSpeed(int16_t pwm_value);
void SendEncoderData(void);
void SendDistanceData(void);
void ReadDistanceSensor(void);
void HCSR04_StartMeasurement(void);
void SensorDiagnostic(void);
void MicroDelay(uint16_t delay_us);
void FloatToString(float value, char* buffer, uint8_t buffer_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // Start Encoder Interface
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  
  // Start PWM for Motor
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  
  // Start Input Capture for HC-SR04 - FIXED
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  
  // Start UART Receive Interrupt
  HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
  
  // Initialize motor to stop
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  
  // Startup message dengan info konfigurasi
  char startup_msg[] = "STM32F401 Ready - Motor + HC-SR04 (FIXED V2)\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)startup_msg, strlen(startup_msg), HAL_MAX_DELAY);
  
  // Tampilkan konfigurasi TIM3
  char tim3_msg[100];
  snprintf(tim3_msg, sizeof(tim3_msg), "TIM3 Config - PSC:%lu, ARR:%lu, Clock:1MHz\r\n", 
           htim3.Instance->PSC, htim3.Instance->ARR);
  HAL_UART_Transmit(&huart2, (uint8_t*)tim3_msg, strlen(tim3_msg), HAL_MAX_DELAY);
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
    
    /* Read distance sensor dengan timing yang tepat */
    ReadDistanceSensor();
    
    /* Handle sensor timeout */
    if (measurement_in_progress && !measurement_done) 
    {
        if (HAL_GetTick() - measurement_start_time > SENSOR_TIMEOUT_MS) 
        {
            measurement_timeout = 1;
            measurement_done = 1;
            measurement_in_progress = 0;
            distance_cm = 0.0f;
            
            char timeout_msg[] = "SENSOR TIMEOUT - No echo\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)timeout_msg, strlen(timeout_msg), 100);
        }
    }
    
    /* Send sensor data to ESP32 every 500ms */
    if (HAL_GetTick() - last_debug_time > 500)
    {
        SendDistanceData();
        
        char dist_str[10];
        FloatToString(distance_cm, dist_str, sizeof(dist_str));
        
        char debug_msg[120];
        snprintf(debug_msg, sizeof(debug_msg), 
                 "ENC:%d | PWM:%d | DIST:%scm | MODE:%d | Pulse:%lu | Status:%s\r\n", 
                 encoder_count, motor_pwm, dist_str, operation_mode, pulse_duration,
                 measurement_in_progress ? "Measuring" : "Ready");
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

/* USER CODE BEGIN 4 */

/**
  * @brief Convert float to string without using float formatting
  */
void FloatToString(float value, char* buffer, uint8_t buffer_size)
{
    if (value < 0 || value > 999.9f) {
        snprintf(buffer, buffer_size, "0.0");
        return;
    }
    
    int integer_part = (int)value;
    int decimal_part = (int)((value - integer_part) * 10);
    snprintf(buffer, buffer_size, "%d.%d", integer_part, decimal_part);
}

/**
  * @brief Microsecond delay function - FIXED
  */
void MicroDelay(uint16_t delay_us)
{
    volatile uint32_t count = SystemCoreClock / 1000000 * delay_us / 10;
    while(count--);
}

/**
  * @brief TIM3 Input Capture Callback for HC-SR04 - FIXED VERSION
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        
        // Check current polarity by reading CC1P bit
        uint32_t current_polarity = htim->Instance->CCER & TIM_CCER_CC1P;
        
        if (current_polarity == 0) 
        {
            // Rising Edge Detected - start of echo pulse
            echo_start = capture_value;
            
            // Change to falling edge detection
            htim->Instance->CCER |= TIM_CCER_CC1P;
            
            // Debug
            char msg[50];
            snprintf(msg, sizeof(msg), "Rising edge: %lu\r\n", echo_start);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }
        else 
        {
            // Falling Edge Detected - end of echo pulse
            echo_end = capture_value;
            
            // Change back to rising edge detection
            htim->Instance->CCER &= ~TIM_CCER_CC1P;
            
            // Calculate pulse duration
            if (echo_end >= echo_start) {
                pulse_duration = echo_end - echo_start;
            } else {
                // Handle timer overflow
                pulse_duration = (0xFFFF - echo_start) + echo_end;
            }
            
            // Calculate distance (cm)
            // Timer clock = 1MHz (1 tick = 1us)
            // Speed of sound = 340 m/s = 0.034 cm/us
            distance_cm = (pulse_duration * 0.034f) / 2.0f;
            
            // Validate range (HC-SR04 range: 2cm - 400cm)
            if (distance_cm < 2.0f || distance_cm > 400.0f) {
                distance_cm = 0.0f;
            }
            
            measurement_done = 1;
            measurement_in_progress = 0;
            measurement_timeout = 0;
            
            // Debug
            char msg[80];
            snprintf(msg, sizeof(msg), "Falling edge: %lu, Duration: %lu us, Distance: %.1f cm\r\n", 
                     echo_end, pulse_duration, distance_cm);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }
    }
}

/**
  * @brief Start HC-SR04 Measurement - FIXED VERSION
  */
void HCSR04_StartMeasurement(void)
{
    // Only start new measurement if previous one is done
    if (measurement_in_progress) {
        return;
    }
    
    // Reset measurement flags
    measurement_done = 0;
    measurement_in_progress = 1;
    measurement_timeout = 0;
    
    // Ensure trigger is low first
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    MicroDelay(2);
    
    // Generate 10us trigger pulse
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    MicroDelay(15);  // 15us untuk memastikan
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    
    // Reset timer and variables
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    echo_start = 0;
    echo_end = 0;
    pulse_duration = 0;
    
    measurement_start_time = HAL_GetTick();
    
    // Debug
    char debug_msg[] = "Trigger sent - waiting for echo\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);
}

/**
  * @brief Read Distance Sensor - FIXED VERSION
  */
void ReadDistanceSensor(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // Minimum 60ms between measurements (HC-SR04 requirement)
    if (current_time - last_measurement_time < 60) {
        return;
    }
    
    // Start new measurement if not in progress
    if (!measurement_in_progress) {
        HCSR04_StartMeasurement();
        last_measurement_time = current_time;
    }
}

/**
  * @brief UART Receive Complete Callback
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
  */
void ProcessUARTCommand(char* command)
{
    if (strncmp(command, "PWM:", 4) == 0)
    {
        int16_t pwm_value = atoi(command + 4);
        SetMotorSpeed(pwm_value);
        
        // Send acknowledgment back to ESP32
        char dist_str[10];
        FloatToString(distance_cm, dist_str, sizeof(dist_str));
        
        char response[80];
        snprintf(response, sizeof(response), "ACK:PWM:%d,ENC:%d,DIST:%s\n", 
                 pwm_value, encoder_count, dist_str);
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
    else if (strcmp(command, "TEST_SENSOR") == 0)
    {
        // Manual sensor test
        HCSR04_StartMeasurement();
        
        // Wait for measurement with timeout
        uint32_t start_wait = HAL_GetTick();
        while (!measurement_done && (HAL_GetTick() - start_wait < SENSOR_TIMEOUT_MS)) {
            HAL_Delay(1);
        }
        
        char result[80];
        snprintf(result, sizeof(result), "Sensor Test - Distance: %.1f cm, Duration: %lu us\r\n", 
                 distance_cm, pulse_duration);
        HAL_UART_Transmit(&huart2, (uint8_t*)result, strlen(result), HAL_MAX_DELAY);
    }
    else if (strcmp(command, "RESET_SENSOR") == 0)
    {
        // Reset sensor state
        measurement_in_progress = 0;
        measurement_done = 0;
        measurement_timeout = 0;
        distance_cm = 0.0f;
        
        char reset_msg[] = "Sensor state reset\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)reset_msg, strlen(reset_msg), HAL_MAX_DELAY);
    }
    else
    {
        // Unknown command
        char error_msg[40];
        snprintf(error_msg, sizeof(error_msg), "ERROR:Unknown command - '%s'\n", command);
        HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
    }
}

/**
  * @brief Set Motor Speed and Direction
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
  * @brief Send Encoder Data to ESP32
  */
void SendEncoderData(void)
{
    char message[30];
    snprintf(message, sizeof(message), "ENC:%d\n", encoder_count);
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

/**
  * @brief Send Distance Data to ESP32
  */
void SendDistanceData(void)
{
    char dist_str[10];
    FloatToString(distance_cm, dist_str, sizeof(dist_str));
    
    char message[30];
    snprintf(message, sizeof(message), "DIST:%s\n", dist_str);
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

/**
  * @brief Sensor Diagnostic Function
  */
void SensorDiagnostic(void)
{
    char dist_str[10];
    FloatToString(distance_cm, dist_str, sizeof(dist_str));
    
    char diag_msg[250];
    snprintf(diag_msg, sizeof(diag_msg),
             "=== HC-SR04 DIAGNOSTIC ===\r\n"
             "Start: %lu | End: %lu | Duration: %lu us\r\n"
             "Distance: %s cm | InProgress: %d | Done: %d | Timeout: %d\r\n"
             "Timer CNT: %lu | CCER: 0x%lX\r\n"
             "Last Measure: %lu ms ago\r\n"
             "==========================\r\n",
             echo_start, echo_end, pulse_duration,
             dist_str, measurement_in_progress, measurement_done, measurement_timeout,
             __HAL_TIM_GET_COUNTER(&htim3), htim3.Instance->CCER,
             HAL_GetTick() - last_measurement_time);
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_msg, strlen(diag_msg), HAL_MAX_DELAY);
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
