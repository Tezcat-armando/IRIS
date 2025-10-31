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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Motor & Encoder Variables
uint16_t encoder_count = 0;
uint16_t motor_speed = 0;
uint8_t motor_direction = 0;

// UART Communication Variables
uint8_t uart_rx_data;
uint8_t uart_rx_index = 0;
char uart_buffer[32];
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
        
        // Restart UART receive
        HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
    }
}

// Redirect printf to UART
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  // Start Encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  
  // Start PWM for Motor Control
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  
  // Start UART Receive Interrupt - 115200 BAUD
  HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
  
  // Initialize motor to stop
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  
  printf("\r\n🚀 STM32F401 Motor Controller - 115200 baud\r\n");
  printf("📡 UART Ready - Waiting for ESP32 Commands...\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    if (new_data_available)
    {
        ProcessUARTCommand();
        new_data_available = 0;
    }
    
    ControlMotorDirection();
    SetMotorPWM();
    
    encoder_count = __HAL_TIM_GET_COUNTER(&htim2);
    
    if (HAL_GetTick() - last_debug_time > 500)
    {
        printf("📊 Encoder: %6d | PWM: %3d | Dir: %d\r\n", 
               encoder_count, motor_speed, motor_direction);
        last_debug_time = HAL_GetTick();
    }
    
    HAL_Delay(10);
  }
}

/* USER CODE BEGIN 4 */
void ProcessUARTCommand(void)
{
    received_pwm = atoi(uart_buffer);
    
    if (received_pwm > 255) received_pwm = 255;
    if (received_pwm < -255) received_pwm = -255;
    
    if (received_pwm < 0) {
        motor_direction = 1;
        motor_speed = -received_pwm;
    } else {
        motor_direction = 0;
        motor_speed = received_pwm;
    }
    
    printf("📥 Received: %s -> PWM: %d\r\n", uart_buffer, received_pwm);
}

void ControlMotorDirection(void)
{
    if (motor_speed > 0) {
        if (motor_direction == 0) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
        }
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    }
}

void SetMotorPWM(void)
{
    if (motor_speed > 999) motor_speed = 999;
    uint16_t pwm_compare = (motor_speed * 1023) / 255;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_compare);
}
/* USER CODE END 4 */
