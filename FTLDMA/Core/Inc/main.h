/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/******SERIAL COMMUNICATION SETTINGS******/
#define UART4_RX_BUFFER_SIZE   128
#define UART5_RX_BUFFER_SIZE   128
#define USART1_RX_BUFFER_SIZE  128
#define USART2_RX_BUFFER_SIZE  128
#define USART3_RX_BUFFER_SIZE  128

extern uint8_t uart4_rx_buffer[UART4_RX_BUFFER_SIZE];
extern uint8_t uart5_rx_buffer[UART5_RX_BUFFER_SIZE];
extern uint8_t usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
extern uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
extern uint8_t usart3_rx_buffer[USART3_RX_BUFFER_SIZE];

extern volatile uint16_t uart4_prev_index;
extern volatile uint16_t uart5_prev_index;
extern volatile uint16_t usart1_prev_index;
extern volatile uint16_t usart2_prev_index;
extern volatile uint16_t usart3_prev_index;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void UpdateDeltaTime(void);
void UART_IdleProcess(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t buffer_size);
void ProcessData(uint8_t* data, uint16_t len, UART_HandleTypeDef *huart);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define STERTH_CHARGE_Pin GPIO_PIN_3
#define STERTH_CHARGE_GPIO_Port GPIOC
#define STRETCH_CAPTURE_Pin GPIO_PIN_0
#define STRETCH_CAPTURE_GPIO_Port GPIOA
#define EMG_Pin GPIO_PIN_1
#define EMG_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define FAN_PUL_Pin GPIO_PIN_0
#define FAN_PUL_GPIO_Port GPIOB
#define SMA_PUL_Pin GPIO_PIN_7
#define SMA_PUL_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SMA_DIR_Pin GPIO_PIN_6
#define SMA_DIR_GPIO_Port GPIOB
#define FAN_DIR_Pin GPIO_PIN_7
#define FAN_DIR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
