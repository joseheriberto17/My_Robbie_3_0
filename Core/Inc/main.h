/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STATUS_LED_RED_Pin GPIO_PIN_8
#define STATUS_LED_RED_GPIO_Port GPIOC
#define C1_1_Pin GPIO_PIN_8
#define C1_1_GPIO_Port GPIOA
#define C2_1_Pin GPIO_PIN_9
#define C2_1_GPIO_Port GPIOA
#define C2_2_Pin GPIO_PIN_15
#define C2_2_GPIO_Port GPIOA
#define AMOT2_Pin GPIO_PIN_2
#define AMOT2_GPIO_Port GPIOD
#define C1_2_Pin GPIO_PIN_3
#define C1_2_GPIO_Port GPIOB
#define AMOT1_Pin GPIO_PIN_4
#define AMOT1_GPIO_Port GPIOB
#define BMOT1_Pin GPIO_PIN_5
#define BMOT1_GPIO_Port GPIOB
#define BMOT2_Pin GPIO_PIN_8
#define BMOT2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void printWelcomeMessage(UART_HandleTypeDef *huart);
float Convert_Pulse_To_Rpm(int32_t counter, int32_t time);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
