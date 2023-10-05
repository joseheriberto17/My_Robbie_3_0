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
#define S1_Pin GPIO_PIN_0
#define S1_GPIO_Port GPIOC
#define S2_Pin GPIO_PIN_1
#define S2_GPIO_Port GPIOC
#define S3_Pin GPIO_PIN_2
#define S3_GPIO_Port GPIOC
#define S11_Pin GPIO_PIN_3
#define S11_GPIO_Port GPIOC
#define LIGHTS_Pin GPIO_PIN_4
#define LIGHTS_GPIO_Port GPIOA
#define S10_Pin GPIO_PIN_5
#define S10_GPIO_Port GPIOA
#define S7_Pin GPIO_PIN_6
#define S7_GPIO_Port GPIOA
#define S8_Pin GPIO_PIN_7
#define S8_GPIO_Port GPIOA
#define S4_Pin GPIO_PIN_4
#define S4_GPIO_Port GPIOC
#define S9_Pin GPIO_PIN_5
#define S9_GPIO_Port GPIOC
#define S5_Pin GPIO_PIN_0
#define S5_GPIO_Port GPIOB
#define S6_Pin GPIO_PIN_1
#define S6_GPIO_Port GPIOB
#define USER_BUTTON_Pin GPIO_PIN_2
#define USER_BUTTON_GPIO_Port GPIOB
#define EN_SENS_Pin GPIO_PIN_12
#define EN_SENS_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_6
#define SD_CS_GPIO_Port GPIOC
#define BUZZ_Pin GPIO_PIN_7
#define BUZZ_GPIO_Port GPIOC
#define STATUS_LED_RED_Pin GPIO_PIN_8
#define STATUS_LED_RED_GPIO_Port GPIOC
#define ACC_INT1_Pin GPIO_PIN_9
#define ACC_INT1_GPIO_Port GPIOC
#define C1_1_Pin GPIO_PIN_8
#define C1_1_GPIO_Port GPIOA
#define C2_1_Pin GPIO_PIN_9
#define C2_1_GPIO_Port GPIOA
#define CE_RF_Pin GPIO_PIN_11
#define CE_RF_GPIO_Port GPIOA
#define CS_RF_Pin GPIO_PIN_12
#define CS_RF_GPIO_Port GPIOA
#define AMOT2_Pin GPIO_PIN_2
#define AMOT2_GPIO_Port GPIOD
#define AMOT1_Pin GPIO_PIN_4
#define AMOT1_GPIO_Port GPIOB
#define BMOT1_Pin GPIO_PIN_5
#define BMOT1_GPIO_Port GPIOB
#define BMOT2_Pin GPIO_PIN_8
#define BMOT2_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_9
#define STATUS_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void printWelcomeMessage(UART_HandleTypeDef *huart);
float Convert_Pulse_To_Rpm(int32_t counter, int32_t sample_time);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
