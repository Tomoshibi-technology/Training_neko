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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MD_nFAULT_Pin GPIO_PIN_4
#define MD_nFAULT_GPIO_Port GPIOA
#define CS1_Encoder_Pin GPIO_PIN_0
#define CS1_Encoder_GPIO_Port GPIOB
#define CS1_MD_Pin GPIO_PIN_1
#define CS1_MD_GPIO_Port GPIOB
#define Slide0_Pin GPIO_PIN_12
#define Slide0_GPIO_Port GPIOB
#define MD_CAL_Pin GPIO_PIN_13
#define MD_CAL_GPIO_Port GPIOB
#define DIP1_Pin GPIO_PIN_14
#define DIP1_GPIO_Port GPIOB
#define DIP3_Pin GPIO_PIN_15
#define DIP3_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOA
#define MD_ENABLE_Pin GPIO_PIN_12
#define MD_ENABLE_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_6
#define LED0_GPIO_Port GPIOB
#define DIP2_Pin GPIO_PIN_7
#define DIP2_GPIO_Port GPIOB
#define DIP0_Pin GPIO_PIN_9
#define DIP0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
