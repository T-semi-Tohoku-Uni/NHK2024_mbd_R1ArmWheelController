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
#define Arm3Switch_Pin GPIO_PIN_0
#define Arm3Switch_GPIO_Port GPIOA
#define Arm3Switch_EXTI_IRQn EXTI0_IRQn
#define Arm2Switch_Pin GPIO_PIN_1
#define Arm2Switch_GPIO_Port GPIOA
#define Arm2Switch_EXTI_IRQn EXTI1_IRQn
#define Arm1Switch_Pin GPIO_PIN_6
#define Arm1Switch_GPIO_Port GPIOA
#define Arm1Switch_EXTI_IRQn EXTI9_5_IRQn
#define Arm0Switch_Pin GPIO_PIN_7
#define Arm0Switch_GPIO_Port GPIOA
#define Arm0Switch_EXTI_IRQn EXTI9_5_IRQn
#define BoardLED_Pin GPIO_PIN_2
#define BoardLED_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
