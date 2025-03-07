/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
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
#define PHASE0_Pin GPIO_PIN_0
#define PHASE0_GPIO_Port GPIOA
#define PHASE0_EXTI_IRQn EXTI0_IRQn
#define PHASE1_Pin GPIO_PIN_1
#define PHASE1_GPIO_Port GPIOA
#define PHASE1_EXTI_IRQn EXTI1_IRQn
#define PHASE2_Pin GPIO_PIN_2
#define PHASE2_GPIO_Port GPIOA
#define PHASE2_EXTI_IRQn EXTI2_IRQn
#define PHASE3_Pin GPIO_PIN_3
#define PHASE3_GPIO_Port GPIOA
#define PHASE3_EXTI_IRQn EXTI3_IRQn
#define RD_DATA_Pin GPIO_PIN_6
#define RD_DATA_GPIO_Port GPIOA
#define RD_DATA_EXTI_IRQn EXTI9_5_IRQn
#define WR_DATA_Pin GPIO_PIN_7
#define WR_DATA_GPIO_Port GPIOA
#define WR_DATA_EXTI_IRQn EXTI9_5_IRQn
#define WR_PROTECT_Pin GPIO_PIN_10
#define WR_PROTECT_GPIO_Port GPIOB
#define WR_PROTECT_EXTI_IRQn EXTI15_10_IRQn
#define DEVICE_ENABLE_Pin GPIO_PIN_5
#define DEVICE_ENABLE_GPIO_Port GPIOB
#define DEVICE_ENABLE_EXTI_IRQn EXTI9_5_IRQn
#define WR_REQ_Pin GPIO_PIN_9
#define WR_REQ_GPIO_Port GPIOB
#define WR_REQ_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
