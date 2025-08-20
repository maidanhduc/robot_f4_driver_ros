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
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern osMutexId pidMutex;

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
#define S5_Pin GPIO_PIN_5
#define S5_GPIO_Port GPIOE
#define S6_Pin GPIO_PIN_6
#define S6_GPIO_Port GPIOE
#define PWM_0_Pin GPIO_PIN_0
#define PWM_0_GPIO_Port GPIOA
#define PWM_1_Pin GPIO_PIN_1
#define PWM_1_GPIO_Port GPIOA
#define PWM_2_Pin GPIO_PIN_2
#define PWM_2_GPIO_Port GPIOA
#define PWM_3_Pin GPIO_PIN_3
#define PWM_3_GPIO_Port GPIOA
#define EN1A_Pin GPIO_PIN_5
#define EN1A_GPIO_Port GPIOA
#define EN2A_Pin GPIO_PIN_6
#define EN2A_GPIO_Port GPIOA
#define EN2B_Pin GPIO_PIN_7
#define EN2B_GPIO_Port GPIOA
#define EN0A_Pin GPIO_PIN_9
#define EN0A_GPIO_Port GPIOE
#define EN0B_Pin GPIO_PIN_11
#define EN0B_GPIO_Port GPIOE
#define MR0_in1_Pin GPIO_PIN_12
#define MR0_in1_GPIO_Port GPIOB
#define MR0_in2_Pin GPIO_PIN_13
#define MR0_in2_GPIO_Port GPIOB
#define MR1_in1_Pin GPIO_PIN_14
#define MR1_in1_GPIO_Port GPIOB
#define MR1_in2_Pin GPIO_PIN_15
#define MR1_in2_GPIO_Port GPIOB
#define MR2_in2_Pin GPIO_PIN_9
#define MR2_in2_GPIO_Port GPIOD
#define MR3_in1_Pin GPIO_PIN_10
#define MR3_in1_GPIO_Port GPIOD
#define MR3_in2_Pin GPIO_PIN_11
#define MR3_in2_GPIO_Port GPIOD
#define EN3A_Pin GPIO_PIN_12
#define EN3A_GPIO_Port GPIOD
#define EN3B_Pin GPIO_PIN_13
#define EN3B_GPIO_Port GPIOD
#define S2_Pin GPIO_PIN_6
#define S2_GPIO_Port GPIOC
#define S1_Pin GPIO_PIN_7
#define S1_GPIO_Port GPIOC
#define S4_Pin GPIO_PIN_8
#define S4_GPIO_Port GPIOC
#define S3_Pin GPIO_PIN_9
#define S3_GPIO_Port GPIOC
#define EN1B_Pin GPIO_PIN_3
#define EN1B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
