/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder1_1_Pin GPIO_PIN_0
#define Encoder1_1_GPIO_Port GPIOA
#define Encoder1_2_Pin GPIO_PIN_1
#define Encoder1_2_GPIO_Port GPIOA
#define LeftForwardIR_Pin GPIO_PIN_3
#define LeftForwardIR_GPIO_Port GPIOA
#define Encoder2_1_Pin GPIO_PIN_6
#define Encoder2_1_GPIO_Port GPIOA
#define Encoder2_2_Pin GPIO_PIN_7
#define Encoder2_2_GPIO_Port GPIOA
#define left_IR_Pin GPIO_PIN_0
#define left_IR_GPIO_Port GPIOB
#define leftDiagonal_IR_Pin GPIO_PIN_1
#define leftDiagonal_IR_GPIO_Port GPIOB
#define LeftIROUT_Pin GPIO_PIN_2
#define LeftIROUT_GPIO_Port GPIOB
#define rightDiagonal_OUT_Pin GPIO_PIN_10
#define rightDiagonal_OUT_GPIO_Port GPIOB
#define rightForward_IR_Pin GPIO_PIN_11
#define rightForward_IR_GPIO_Port GPIOB
#define rightDiagonal_IR_Pin GPIO_PIN_12
#define rightDiagonal_IR_GPIO_Port GPIOB
#define RightForwardlOUT_Pin GPIO_PIN_13
#define RightForwardlOUT_GPIO_Port GPIOB
#define right_IR_Pin GPIO_PIN_14
#define right_IR_GPIO_Port GPIOB
#define RightIROUT_Pin GPIO_PIN_15
#define RightIROUT_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_8
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_9
#define PWMB_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_10
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_11
#define IN2_GPIO_Port GPIOA
#define LeftForwardIROUT_Pin GPIO_PIN_12
#define LeftForwardIROUT_GPIO_Port GPIOA
#define IN4_Pin GPIO_PIN_3
#define IN4_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_4
#define STBY_GPIO_Port GPIOB
#define leftDiagonalOUT_Pin GPIO_PIN_5
#define leftDiagonalOUT_GPIO_Port GPIOB
#define LeftDiagonalIROUT_Pin GPIO_PIN_6
#define LeftDiagonalIROUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
