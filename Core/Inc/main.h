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
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ACC_IRQ2_Pin GPIO_PIN_8
#define ACC_IRQ2_GPIO_Port GPIOB
#define BACKSTRAP_Pin GPIO_PIN_9
#define BACKSTRAP_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_0
#define LED_B_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOA
#define MOT_IN1_Pin GPIO_PIN_8
#define MOT_IN1_GPIO_Port GPIOA
#define MOT_IN2_Pin GPIO_PIN_9
#define MOT_IN2_GPIO_Port GPIOA
#define MB_SW3_Pin GPIO_PIN_2
#define MB_SW3_GPIO_Port GPIOB
#define MB_SW1_Pin GPIO_PIN_0
#define MB_SW1_GPIO_Port GPIOB
#define MB_SW2_Pin GPIO_PIN_1
#define MB_SW2_GPIO_Port GPIOB
#define KEEPON_Pin GPIO_PIN_4
#define KEEPON_GPIO_Port GPIOE
#define MUX_CONTROL_Pin GPIO_PIN_10
#define MUX_CONTROL_GPIO_Port GPIOA
#define MOT_FAULT_Pin GPIO_PIN_15
#define MOT_FAULT_GPIO_Port GPIOA
#define BAT_MEAS_EN_Pin GPIO_PIN_3
#define BAT_MEAS_EN_GPIO_Port GPIOB
#define BC_STATUS_Pin GPIO_PIN_4
#define BC_STATUS_GPIO_Port GPIOB
#define ACC_IRQ1_Pin GPIO_PIN_5
#define ACC_IRQ1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
