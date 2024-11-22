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
#include "cmsis_os.h"
#include "eeprom_emul.h"
#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "bmlite_if_callbacks.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    BMLITE_ERROR_OK = 0,
    BMLITE_ERROR_CAPTURE,
    BMLITE_ERROR_CAPTURE_START,
    BMLITE_ERROR_ENROLL_START,
    BMLITE_ERROR_ENROLL_ADD,
    BMLITE_ERROR_ENROLL_FINISH,
    BMLITE_ERROR_WRONG_ANSWER,
    BMLITE_ERROR_FINGER_WAIT,
    BMLITE_ERROR_IDENTYFY,
    BMLITE_ERROR_TEMPLATE_SAVE,
    BMLITE_ERROR_TEMPLATE_DELETE,
    BMLITE_ERROR_TEMPLATE_COUNT,
    BMLITE_ERROR_TEMPLATE_GETIDS,
    BMLITE_ERROR_IMAGE_EXTRACT,
    BMLITE_ERROR_IMAGE_GETSIZE,
    BMLITE_ERROR_IMAGE_GET,
    BMLITE_ERROR_GETVERSION,
    BMLITE_ERROR_SW_RESET,
    BMLITE_ERROR_CALIBRATE,
    BMLITE_ERROR_CALIBRATE_DELETE,
    BMLITE_ERROR_SEND_CMD,
    BMLITE_ERROR_GET_ARG, // 22
} bmlite_error_t;

//
// Callbacks
//
typedef enum {
    BIOMETRIC_ON_START_CAPTURE = 23,
    BIOMETRIC_ON_FINISH_CAPTURE,
    BIOMETRIC_ON_START_ENROLL,
    BIOMETRIC_ON_FINISH_ENROLL,
    BIOMETRIC_ON_START_ENROLLCAPTURE,
    BIOMETRIC_ON_FINISH_ENROLLCAPTURE,
    BIOMETRIC_ON_IDENTIFY_START,
    BIOMETRIC_ON_IDENTIFY_FINISH,
} bmlite_callback_evt_t;

typedef enum {
    MOTOR_LOCK = 1,
    MOTOR_UNLOCK,
    MOTOR_IS_LOCK,
} app_event_t;

typedef struct {
  app_event_t type;
  bmlite_callback_evt_t cb;
  bool isBioError;
  bmlite_error_t bioError;
  int32_t bioVal;
} app_message_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

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
#define BIO_RST_Pin GPIO_PIN_9
#define BIO_RST_GPIO_Port GPIOB
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
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
