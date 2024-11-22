/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.h
  * @author  MCD Application Team
  * @brief   Header for custom_stm.c module.
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
#ifndef CUSTOM_STM_H
#define CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LS_Motor_DRV8823.h"
#include <LS_DeviceLock.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* FireArm */
  CUSTOM_STM_BATTERY_STATUS,
  CUSTOM_STM_WEAPON_STATUS,
  CUSTOM_STM_WEAPON_CONTROL,
  CUSTOM_STM_BIOMETRIC_STATUS,
  CUSTOM_STM_BIOMETRIC_CONTROL,
  /* DeviceLock */
  CUSTOM_STM_ACTION,
  CUSTOM_STM_PIN,
  CUSTOM_STM_STATE,
  CUSTOM_STM_PIN_STATUS,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* Battery_Status */
  CUSTOM_STM_BATTERY_STATUS_READ_EVT,
  /* Weapon_Status */
  CUSTOM_STM_WEAPON_STATUS_READ_EVT,
  /* Weapon_Control */
  CUSTOM_STM_WEAPON_CONTROL_WRITE_NO_RESP_EVT,
  /* Action */
  CUSTOM_STM_ACTION_WRITE_NO_RESP_EVT,
  /* Pin */
  CUSTOM_STM_PIN_WRITE_NO_RESP_EVT,
  /* State */
  CUSTOM_STM_STATE_READ_EVT,
  /* Pin_Status */
  CUSTOM_STM_PIN_STATUS_READ_EVT,

  CUSTOM_STM_NOTIFICATION_COMPLETE_EVT,

  CUSTOM_STM_BOOT_REQUEST_EVT
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t 	data[(BLE_EVT_MAX_PARAM_LEN - 2) - 8];
  uint16_t	Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
  uint16_t                      AttrHandle;
} Custom_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
extern uint16_t SizeBattery_Status;
extern uint16_t SizeWeapon_Status;
extern uint16_t SizeWeapon_Control;
extern uint16_t SizeAction;
extern uint16_t SizePin;
extern uint16_t SizeState;
extern uint16_t SizePin_Status;

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define BATTERY_STATUS_HANDLE 14
#define WEAPON_STATUS_HANDLE  16
#define WEAPON_CONTROL_HANDLE 18
#define TX_DATA_HANDLE  20
#define RX_DATA_HANDLE  22

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc(void);
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
tBleStatus Custom_STM_App_Update_Char_Variable_Length(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload, uint8_t size);
tBleStatus Custom_STM_App_Update_Char_Ext(uint16_t Connection_Handle, Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*CUSTOM_STM_H */
