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
#include "ble_types.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BATTERY_STATUS,
  WEAPON_STATUS,
  WEAPON_CONTROL,

  ACTION,
  PIN,
  STATE,
  PIN_STATUS,

  BIOMETRIC_STATUS,
  BIOMETRIC_CONTROL,
  BIOMETRIC_FINGERPRINT_LIST,
} BLE_CharID_t;

typedef enum
{
  /* Battery_Status */
  BATTERY_STATUS_READ_EVT,
  /* Weapon_Status */
  WEAPON_STATUS_READ_EVT,
  /* Weapon_Control */
  WEAPON_CONTROL_WRITE_NO_RESP_EVT,
  /* Action */
  ACTION_WRITE_NO_RESP_EVT,
  /* Pin */
  PIN_WRITE_NO_RESP_EVT,
  /* State */
  STATE_READ_EVT,
  /* Pin_Status */
  PIN_STATUS_READ_EVT,

  /* Weapon_Control */
  BIOMETRIC_STATUS_READ_EVT,
  /* Weapon_Control */
  BIOMETRIC_CONTROL_WRITE_NO_RESP_EVT,

  BIOMETRIC_FINGERPRINT_LIST_READ_EVT,

  NOTIFICATION_COMPLETE_EVT,

  BOOT_REQUEST_EVT
} BLE_Char_Evt_t;

typedef struct
{
  uint8_t 	data[(BLE_EVT_MAX_PARAM_LEN - 2) - 8];
  uint16_t	Length;
} BLE_Data_t;

typedef struct
{
  BLE_Char_Evt_t  eventID;
  BLE_Data_t      DataTransfered;
  uint16_t        ConnectionHandle;
  uint8_t         ServiceInstance;
  uint16_t        AttrHandle;
} BLE_Event_Info_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
extern uint16_t SizeBattery_Status;
extern uint16_t SizeWeapon_Status;
extern uint16_t SizeWeapon_Control;
extern uint16_t SizeBiometric_Status;
extern uint16_t SizeBiometric_Control;
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
void App_Notification(BLE_Event_Info_t *pNotification);
tBleStatus App_Update_Char(BLE_CharID_t CharOpcode,  uint8_t *pPayload);
tBleStatus App_Update_Char_Variable_Length(BLE_CharID_t CharOpcode, uint8_t *pPayload, uint8_t size);
tBleStatus App_Update_Char_Ext(uint16_t Connection_Handle, BLE_CharID_t CharOpcode, uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*CUSTOM_STM_H */
