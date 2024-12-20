
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
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

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* LS_Service */
  uint16_t              ConnectionHandle;
} Custom_App_Context_t;



/* Private defines ------------------------------------------------------------*/


/* Private macros -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

//static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;


/* Private function prototypes -----------------------------------------------*/
/* LS_Service */



/* Functions Definition ------------------------------------------------------*/
void App_Notification(BLE_Event_Info_t *pNotification)
{
  app_message_t	evt;
  osStatus_t	ret;
  uint8_t         weaponControlVal = 0;
  biometric_control_action_t action = 0;
  switch (pNotification->eventID)
  {

    /* FireArm */
    case BATTERY_STATUS_READ_EVT:
      /* USER CODE BEGIN BATTERY_STATUS_READ_EVT */

      /* USER CODE END BATTERY_STATUS_READ_EVT */
      break;

    case WEAPON_STATUS_READ_EVT:
      /* USER CODE BEGIN WEAPON_STATUS_READ_EVT */

      /* USER CODE END WEAPON_STATUS_READ_EVT */
      break;

    case WEAPON_CONTROL_WRITE_NO_RESP_EVT:
        weaponControlVal = pNotification->DataTransfered.data[0];
        if (weaponControlVal == 1) {
            evt.type      = MOTOR_LOCK;
            ret           = osMessageQueuePut(motorQueueHandle, &evt, 0, 0);
        } else if (weaponControlVal == 2) {
            evt.type      = MOTOR_UNLOCK;
            ret           = osMessageQueuePut(motorQueueHandle, &evt, 0, 0);
        }
        if (ret != osOK) {
            //Error_Handler();
        }
        break;

    //
    // Device lock
    //
    case ACTION_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN ACTION_WRITE_NO_RESP_EVT */

      /* USER CODE END ACTION_WRITE_NO_RESP_EVT */
      break;

    case PIN_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN PIN_WRITE_NO_RESP_EVT */

      /* USER CODE END PIN_WRITE_NO_RESP_EVT */
      break;

    case STATE_READ_EVT:
      /* USER CODE BEGIN STATE_READ_EVT */

      /* USER CODE END STATE_READ_EVT */
      break;

    case PIN_STATUS_READ_EVT:
      /* USER CODE BEGIN PIN_STATUS_READ_EVT */

      /* USER CODE END PIN_STATUS_READ_EVT */
      break;

    case BIOMETRIC_STATUS_READ_EVT:

      break;

    case BIOMETRIC_CONTROL_WRITE_NO_RESP_EVT:
    	action				= (biometric_control_action_t)pNotification->DataTransfered.data[0];
        evt.bioControlVal	= action;
        ret           		= osMessageQueuePut(biometricQueueHandle, &evt, 0, 0);
    	break;
    case BIOMETRIC_FINGERPRINT_LIST_READ_EVT:
        App_Update_Char(BIOMETRIC_FINGERPRINT_LIST, LS_BM_Lite_GetFingerprintList());

      break;
    case NOTIFICATION_COMPLETE_EVT:
      break;

    default:
      break;
  }

  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{

  switch (pNotification->Custom_Evt_Opcode)
  {

    case CUSTOM_CONN_HANDLE_EVT :

      break;

    case CUSTOM_DISCON_HANDLE_EVT :

      break;

    default:

      break;
  }
  return;
}

void Custom_APP_Init(void)
{

  return;
}



/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* LS_Service */


