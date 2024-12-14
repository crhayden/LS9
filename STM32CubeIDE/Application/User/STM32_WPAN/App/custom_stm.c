/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.c
  * @author  MCD Application Team
  * @brief   Custom Example Service.
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

/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"
#include "custom_stm.h"

/* USER CODE BEGIN Includes */
#include <LS_Biometric_Lite.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  FirearmService;              /**< FireArm handle */
  uint16_t  BatteryStatusChar;       /**< Battery_Status handle */
  uint16_t  WeaponStatusChar;        /**< Weapon_Status handle */
  uint16_t  WeaponControlChar;       /**< Weapon_Control handle */

  uint16_t  DeviceLockService;           /**< DeviceLock handle */
  uint16_t  ActionChar;               /**< Action handle */
  uint16_t  PinChar;                  /**< Pin handle */
  uint16_t  StateChar;                /**< State handle */
  uint16_t  PinStatusChar;           /**< Pin_Status handle */

  uint16_t  BiometricService;              /**< FireArm handle */
  uint16_t  BiometricErrorChar;     /**< Weapon_Control handle */
  uint16_t  BiometricControlChar;    /**< Weapon_Control handle */

}BLE_Handles_t;

extern uint16_t Connection_Handle;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint16_t _BatStatusSize = 1;
uint16_t _WeaponStatusSize = 1;
uint16_t _WeaponControlSize = 1;
uint16_t _BiometricStatusSize = 1;
uint16_t _BiometricControlSize = 1;
uint16_t _ActionSize = 1;
uint16_t _PinSize = 4;
uint16_t _StateSize = 1;
uint16_t _PinStatusSize = 1;

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
static BLE_Handles_t Handles;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Event_Handler(void *pckt);

static tBleStatus Generic_STM_App_Update_Char_Ext(uint16_t ConnectionHandle, uint16_t ServiceHandle, uint16_t CharHandle, uint16_t CharValueLen, uint8_t *pPayload);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)
                                                                        
#define COPY_FIREARM_UUID(uuid_struct)              COPY_UUID_128(uuid_struct,0x4C,0x4F,0x44,0x45,0x53,0x54,0x41,0x52,0x46,0x41,0x53,0x45,0x52,0x56,0x00,0x00)
#define COPY_BATTERY_STATUS_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x4C,0x4F,0x44,0x46,0x53,0x54,0x41,0x52,0x46,0x41,0x53,0x45,0x52,0x56,0x00,0x00)
#define COPY_WEAPON_STATUS_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0x4C,0x4F,0x44,0x47,0x53,0x54,0x41,0x52,0x46,0x41,0x53,0x45,0x52,0x56,0x00,0x00)
#define COPY_WEAPON_CONTROL_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x4C,0x4F,0x44,0x48,0x53,0x54,0x41,0x52,0x46,0x41,0x53,0x45,0x52,0x56,0x00,0x00)

#define COPY_DEVICELOCK_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x44,0x45,0x46,0x49,0x43,0x45,0x4C,0x4F,0x43,0x4B,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_ACTION_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x44,0x45,0x46,0x4A,0x43,0x45,0x4C,0x4F,0x43,0x4B,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_PIN_UUID(uuid_struct)              COPY_UUID_128(uuid_struct,0x44,0x45,0x46,0x4B,0x43,0x45,0x4C,0x4F,0x43,0x4B,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_STATE_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x44,0x45,0x46,0x4C,0x43,0x45,0x4C,0x4F,0x43,0x4B,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_PIN_STATUS_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x44,0x45,0x46,0x4D,0x43,0x45,0x4C,0x4F,0x43,0x4B,0x00,0x00,0x00,0x00,0x00,0x00)
                                                                        
#define COPY_BIOMETRIC_UUID(uuid_struct)              COPY_UUID_128(uuid_struct,0x42,0x69,0x6F,0x6D,0x64,0x74,0x72,0x69,0x63,0x00,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_BIOMETRIC_STATUS_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x42,0x69,0x6F,0x6E,0x64,0x74,0x72,0x69,0x63,0x00,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_BIOMETRIC_CONTROL_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x42,0x69,0x6F,0x6F,0x64,0x74,0x72,0x69,0x63,0x00,0x00,0x00,0x00,0x00,0x00,0x00)


/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  aci_gatt_write_permit_req_event_rp0   *write_perm_req;
  aci_gatt_read_permit_req_event_rp0    *read_req;
  aci_gatt_notification_complete_event_rp0    *notification_complete;
  BLE_Event_Info_t     Notification;
  LS_DeviceLock_States 	lockState 	= DISABLED;
  LS_DeviceLock_Actions	action		= aNONE;
  uint8_t pinCode[4] = {0};

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch (event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch (blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:

          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
          Notification.DataTransfered.data[0] = attribute_modified->Attr_Data[0];
          return_value = SVCCTL_EvtAckFlowEnable;

          if (attribute_modified->Attr_Handle == (Handles.WeaponControlChar + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            Notification.eventID = WEAPON_CONTROL_WRITE_NO_RESP_EVT;
            App_Notification(&Notification);
          } 
          else if (attribute_modified->Attr_Handle == (Handles.ActionChar + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            action = Notification.DataTransfered.data[0];
            LS_DeviceLock_Action(action);
          }
          else if (attribute_modified->Attr_Handle == (Handles.PinChar + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            Notification.eventID = PIN_WRITE_NO_RESP_EVT;
            memcpy(&pinCode[0], &attribute_modified->Attr_Data[0], attribute_modified->Attr_Data_Length);
            LS_DeviceLock_HandleAction(&pinCode[0]); 
          }
          else if (attribute_modified->Attr_Handle == (Handles.BiometricControlChar + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            Notification.eventID = BIMOETRIC_CONTROL_WRITE_NO_RESP_EVT;
            App_Notification(&Notification);
          }
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :

          read_req = (aci_gatt_read_permit_req_event_rp0*)blecore_evt->data;
          return_value = SVCCTL_EvtAckFlowEnable;
          aci_gatt_allow_read(read_req->Connection_Handle);
          if (read_req->Attribute_Handle == (Handles.BatteryStatusChar + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            uint8_t batVal = LS_Battery_GetBatteryVoltage();
            App_Update_Char(BATTERY_STATUS, &batVal);

          } 
          else if (read_req->Attribute_Handle == (Handles.WeaponStatusChar + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
        	motor_states_t isLocked = LS_Motor_GetState();
            App_Update_Char(WEAPON_STATUS, &isLocked);
          } 

    		  else if (read_req->Attribute_Handle == (Handles.StateChar + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
    		  {
    		      lockState = LS_DeviceLock_CheckState();
    		      App_Update_Char(STATE, (uint8_t*)&lockState);
    			} 

          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:

          write_perm_req = (aci_gatt_write_permit_req_event_rp0*)blecore_evt->data;
          if (write_perm_req->Attribute_Handle == (Handles.WeaponControlChar + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            

          } 


          break;

    case ACI_GATT_NOTIFICATION_COMPLETE_VSEVT_CODE:
        {

          notification_complete = (aci_gatt_notification_complete_event_rp0*)blecore_evt->data;
          Notification.eventID = NOTIFICATION_COMPLETE_EVT;
          Notification.AttrHandle = notification_complete->Attr_Handle;
          App_Notification(&Notification);

          break;
        }


        default:

          break;
      }

      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */



    default:

      break;
  }

  /* USER CODE BEGIN Event_Handler_2 */

  /* USER CODE END Event_Handler_2 */

  return(return_value);
}/* end Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{

  Char_UUID_t  uuid;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t max_attr_record;

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_1 */

  /* USER CODE END SVCCTL_InitCustomSvc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Event_Handler);

  /**
   *          LS_Service
   *
   * Max_Attribute_Records = 1 + 2*4 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for LS_Service +
   *                                2 for Battery_Status +
   *                                2 for Weapon_Status +
   *                                2 for Weapon_Control +
   *                              = 9
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 9;

  COPY_FIREARM_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(Handles.FirearmService));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: LS_Service, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: LS_Service \n\r");
  }

  /**
   *  Battery_Status
   */
  COPY_BATTERY_STATUS_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(Handles.FirearmService,
                          UUID_TYPE_128, &uuid,
                          _BatStatusSize,
                          CHAR_PROP_READ ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(Handles.BatteryStatusChar));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : BATTERY_STATUS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : BATTERY_STATUS \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char1 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char1 */
  /**
   *  Weapon_Status
   */
  COPY_WEAPON_STATUS_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(Handles.FirearmService,
                          UUID_TYPE_128, &uuid,
                          _WeaponStatusSize,
                          CHAR_PROP_READ ,
                          ATTR_PERMISSION_NONE,
						              GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(Handles.WeaponStatusChar));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : WEAPON_Status, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : WEAPON_Status \n\r");
  }

  /**
   *  Weapon_Control
   */
  COPY_WEAPON_CONTROL_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(Handles.FirearmService,
                          UUID_TYPE_128, &uuid,
                          _WeaponControlSize,
                          CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE ,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(Handles.WeaponControlChar));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : WEAPON_CONTROL, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : WEAPON_CONTROL \n\r");
  }

  max_attr_record = 9;
  COPY_DEVICELOCK_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(Handles.DeviceLockService));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: DeviceLock, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: DeviceLock \n\r");
  }

  /**
   *  Action
   */
  COPY_ACTION_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(Handles.DeviceLockService,
                          UUID_TYPE_128, &uuid,
                          _ActionSize,
                          CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(Handles.ActionChar));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : ACTION, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : ACTION \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char1 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char1 */
  /**
   *  Pin
   */
  COPY_PIN_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(Handles.DeviceLockService,
                          UUID_TYPE_128, &uuid,
                          _PinSize,
                          CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(Handles.PinChar));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : PIN, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : PIN \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char2 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char2 */
  /**
   *  State
   */
  COPY_STATE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(Handles.DeviceLockService,
                          UUID_TYPE_128, &uuid,
                          _StateSize,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(Handles.StateChar));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : STATE, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : STATE \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char3 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char3 */
  /**
   *  Pin_Status
   */
  COPY_PIN_STATUS_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(Handles.DeviceLockService,
                          UUID_TYPE_128, &uuid,
                          _PinStatusSize,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(Handles.PinStatusChar));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : PIN_STATUS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : PIN_STATUS \n\r");
  }

  max_attr_record = 9;

  COPY_BIOMETRIC_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(Handles.BiometricService));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: LS_Service, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: LS_Service \n\r");
  }


  /**
   *  Biometric_Status
   */
  COPY_BIOMETRIC_STATUS_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(Handles.BiometricService,
                          UUID_TYPE_128, &uuid,
                          _BiometricStatusSize,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(Handles.BiometricErrorChar));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : BIOMETRIC_STATUS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : BIOMETRIC_STATUS \n\r");
  }
  /**
   *  Biometric_Control
   */
  COPY_BIOMETRIC_CONTROL_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(Handles.BiometricService,
                          UUID_TYPE_128, &uuid,
                          _BiometricControlSize,
              CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(Handles.BiometricControlChar));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : BIOMETRIC_CONTROL, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : BIOMETRIC_CONTROL \n\r");
  }


  return;
}

/**
 * @brief  Characteristic update
 * @param  id: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus App_Update_Char(BLE_CharID_t id, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN App_Update_Char_1 */

  /* USER CODE END App_Update_Char_1 */

  switch (id)
  {

    case BATTERY_STATUS:
      ret = aci_gatt_update_char_value(Handles.FirearmService,
                                       Handles.BatteryStatusChar,
                                       0, /* charValOffset */
                                       _BatStatusSize, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BATTERY_STATUS command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BATTERY_STATUS command\n\r");
      }
      /* USER CODE BEGIN App_Update_Service_1_Char_1*/

      /* USER CODE END App_Update_Service_1_Char_1*/
      break;

    case WEAPON_STATUS:
      ret = aci_gatt_update_char_value(Handles.FirearmService,
                                       Handles.WeaponStatusChar,
                                       0, /* charValOffset */
									   _WeaponStatusSize, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value WEAPON_STATUS command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value WEAPON_STATUS command\n\r");
      }
      /* USER CODE BEGIN App_Update_Service_1_Char_1*/

      /* USER CODE END App_Update_Service_1_Char_1*/
      break;

    case WEAPON_CONTROL:
      ret = aci_gatt_update_char_value(Handles.FirearmService,
                                       Handles.WeaponStatusChar,
                                       0, /* charValOffset */
                                       _WeaponControlSize, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value WEAPON_CONTROL command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value WEAPON_CONTROL command\n\r");
      }
      /* USER CODE BEGIN App_Update_Service_1_Char_2*/

      /* USER CODE END App_Update_Service_1_Char_2*/
      break;

    case BIOMETRIC_STATUS:
      ret = aci_gatt_update_char_value(Handles.FirearmService,
                                       Handles.BiometricErrorChar,
                                       0, /* charValOffset */
                                       _BiometricStatusSize, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BIOMETRIC_STATUS command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BIOMETRIC_STATUS command\n\r");
      }
      /* USER CODE BEGIN App_Update_Service_1_Char_2*/

      /* USER CODE END App_Update_Service_1_Char_2*/
      break;
    case STATE:
      ret = aci_gatt_update_char_value(Handles.DeviceLockService,
                                       Handles.StateChar,
                                       0, /* charValOffset */
									   _StateSize, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value STATE command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value STATE command\n\r");
      }
      /* USER CODE BEGIN App_Update_Service_1_Char_2*/

      /* USER CODE END App_Update_Service_1_Char_2*/
      break;


    default:
      break;
  }

  /* USER CODE BEGIN App_Update_Char_2 */

  /* USER CODE END App_Update_Char_2 */

  return ret;
}

/**
 * @brief  Characteristic update
 * @param  id: Characteristic identifier
 * @param  pPayload: Characteristic value
 * @param  size: Length of the characteristic value in octets
 *
 */
tBleStatus App_Update_Char_Variable_Length(BLE_CharID_t id, uint8_t *pPayload, uint8_t size)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN App_Update_Char_Variable_Length_1 */

  /* USER CODE END App_Update_Char_Variable_Length_1 */

  switch (id)
  {

    case BATTERY_STATUS:
      ret = aci_gatt_update_char_value(Handles.FirearmService,
                                       Handles.BatteryStatusChar,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BATTERY_STATUS command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BATTERY_STATUS command\n\r");
      }
      /* USER CODE BEGIN App_Update_Char_Variable_Length_Service_1_Char_1*/

      /* USER CODE END App_Update_Char_Variable_Length_Service_1_Char_1*/
      break;

    case WEAPON_CONTROL:
      ret = aci_gatt_update_char_value(Handles.FirearmService,
                                       Handles.WeaponControlChar,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value WEAPON_CONTROL command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value WEAPON_CONTROL command\n\r");
      }
      /* USER CODE BEGIN App_Update_Char_Variable_Length_Service_1_Char_2*/

      /* USER CODE END App_Update_Char_Variable_Length_Service_1_Char_2*/
      break;
    default:
      break;
  }

  /* USER CODE BEGIN App_Update_Char_Variable_Length_2 */

  /* USER CODE END App_Update_Char_Variable_Length_2 */

  return ret;
}

/**
 * @brief  Characteristic update
 * @param  Connection_Handle
 * @param  id: Characteristic identifier
 * @param  pPayload: Characteristic value
 *
 */
tBleStatus App_Update_Char_Ext(uint16_t Connection_Handle, BLE_CharID_t id, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN App_Update_Char_Ext_1 */

  /* USER CODE END App_Update_Char_Ext_1 */

  switch (id)
  {

    case BATTERY_STATUS:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_1*/

      /* USER CODE END Updated_Length_Service_1_Char_1*/
    Generic_STM_App_Update_Char_Ext(Connection_Handle, Handles.FirearmService, Handles.BatteryStatusChar, _BatStatusSize, pPayload);

      break;

    case WEAPON_CONTROL:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_2*/

      /* USER CODE END Updated_Length_Service_1_Char_2*/
    Generic_STM_App_Update_Char_Ext(Connection_Handle, Handles.FirearmService, Handles.WeaponControlChar, _WeaponControlSize, pPayload);

      break;

    default:
      break;
  }

  /* USER CODE BEGIN App_Update_Char_Ext_2 */

  /* USER CODE END App_Update_Char_Ext_2 */

  return ret;
}

static tBleStatus Generic_STM_App_Update_Char_Ext(uint16_t ConnectionHandle, uint16_t ServiceHandle, uint16_t CharHandle, uint16_t CharValueLen, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                       ServiceHandle,
                                       CharHandle,
                                       0, /* update type:0 do not notify, 1 notify, 2 indicate */
                                       CharValueLen, /* charValueLen */
                                       0, /* value offset */
                                       243, /* value length */
                                       (uint8_t *)  pPayload);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 1, result : 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 1\n\r");
  }
  /* USER CODE BEGIN App_Update_Char_Ext_Service_1_Char_1*/

  if (CharValueLen-243<=243)
  {
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         1, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243, /* value offset */
                                         CharValueLen-243, /* value length */
                                         (uint8_t *)  ((pPayload)+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 2, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 2\n\r");
    }
  }
  else
  {
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         0, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243, /* value offset */
                                         243, /* value length */
                                         (uint8_t *)  ((pPayload)+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 3, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 3\n\r");
    }
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         1, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243+243, /* value offset */
                                         CharValueLen-243-243, /* value length */
                                         (uint8_t *)  ((pPayload)+243+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 4, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 4\n\r");
    }
  }
  return ret;
}

