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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  CustomFirearmHdle;              /**< FireArm handle */

  uint16_t  CustomBattery_StatusHdle;       /**< Battery_Status handle */
  uint16_t  CustomWeapon_StatusHdle;        /**< Weapon_Status handle */
  uint16_t  CustomWeapon_ControlHdle;       /**< Weapon_Control handle */
  uint16_t  CustomBiometric_StatusHdle;     /**< Weapon_Control handle */
  uint16_t  CustomBiometric_ControlHdle;    /**< Weapon_Control handle */

  uint16_t  CustomDevicelockHdle;           /**< DeviceLock handle */

  uint16_t  CustomActionHdle;               /**< Action handle */
  uint16_t  CustomPinHdle;                  /**< Pin handle */
  uint16_t  CustomStateHdle;                /**< State handle */
  uint16_t  CustomPin_StatusHdle;           /**< Pin_Status handle */
/* USER CODE BEGIN Context */
  /* Place holder for Characteristic Descriptors Handle*/

/* USER CODE END Context */
}CustomContext_t;

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
uint16_t SizeBattery_Status = 1;
uint16_t SizeWeapon_Status = 1;
uint16_t SizeWeapon_Control = 1;
uint16_t SizeBiometric_Status = 1;
uint16_t SizeBiometric_Control = 1;
uint16_t SizeAction = 1;
uint16_t SizePin = 4;
uint16_t SizeState = 1;
uint16_t SizePin_Status = 1;

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
static CustomContext_t CustomContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

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
#define COPY_BIOMETRIC_STATUS_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x4C,0x4F,0x44,0x49,0x53,0x54,0x41,0x52,0x46,0x41,0x53,0x45,0x52,0x56,0x00,0x00)
#define COPY_BIOMETRIC_CONTROL_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x4C,0x4F,0x44,0x4A,0x53,0x54,0x41,0x52,0x46,0x41,0x53,0x45,0x52,0x56,0x00,0x00)

#define COPY_DEVICELOCK_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x44,0x45,0x46,0x49,0x43,0x45,0x4C,0x4F,0x43,0x4B,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_ACTION_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x44,0x45,0x46,0x4A,0x43,0x45,0x4C,0x4F,0x43,0x4B,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_PIN_UUID(uuid_struct)              COPY_UUID_128(uuid_struct,0x44,0x45,0x46,0x4B,0x43,0x45,0x4C,0x4F,0x43,0x4B,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_STATE_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x44,0x45,0x46,0x4C,0x43,0x45,0x4C,0x4F,0x43,0x4B,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_PIN_STATUS_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x44,0x45,0x46,0x4D,0x43,0x45,0x4C,0x4F,0x43,0x4B,0x00,0x00,0x00,0x00,0x00,0x00)
/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  aci_gatt_write_permit_req_event_rp0   *write_perm_req;
  aci_gatt_read_permit_req_event_rp0    *read_req;
  aci_gatt_notification_complete_event_rp0    *notification_complete;
  Custom_STM_App_Notification_evt_t     Notification;
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

          if (attribute_modified->Attr_Handle == (CustomContext.CustomWeapon_ControlHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            Notification.Custom_Evt_Opcode = CUSTOM_STM_WEAPON_CONTROL_WRITE_NO_RESP_EVT;
            Custom_STM_App_Notification(&Notification);
          } 
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomActionHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            action = Notification.DataTransfered.data[0];
            LS_DeviceLock_Action(action);
          }
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomPinHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            Notification.Custom_Evt_Opcode = CUSTOM_STM_PIN_WRITE_NO_RESP_EVT;
            memcpy(&pinCode[0], &attribute_modified->Attr_Data[0], attribute_modified->Attr_Data_Length);
            LS_DeviceLock_HandleAction(&pinCode[0]); 
          }
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :

          read_req = (aci_gatt_read_permit_req_event_rp0*)blecore_evt->data;
          return_value = SVCCTL_EvtAckFlowEnable;
          aci_gatt_allow_read(read_req->Connection_Handle);
          if (read_req->Attribute_Handle == (CustomContext.CustomBattery_StatusHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            uint8_t batVal = LS_Battery_GetBatteryVoltage();
            Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_STATUS, &batVal);

          } 
          else if (read_req->Attribute_Handle == (CustomContext.CustomWeapon_StatusHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
        	motor_states_t isLocked = LS_Motor_GetState();
            Custom_STM_App_Update_Char(CUSTOM_STM_WEAPON_STATUS, &isLocked);
          } 

    		  else if (read_req->Attribute_Handle == (CustomContext.CustomStateHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
    		  {
    		      lockState = LS_DeviceLock_CheckState();
    		      Custom_STM_App_Update_Char(CUSTOM_STM_STATE, (uint8_t*)&lockState);
    			} 

          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:

          write_perm_req = (aci_gatt_write_permit_req_event_rp0*)blecore_evt->data;
          if (write_perm_req->Attribute_Handle == (CustomContext.CustomWeapon_ControlHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            

          } 


          break;

    case ACI_GATT_NOTIFICATION_COMPLETE_VSEVT_CODE:
        {

          notification_complete = (aci_gatt_notification_complete_event_rp0*)blecore_evt->data;
          Notification.Custom_Evt_Opcode = CUSTOM_STM_NOTIFICATION_COMPLETE_EVT;
          Notification.AttrHandle = notification_complete->Attr_Handle;
          Custom_STM_App_Notification(&Notification);

          break;
        }


        default:

          break;
      }

      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */



    default:

      break;
  }

  /* USER CODE BEGIN Custom_STM_Event_Handler_2 */

  /* USER CODE END Custom_STM_Event_Handler_2 */

  return(return_value);
}/* end Custom_STM_Event_Handler */

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
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

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
  max_attr_record = 12;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_FIREARM_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomFirearmHdle));
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
  ret = aci_gatt_add_char(CustomContext.CustomFirearmHdle,
                          UUID_TYPE_128, &uuid,
                          SizeBattery_Status,
                          CHAR_PROP_READ ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomBattery_StatusHdle));
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
  ret = aci_gatt_add_char(CustomContext.CustomFirearmHdle,
                          UUID_TYPE_128, &uuid,
                          SizeWeapon_Status,
                          CHAR_PROP_READ ,
                          ATTR_PERMISSION_NONE,
						              GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomWeapon_StatusHdle));
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
  ret = aci_gatt_add_char(CustomContext.CustomFirearmHdle,
                          UUID_TYPE_128, &uuid,
                          SizeWeapon_Control,
                          CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE ,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomWeapon_ControlHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : WEAPON_CONTROL, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : WEAPON_CONTROL \n\r");
  }
  /**
   *  Biometric_Status
   */
  COPY_BIOMETRIC_STATUS_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomFirearmHdle,
                          UUID_TYPE_128, &uuid,
                          SizeBiometric_Status,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomBiometric_StatusHdle));
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
  ret = aci_gatt_add_char(CustomContext.CustomFirearmHdle,
                          UUID_TYPE_128, &uuid,
                          SizeBiometric_Control,
						  CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomBiometric_ControlHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : BIOMETRIC_CONTROL, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : BIOMETRIC_CONTROL \n\r");
  }


  max_attr_record = 9;
  COPY_DEVICELOCK_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomDevicelockHdle));
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
  ret = aci_gatt_add_char(CustomContext.CustomDevicelockHdle,
                          UUID_TYPE_128, &uuid,
                          SizeAction,
                          CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomActionHdle));
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
  ret = aci_gatt_add_char(CustomContext.CustomDevicelockHdle,
                          UUID_TYPE_128, &uuid,
                          SizePin,
                          CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomPinHdle));
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
  ret = aci_gatt_add_char(CustomContext.CustomDevicelockHdle,
                          UUID_TYPE_128, &uuid,
                          SizeState,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomStateHdle));
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
  ret = aci_gatt_add_char(CustomContext.CustomDevicelockHdle,
                          UUID_TYPE_128, &uuid,
                          SizePin_Status,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomPin_StatusHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : PIN_STATUS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : PIN_STATUS \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char4 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char4 */

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_2 */

  /* USER CODE END SVCCTL_InitCustomSvc_2 */

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_1 */

  /* USER CODE END Custom_STM_App_Update_Char_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_BATTERY_STATUS:
      ret = aci_gatt_update_char_value(CustomContext.CustomFirearmHdle,
                                       CustomContext.CustomBattery_StatusHdle,
                                       0, /* charValOffset */
                                       SizeBattery_Status, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BATTERY_STATUS command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BATTERY_STATUS command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_1*/
      break;

    case CUSTOM_STM_WEAPON_STATUS:
      ret = aci_gatt_update_char_value(CustomContext.CustomFirearmHdle,
                                       CustomContext.CustomWeapon_StatusHdle,
                                       0, /* charValOffset */
									   SizeWeapon_Status, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value WEAPON_STATUS command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value WEAPON_STATUS command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_1*/
      break;

    case CUSTOM_STM_WEAPON_CONTROL:
      ret = aci_gatt_update_char_value(CustomContext.CustomFirearmHdle,
                                       CustomContext.CustomWeapon_StatusHdle,
                                       0, /* charValOffset */
                                       SizeWeapon_Control, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value WEAPON_CONTROL command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value WEAPON_CONTROL command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_2*/
      break;

    case CUSTOM_STM_BIOMETRIC_STATUS:
      ret = aci_gatt_update_char_value(CustomContext.CustomFirearmHdle,
                                       CustomContext.CustomBiometric_StatusHdle,
                                       0, /* charValOffset */
                                       SizeBiometric_Status, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BIOMETRIC_STATUS command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BIOMETRIC_STATUS command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_2*/
      break;
    case CUSTOM_STM_STATE:
      ret = aci_gatt_update_char_value(CustomContext.CustomDevicelockHdle,
                                       CustomContext.CustomStateHdle,
                                       0, /* charValOffset */
									   SizeState, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value STATE command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value STATE command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_2*/
      break;


    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */

  /* USER CODE END Custom_STM_App_Update_Char_2 */

  return ret;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  pPayload: Characteristic value
 * @param  size: Length of the characteristic value in octets
 *
 */
tBleStatus Custom_STM_App_Update_Char_Variable_Length(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload, uint8_t size)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_1 */

  /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_BATTERY_STATUS:
      ret = aci_gatt_update_char_value(CustomContext.CustomFirearmHdle,
                                       CustomContext.CustomBattery_StatusHdle,
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
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_1*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_1*/
      break;

    case CUSTOM_STM_WEAPON_CONTROL:
      ret = aci_gatt_update_char_value(CustomContext.CustomFirearmHdle,
                                       CustomContext.CustomWeapon_ControlHdle,
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
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_2*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_2*/
      break;
    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_2 */

  /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_2 */

  return ret;
}

/**
 * @brief  Characteristic update
 * @param  Connection_Handle
 * @param  CharOpcode: Characteristic identifier
 * @param  pPayload: Characteristic value
 *
 */
tBleStatus Custom_STM_App_Update_Char_Ext(uint16_t Connection_Handle, Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_1 */

  /* USER CODE END Custom_STM_App_Update_Char_Ext_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_BATTERY_STATUS:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_1*/

      /* USER CODE END Updated_Length_Service_1_Char_1*/
    Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomFirearmHdle, CustomContext.CustomBattery_StatusHdle, SizeBattery_Status, pPayload);

      break;

    case CUSTOM_STM_WEAPON_CONTROL:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_2*/

      /* USER CODE END Updated_Length_Service_1_Char_2*/
    Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomFirearmHdle, CustomContext.CustomWeapon_ControlHdle, SizeWeapon_Control, pPayload);

      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_2 */

  /* USER CODE END Custom_STM_App_Update_Char_Ext_2 */

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
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_Service_1_Char_1*/

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

