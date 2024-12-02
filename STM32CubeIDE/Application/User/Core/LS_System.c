/**
 * @file LS_System.h
 * @brief Polls system voltage
 *
 * @date 9/13/2024
 *
 **/
#include "LS_System.h"
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Constants
///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Macros
///
////////////////////////////////////////////////////////////////////////////////
#define SYSTEM_TASK_SLEEP_TIME	200
////////////////////////////////////////////////////////////////////////////////
////
///
///                           Internal Types
///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Data
///
////////////////////////////////////////////////////////////////////////////////

osThreadId_t systemTaskHandle;
const osThreadAttr_t systemTask_attributes = {
  .name = "systemTask",
  .priority = (osPriority_t) osPriorityLow7,
  .stack_size = 128 * 8
};
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Functions
///
////////////////////////////////////////////////////////////////////////////////
///
/// @brief  Function implementing the system task.
///
/// @param  argument: Hardcoded to 0.
///
/// @return void
///
static void SystemTask(void * argument) {
    //uint16_t template_id;
    bool match = false;
    motor_states_t state = s_UNKNOWN;
	app_message_t		evt;
  	osStatus_t			ret;
  	//fpc_bep_result_t	res;
    for (;;) {
    	if(!isBioOperationInProgress) {
//			LS_BM_Lite_Wait_For_Finger_Present();
//			match = LS_BM_Lite_Identify();
			if (match) {
				state = LS_Motor_GetState();
				if (state == s_LOCKED) {
					evt.type      = MOTOR_UNLOCK;
					ret           = osMessageQueuePut(motorQueueHandle, &evt, 0, 0);
					if (ret != osOK) {
						Error_Handler();
					}
				}
				//
				// for testing only
				//
				else if (state == s_UNLOCKED) {
					evt.type      = MOTOR_LOCK;
					ret           = osMessageQueuePut(motorQueueHandle, &evt, 0, 0);
					if (ret != osOK) {
						Error_Handler();
					}
				}
			}
//			else {
//				osDelay(SYSTEM_TASK_SLEEP_TIME*5);
//			}
    	}
        osDelay(SYSTEM_TASK_SLEEP_TIME);
    }
}
//------------------------------------------------------------------------------
//
//                           Callback Handlers
//
//------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////
///
///                           External Functions
///
///////////////////////////////////////////////////////////////////////////////
void LS_System_Init() {
	//
	// Create the system task
	//
	systemTaskHandle = osThreadNew(SystemTask, NULL, &systemTask_attributes);

}
////////////////////////////////////////////////////////////////////////////////
///
///                              Global Data
///
////////////////////////////////////////////////////////////////////////////////


