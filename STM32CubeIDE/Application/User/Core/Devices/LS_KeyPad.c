/*
 * LS_KeyPad.c
 * @brief Manages keypad communication
 *
 *  Created on: Aug 24, 2024
 *      Author: chris
 */
#include <LS_KeyPad.h>
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
osThreadId_t keypadTaskHandle;
const osThreadAttr_t keypadTask_attributes = {
  .name = "keypadTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Functions
///
////////////////////////////////////////////////////////////////////////////////
///
/// @brief  Function implementing the keypad task.
///
/// @param  argument: Hardcoded to 0.
///
/// @return void
///
static void StartKeypadTask(void * argument) {

	for (;;) {
		osDelay(500);
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
////////////////////////////////////////////////////////////////////////////////
void LS_KeyPad_Init() {
	  keypadTaskHandle = osThreadNew(StartKeypadTask, NULL, &keypadTask_attributes);
}
////////////////////////////////////////////////////////////////////////////////
///
///                              Global Data
///
////////////////////////////////////////////////////////////////////////////////
