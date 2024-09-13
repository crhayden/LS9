/*
 * LS_Motor_DRV8832.c
 * @brief Manages motor control
 *
 *  Created on: Aug 24, 2024
 *      Author: chris
 */

#include <LS_Motor_DRV8823.h>
#include "LS_Battery.h"
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
#define LOCK	1
#define UNLOCK 	2
#define COAST	3
#define BRAKE   4
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
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osMessageQueueId_t motorQueueHandle; 
const osMessageQueueAttr_t motorQueue_attributes = { 
  .name = "motorQueue"
}; 
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Functions
///
////////////////////////////////////////////////////////////////////////////////
static void motorPos(uint8_t motorState) {
	if (motorState == LOCK) {
		HAL_GPIO_WritePin(GPIOA, MOT_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, MOT_IN2_Pin, GPIO_PIN_RESET);
	} else if (motorState == UNLOCK) {
		HAL_GPIO_WritePin(GPIOA, MOT_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MOT_IN2_Pin, GPIO_PIN_SET);
	} else if (motorState == BRAKE){
		HAL_GPIO_WritePin(GPIOA, MOT_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, MOT_IN2_Pin, GPIO_PIN_SET);
	}
}
///
/// @brief  Function implementing the motor task.
///
/// @param  argument: Hardcoded to 0.
///
/// @return void
///
static void MotorTask(void * argument) {
	app_message_t		evt;
	for (;;) {
		if (osMessageQueueGet(motorQueueHandle, &evt, 0, osWaitForever) ==  osOK){
			switch (evt.type) {
				case MOTOR_LOCK:
					//HG_Motors_steer(evt.degrees, evt.motor_id);
					motorPos(LOCK);
					osDelay(135);
					motorPos(BRAKE);
					break;
				case MOTOR_UNLOCK:
					motorPos(UNLOCK);
					osDelay(135);
					motorPos(BRAKE);
					//HG_Motors_steer(evt.degrees, evt.motor_id);
					break;
				case MOTOR_IS_LOCK:
					//HG_Motors_steer(evt.degrees, evt.motor_id);
					break;
				default:
					break;

			}
		}
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
void LS_Motor_DRV8823_Init() {
  	motorQueueHandle = osMessageQueueNew (16, sizeof(app_message_t), &motorQueue_attributes);
	motorTaskHandle = osThreadNew(MotorTask, NULL, &motorTask_attributes);
}
////////////////////////////////////////////////////////////////////////////////
///
///                              Global Data
///
////////////////////////////////////////////////////////////////////////////////


