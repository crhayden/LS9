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
#define HALL_LOW_THRESHOLD		1.55f
#define HALL_HIGH_THRESHOLD		2.21f
////////////////////////////////////////////////////////////////////////////////
////
///
///                           Internal Types
///
////////////////////////////////////////////////////////////////////////////////
typedef struct {
	float maxUnlockVal;
	float minlockVal;
	uint32_t num_samples;
} motor_test_data_t;
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Data
///
////////////////////////////////////////////////////////////////////////////////
motor_test_data_t testData = {.maxUnlockVal = 0, .minlockVal = 100, .num_samples = 0};
///
/// Initial motor state
///
motor_states_t m_motorState = s_LOCKED;
osMutexId_t motorStateMutexID;
///
/// Motor queues & task handle/attributes
///
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
///
/// Sets the motor state
///
static void _setMotorState(motor_states_t motorState) {
	osMutexAcquire(motorStateMutexID, osWaitForever);
	m_motorState = motorState;
	osMutexRelease(motorStateMutexID);
}
static motor_states_t _getMotorState() {
	return m_motorState;
}
///
/// @brief  Function to select mux inputs.
///
/// @param  argument: input_device_t.
///
/// @return void
///
static void _setMux(input_device_t dev) {
	if (dev == HALL) {
		HAL_GPIO_WritePin(GPIOA, MUX_CONTROL_Pin, GPIO_PIN_SET);
	} else if (dev == BATTERY) {
		HAL_GPIO_WritePin(GPIOA, MUX_CONTROL_Pin, GPIO_PIN_RESET);
	}
}
///
/// @brief  Function implementing control of the motor.
///
/// @param  argument: motor_action_t.
///
/// @return void
///
static void _motorPosition(motor_action_t action) {
	if (action == a_LOCK) {
		HAL_GPIO_WritePin(GPIOA, MOT_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, MOT_IN2_Pin, GPIO_PIN_RESET);
	} else if (action == a_UNLOCK) {
		HAL_GPIO_WritePin(GPIOA, MOT_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, MOT_IN2_Pin, GPIO_PIN_SET);
	} else if (action == a_BRAKE){
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
	_setMux(HALL);
	for (;;) {
		if (osMessageQueueGet(motorQueueHandle, &evt, 0, osWaitForever) ==  osOK){
			switch (evt.type) {
				case MOTOR_LOCK:
					if (m_motorState != s_LOCKED){
						HAL_ADC_Start_IT(&hadc1);
						_motorPosition(a_LOCK);
					}
					break;
				case MOTOR_UNLOCK:
					if (m_motorState != s_UNLOCKED){
						HAL_ADC_Start_IT(&hadc1);
						_motorPosition(a_UNLOCK);
					}
					break;
				case MOTOR_IS_LOCK:
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
	testData.num_samples++;
	const uint32_t maxADCVal 	= 4095;
	float adcVoltage			= 0;
	uint32_t adcVal = HAL_ADC_GetValue (hadc1);
	adcVoltage = (((float)adcVal/(float)maxADCVal) * 3.3);
	if (adcVoltage > testData.maxUnlockVal) {
		testData.maxUnlockVal = adcVoltage;
	}
	if (adcVoltage < testData.minlockVal) {
		testData.minlockVal = adcVoltage;
	}
	if (_getMotorState() == s_LOCKED && adcVoltage > HALL_HIGH_THRESHOLD) {
		_motorPosition(a_BRAKE);
		_setMotorState(s_UNLOCKED);
		HAL_ADC_Stop_IT(hadc1);
	} else if (_getMotorState() == s_UNLOCKED && adcVoltage < HALL_LOW_THRESHOLD) {
		_motorPosition(a_BRAKE);
		_setMotorState(s_LOCKED);
		HAL_ADC_Stop_IT(hadc1);
	}
}
////////////////////////////////////////////////////////////////////////////////
///
///                           External Functions
///
////////////////////////////////////////////////////////////////////////////////
motor_states_t LS_Motor_GetState() {
	return _getMotorState();
}
void LS_Motor_DRV8823_Init() {
	motorStateMutexID = osMutexNew(NULL);
	//
	// Register for the DMA complete callback
	//
	HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_CONVERSION_COMPLETE_CB_ID, HAL_ADC_ConvCpltCallback);
	//
	// Create motor queue & task
	//
	motorQueueHandle = osMessageQueueNew (16, sizeof(app_message_t), &motorQueue_attributes);
	motorTaskHandle = osThreadNew(MotorTask, NULL, &motorTask_attributes);
}
////////////////////////////////////////////////////////////////////////////////
///
///                              Global Data
///
////////////////////////////////////////////////////////////////////////////////


