/**
 * @file LS_Battery.h
 * @brief Polls battery voltage
 *
 * @date 9/13/2024
 *
 **/
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
#define NUM_BAT_SAMPLES 		10
#define BATTERY_TASK_SLEEP_TIME	100
#define HALL_LOW_THRESHOLD		0.25f
#define HALL_HIGH_THRESHOLD		1.5f
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
float		batVoltage 	= 0;
float		hallVoltage = 0;
uint8_t 	batPercent 	= 0;
bool 		shouldMeasureBattery = true;
bool 		isLocked 	= false;
osThreadId_t batteryTaskHandle;
const osThreadAttr_t batteryTask_attributes = {
  .name = "batteryTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Functions
///
////////////////////////////////////////////////////////////////////////////////

static void SetMux(input_device_t dev) {
	if (dev == HALL) {
		HAL_GPIO_WritePin(GPIOA, MUX_CONTROL_Pin, GPIO_PIN_SET);
	} else if (dev == BATTERY) {
		HAL_GPIO_WritePin(GPIOA, MUX_CONTROL_Pin, GPIO_PIN_RESET);
	}
}
///
/// @brief  Function implementing the battery task.
///
/// @param  argument: Hardcoded to 0.
///
/// @return void
///
static void BatteryTask(void * argument) {

    for (;;) {
    	//
    	// Alternate measuring between battery voltage and hall effect
    	//
    	if(shouldMeasureBattery) {
    		SetMux(BATTERY);
    	}
    	else {
    		SetMux(HALL);
    	}
		HAL_ADC_Start_IT(&hadc1);
        osDelay(BATTERY_TASK_SLEEP_TIME);
    }
}
//------------------------------------------------------------------------------
//
//                           Callback Handlers
//
//------------------------------------------------------------------------------

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
	//
	// Don't make updated when we are in stealth mode
	//
	const uint32_t maxADCVal = 4095;
	float adcVoltage		= 0;
	//
	// Take x samples of the ADC and average
	//
	for (int i = 0; i<=NUM_BAT_SAMPLES; i++) {
		uint32_t adcVal = HAL_ADC_GetValue (hadc1);
		adcVoltage += (((float)adcVal/(float)maxADCVal) * 3.3);
	}
	adcVoltage = adcVoltage/NUM_BAT_SAMPLES;
	//
	// process for each input type
	//
	if(shouldMeasureBattery) {
		batPercent = (adcVoltage/3.3) * 100;
		if (batPercent > 100) {
			batPercent = 100;
		}
		shouldMeasureBattery = false;
	}
	else
	{
		if (adcVoltage > HALL_HIGH_THRESHOLD) {
			isLocked = false;
		} else if (adcVoltage < HALL_LOW_THRESHOLD) {
			isLocked = true;
		}
		shouldMeasureBattery = true;
	}
}
////////////////////////////////////////////////////////////////////////////////
///
///                           External Functions
///
////////////////////////////////////////////////////////////////////////////////
uint8_t LS_Battery_GetBatteryVoltage() {
	return batPercent;
}
bool LS_Battery_isTrigLocked() {
	return isLocked;
}
void LS_Battery_Init() {
	//
	// Register for the DMA complete callback
	//
  	HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_CONVERSION_COMPLETE_CB_ID, HAL_ADC_ConvCpltCallback);
	//
	// Create the battery task
	//
	batteryTaskHandle = osThreadNew(BatteryTask, NULL, &batteryTask_attributes);

}
////////////////////////////////////////////////////////////////////////////////
///
///                              Global Data
///
////////////////////////////////////////////////////////////////////////////////

