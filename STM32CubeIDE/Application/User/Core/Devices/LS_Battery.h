/**
 * @file LS_Battery.h
 * @brief Polls battery voltage
 *
 * @date 9/13/2024
 *
 **/
#ifndef SRC_DEVICES_LS_BATTERY_H_
#define SRC_DEVICES_LS_BATTERY_H_
#include "main.h"
#include "stdbool.h"
////////////////////////////////////////////////////////////////////////////////
///
///                              External Data
///
////////////////////////////////////////////////////////////////////////////////
typedef enum {
	HALL 	= 1,
	BATTERY,
} input_device_t;
////////////////////////////////////////////////////////////////////////////////
///
///                              External Functions
///
////////////////////////////////////////////////////////////////////////////////
///
/// get the battery voltage as a percentage
///
/// @return     void
///
uint8_t LS_Battery_GetBatteryVoltage();
///
/// get the trigger state
///
/// @return     void
///
bool LS_Battery_isTrigLocked();
///
/// Initializes the Battery moinitoring task
///
/// @return     void
///
void LS_Battery_Init();
#endif /* SRC_DEVICES_LS_BATTERY_H_ */
