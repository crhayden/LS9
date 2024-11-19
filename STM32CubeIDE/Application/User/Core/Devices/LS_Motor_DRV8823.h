/*
 * LS_Motor_DRV8823.h
 * @brief Manages motor control
 *
 *  Created on: Aug 24, 2024
 *      Author: chris
 */

#ifndef SRC_DEVICES_LS_MOTOR_DRV8823_H_
#define SRC_DEVICES_LS_MOTOR_DRV8823_H_
#include "main.h"
////////////////////////////////////////////////////////////////////////////////
///
///                              External Types
///
////////////////////////////////////////////////////////////////////////////////
///
/// Mux options
///
typedef enum {
	HALL 	= 1,
	BATTERY,
} input_device_t;
///
/// Possible motor actions
///
typedef enum {
	a_LOCK,
	a_UNLOCK,
	a_COAST,
	a_BRAKE,
	a_UNKNOWN,
} motor_action_t;
///
/// Possible motor states
///
typedef enum {
	s_LOCKED,
	s_UNLOCKED,
	s_UNKNOWN,
} motor_states_t;
////////////////////////////////////////////////////////////////////////////////
///
///                              External Data
///
////////////////////////////////////////////////////////////////////////////////
extern osMessageQueueId_t motorQueueHandle; 
////////////////////////////////////////////////////////////////////////////////
///
///                              External Functions
///
////////////////////////////////////////////////////////////////////////////////
///
/// Gets the current motor state
///
/// @return     motor_states_t
///
motor_states_t LS_Motor_GetState();
///
/// Initializes the motor subsystem
///
/// @return     void
///
void LS_Motor_DRV8823_Init();
#endif /* SRC_DEVICES_LS_MOTOR_DRV8823_H_ */
