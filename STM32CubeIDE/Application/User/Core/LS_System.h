/*
 * LS_System.h
 *
 *  Created on: Nov 27, 2024
 *      Author: chris
 */

#ifndef APPLICATION_USER_CORE_LS_SYSTEM_H_
#define APPLICATION_USER_CORE_LS_SYSTEM_H_

#include "main.h"
#include "stdbool.h"
#include <LS_Biometric_Lite.h>
#include <math.h>
#include "bmlite_if.h"
#include "hcp_tiny.h"
#include "platform.h"
#include "bmlite_hal.h"
#include <LS_Motor_DRV8823.h>
////////////////////////////////////////////////////////////////////////////////
///
///                              External Data
///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///
///                              External Functions
///
////////////////////////////////////////////////////////////////////////////////
///
/// Initializes the System task
///
/// @return     void
///
void LS_System_Init();
#endif /* APPLICATION_USER_CORE_LS_SYSTEM_H_ */
