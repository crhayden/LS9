/*
 * LS_KeyPad.h
 * @brief Manages keypad communication
 *
 *  Created on: Aug 24, 2024
 *      Author: chris
 */
#ifndef SRC_DEVICES_LS_KEYPAD_H_
#define SRC_DEVICES_LS_KEYPAD_H_
#include "main.h"
#include "stdbool.h"
#include <LS_Flash_Internal.h>
#include <LS_Motor_DRV8823.h>
#include <LS_DeviceLock.h>
#include <stdio.h>
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
/// Initializes the keypad subsystem
///
/// @return     void
///
void LS_KeyPad_Init();
#endif /* SRC_DEVICES_LS_KEYPAD_H_ */
