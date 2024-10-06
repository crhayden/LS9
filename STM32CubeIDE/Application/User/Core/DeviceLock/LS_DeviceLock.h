/**
 * @file LS_DeviceLock.h
 * @brief Device locking API
 *
 * @date 10/2/2024
 *
 **/
#ifndef APPLICATION_USER_CORE_DEVICELOCK_LS_DEVICELOCK_H_
#define APPLICATION_USER_CORE_DEVICELOCK_LS_DEVICELOCK_H_
#include "main.h"
#include <LS_Flash_Internal.h>
////////////////////////////////////////////////////////////////////////////////
///
///                           External Macros
///
////////////////////////////////////////////////////////////////////////////////   	  
///
/// Pin code is alphanumeric and case sensitive
///
#define LS_PIN_CODE_SIZE        	4       	  
#define DEVLOCK_ACTION_HANDLE       67
#define DEVLOCK_PIN_HANDLE          69
#define DEVLOCK_STATE_HANDLE        71
#define DEVLOCK_PINSTATUS_HANDLE    73
////////////////////////////////////////////////////////////////////////////////
///
///                           External Types
///
////////////////////////////////////////////////////////////////////////////////   
///
/// Device lock states
///
typedef enum _LS_DeviceLock_States {
    LOCKED, 
    UNLOCKED,   
    DISABLED,   
} LS_DeviceLock_States;
///
/// Device lock actions
///
typedef enum _LS_DeviceLock_Actions {
    aLOCK,
    aUNLOCK,
    aDISABLE,
    aCHANGEPIN,
    aNONE,
} LS_DeviceLock_Actions;
////////////////////////////////////////////////////////////////////////////////
///
///                              API Functions
///
////////////////////////////////////////////////////////////////////////////////
///
/// Reset the lock state 
///
/// @return void
///
void LS_DeviceLock_ResetState();
///
/// Get the current lock state
///
/// @return LS_DeviceLock_States
///
LS_DeviceLock_States LS_DeviceLock_CheckState();
///
/// Set the desired action
///
/// @param[in]  action   - the desired action.
///
/// @return void
///
void LS_DeviceLock_Action(LS_DeviceLock_Actions action);
///
/// Handle the desired action
///
/// @param[in]  pUserCode   - user pin code.
///
/// @return void
///
void LS_DeviceLock_HandleAction(uint8_t *pUserCode); 
///
/// Read the current pin status
///
/// @return uint8_t - 2 = success, 1 otherwise 
///
uint8_t LS_DeviceLock_ReadPinSatus();
///
/// Initializes the device lock code
///
/// @return     void
///
void LS_DeviceLock_Init();

#endif /* APPLICATION_USER_CORE_DEVICELOCK_LS_DEVICELOCK_H_ */
