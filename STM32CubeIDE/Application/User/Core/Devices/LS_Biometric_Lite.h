/**
 * @file LS_Biometric_Lite.h
 * @brief Manages communication to/from host & biometric scanner
 *
 * @date 4/30/2024
 *
 **/
#ifndef SRC_DEVICES_LS_BIOMETRIC_LITE_H_
#define SRC_DEVICES_LS_BIOMETRIC_LITE_H_
#include "main.h"
#include "fpc_bep_types.h"
#include "custom_stm.h"
////////////////////////////////////////////////////////////////////////////////
///
///                              External Data
///
///////////////////////////////////////////////////////////////////////////////
extern osMessageQueueId_t biometricQueueHandle;
extern osMutexId_t biometricOpMutexID;
////////////////////////////////////////////////////////////////////////////////
///
///                              External Types
///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///
///                              External Functions
///
///////////////////////////////////////////////////////////////////////////////
fpc_bep_result_t LS_BM_Lite_Capture();
bool LS_BM_Lite_Identify();
void LS_BM_Lite_Wait_For_Finger_Present();
void stm_uart_init(uint32_t baudrate);
///
/// Initializes the biometric subsystem
///
/// @return     void
///
void LS_BM_Lite_Init();
#endif /* SRC_DEVICES_LS_BIOMETRIC_LITE_H_ */
