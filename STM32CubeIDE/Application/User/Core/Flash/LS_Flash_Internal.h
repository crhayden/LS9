/**
 * @file LS_Flash_Internal.c
 * @brief Manages internal flash operations
 *
 * @date 9/8/2024
 *
 **/
#ifndef FLASH_LS_FLASH_INTERNAL_H_
#define FLASH_LS_FLASH_INTERNAL_H_
#include "main.h"

#include "shci.h"
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
void LS_FS_WriteU32(uint32_t addr, uint32_t val);
void LS_FS_ReadU32(uint32_t addr, uint32_t *pVal);
///
/// Initializes the biometric subsystem
///
/// @return     void
///
void LS_Flash_Init();
#endif /* FLASH_LS_FLASH_INTERNAL_H_ */
