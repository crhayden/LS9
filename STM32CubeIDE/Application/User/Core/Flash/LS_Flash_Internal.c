
/**
 * @file LS_Flash_Internal.c
 * @brief Manages internal flash operations
 *
 * @date 9/8/2024
 *
 **/
#include <LS_Flash_Internal.h>
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Constants
///
////////////////////////////////////////////////////////////////////////////////
//#ifdef DUALCORE_FLASH_SHARING
#define HSEM_PROCESS_1 12U /* Number taken randomly to identify the process locking a semaphore in the driver context */
//#endif
#define PWR_FLAG_WUF PWR_FLAG_WUF3
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Macros
///
////////////////////////////////////////////////////////////////////////////////
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
__IO uint32_t ErasingOnGoing = 0;
__IO uint32_t ErasingUpdate = 0;
uint32_t a_VarDataTab[NB_OF_VARIABLES] = {0};
static EE_Status ee_status = EE_OK;
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Functions
///
////////////////////////////////////////////////////////////////////////////////
static void PVD_Config(void)
{
  PWR_PVDTypeDef sConfigPVD;
  sConfigPVD.PVDLevel = PWR_PVDLEVEL_6;
  sConfigPVD.Mode     = PWR_PVD_MODE_IT_RISING;
  if (HAL_PWR_ConfigPVD(&sConfigPVD) != HAL_OK) {Error_Handler();}

  /* Enable PVD */
  HAL_PWR_EnablePVD();

  /* Enable and set PVD Interrupt priority */
  HAL_NVIC_SetPriority(PVD_PVM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);
}
////////////////////////////////////////////////////////////////////////////////
///
///                           External Functions
///
////////////////////////////////////////////////////////////////////////////////
void LS_FS_WriteU32(uint32_t addr, uint32_t val) {  
  
  /* If an erasing pages operation has just finished, we set OFF the Erase Activity mechanism seen by CPU2 */
  if (ErasingUpdate == 1)
  {
    SHCI_C2_FLASH_EraseActivity(ERASE_ACTIVITY_OFF);
    ErasingUpdate = 0;
  }
  /* Wait any cleanup is completed before accessing flash again */
  if (ErasingOnGoing == 1)
  { 
    return;
  }
  /*  Wait for the flash semaphore to be free */    
  if( HAL_HSEM_IsSemTaken(CFG_HW_FLASH_SEMID) ) return;
  if(HAL_HSEM_Take(CFG_HW_FLASH_SEMID, HSEM_PROCESS_1) != HAL_OK) return;
  
  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();

#ifdef DUALCORE_FLASH_SHARING
    ee_status = EE_WriteVariable32bits(addr, val);
    /* If flash is used by CPU2 */
    if(ee_status == EE_FLASH_USED){
      /* Lock the Flash Program Erase controller and release the semaphore */
      HAL_FLASH_Lock();
      HAL_HSEM_Release(CFG_HW_FLASH_SEMID, HSEM_PROCESS_1);
      /* Give back the control to sequencer */
      return;
    }    
#else
      ee_status = EE_WriteVariable32bits(addr, val);
#endif
  /* Lock the Flash Program Erase controller and release flash semaphore if needed */
  HAL_FLASH_Lock();
  if(ErasingOnGoing != 1) HAL_HSEM_Release(CFG_HW_FLASH_SEMID, HSEM_PROCESS_1);


}



EE_Status LS_FS_ReadU32(uint32_t addr, uint32_t *pVal){
  
  /* If an erasing pages operation has just finished, we set OFF the Erase Activity mechanism seen by CPU2 */
  if (ErasingUpdate == 1)
  {
    SHCI_C2_FLASH_EraseActivity(ERASE_ACTIVITY_OFF);
    ErasingUpdate = 0;
  }
  
  /* Wait any cleanup is completed before accessing flash again */
  if (ErasingOnGoing == 1)
  { 
    return ee_status;
  }
    
  /*  Wait for the flash semaphore to be free */    
  if( HAL_HSEM_IsSemTaken(CFG_HW_FLASH_SEMID) ) return ee_status;
  if(HAL_HSEM_Take(CFG_HW_FLASH_SEMID, HSEM_PROCESS_1) != HAL_OK) return ee_status;
  
  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();
      
  ee_status|= EE_ReadVariable32bits(addr, pVal);
  
  if (ee_status == EE_OK || ee_status == EE_NO_DATA) {
  } else {
	  Error_Handler();
  }
  /* Lock the Flash Program Erase controller and release flash semaphore if needed */
  HAL_FLASH_Lock();
  if(ErasingOnGoing != 1) HAL_HSEM_Release(CFG_HW_FLASH_SEMID, HSEM_PROCESS_1);
  return ee_status;
  
}


void LS_Flash_Init() {

  EE_Status ee_status = EE_OK;

  /* Enable and set FLASH Interrupt priority */
  /* FLASH interrupt is used for the purpose of pages clean up under interrupt */
  HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FLASH_IRQn);

  /* Clear OPTVERR bit */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPTVERR) != RESET) ;

  /* Configure Programmable Voltage Detector (PVD) (optional) */
  /* PVD interrupt is used to suspend the current application flow in case
     a power-down is detected, allowing the flash interface to finish any
     ongoing operation before a reset is triggered. */
  PVD_Config();

  /* When a direct writing is requested - that is to say no page transfer
     is required to achieve the write - the call of EE_WriteVariableXXbits
     function can return the EE_FLASH_USED value meaning that the flash is currently
     used by CPU2 (semaphore 7 locked). When this is the case, the driver
     also activates the interrupt associated to the release of the semaphore
     (flash not used by CPU2 anymore).
     Then, to benefit from this possibility HSEM_IRQn interrupts are configured.
     HAL_HSEM_FreeCallback is called when the semaphore is released. */
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_NVIC_SetPriority(HSEM_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(HSEM_IRQn);

  /*  Wait for the flash semaphore to be free and take it */
  while(HAL_HSEM_Take(CFG_HW_FLASH_SEMID, HSEM_PROCESS_1) != HAL_OK)
  {
    while( HAL_HSEM_IsSemTaken(CFG_HW_FLASH_SEMID) ) ;
  }

  /* Unlock the Flash Program Erase controller for intialization */
  HAL_FLASH_Unlock();

  /* Set EEPROM emulation firmware to erase all potentially incompletely erased
   pages if the system came from an asynchronous reset. Conditional erase is
   safe to use if all Flash operations where completed before the system reset */
  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET)
  {

    /* System reset comes from a power-on reset: Forced Erase */
    /* Initialize EEPROM emulation driver (mandatory) */
    ee_status = EE_Init(EE_FORCED_ERASE);
    if(ee_status != EE_OK) {Error_Handler();}
  }
  else
  {
    /* Clear the Standby flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    /* Check and Clear the Wakeup flag */
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF) != RESET)
    {
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);
    }

    /* Blink LED_OK (Green) upon wakeup */

    /* System reset comes from a STANDBY wakeup: Conditional Erase*/
    /* Initialize EEPROM emulation driver (mandatory) */
    ee_status = EE_Init(EE_CONDITIONAL_ERASE);
    if(ee_status != EE_OK) {Error_Handler();}
  }

  /* Lock the Flash Program Erase controller and release flash semaphore */
  HAL_FLASH_Lock();
  HAL_HSEM_Release(CFG_HW_FLASH_SEMID, HSEM_PROCESS_1);
}
////////////////////////////////////////////////////////////////////////////////
///
///                              Global Data
///
////////////////////////////////////////////////////////////////////////////////


