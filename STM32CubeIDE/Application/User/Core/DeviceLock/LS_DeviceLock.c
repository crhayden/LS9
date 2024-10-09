/**
 * @file LS_DeviceLock.c
 * @brief Device lock API
 *
 * @date 10/2/2024
 *
 **/
#include <LS_DeviceLock.h>
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Macros
///
////////////////////////////////////////////////////////////////////////////////   	  
///
/// Default pin code
///
#define LS_DEFAULT_PIN_CODE                   0xFFFFFFFF
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Constants
///
////////////////////////////////////////////////////////////////////////////////  
///
/// virtual address for pin code
///
static const uint16_t _pinCodeAddr   =  0x0001;
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Types
///
////////////////////////////////////////////////////////////////////////////////
///
/// Device lock current state
///
LS_DeviceLock_States lockState = DISABLED; 
///
/// Device lock current pin status
///
uint8_t pinStatus = 1;
///
/// Next intended action
///
LS_DeviceLock_Actions currentAction = aNONE; 
///
/// Device lock error codes
///
typedef enum _LS_DeviceLock_err {
    NONE,           ///< No error
    INVALID_PARAM,  ///< Invalid parameter
    IS_DEFAULT,     ///< Pin is the default value
    IS_MATCH,       ///< Pin code is a match
    IS_NOTMATCH,    ///< Pin code is not a match
    FILESYSTEM,     ///< Error reported by filesystem
} LS_DeviceLock_err;
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Data
///
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Functions
///
////////////////////////////////////////////////////////////////////////////////
///
/// Convert 4 byte array to uint_32
///
/// @param[in]  pData8  pointer to 4 byte value
///
/// @return     data32  32bit value
///
static uint32_t _Convert_UInt8_To_UInt32(uint8_t *pData8) {
    uint32_t    data32  = pData8[0];
                data32 |= pData8[1] << 8;
                data32 |= pData8[2] << 16;
                data32 |= pData8[3] << 24;
    return  	data32;
}
///
/// Resets user pin code to default value
///
/// @return void
///
static void _ResetPin() {
    //
    // Reset pin code to default value
    //
    LS_FS_WriteU32(_pinCodeAddr, LS_DEFAULT_PIN_CODE);
}
///
/// Stores a pin to flash 
///
/// @param[in]  pPin   pointer to a pin code.
///
/// @return     LS_DeviceLock_err
///
static LS_DeviceLock_err _StorePin(uint8_t *pPin) {
    //
    // Pin code must be a valid pointer
    //
    if (!pPin) {
        return INVALID_PARAM;
    }
    uint32_t userPinCode    = _Convert_UInt8_To_UInt32(pPin);
    LS_FS_WriteU32(_pinCodeAddr, userPinCode);
    return NONE;
}
///
/// Checks recieved pin with pin stored in flash
///
/// @param[in]  pCode - pointer to a pin code.
///
/// @return     LS_DeviceLock_err
///
static LS_DeviceLock_err _CheckPin(uint8_t *pUserCode) {
    LS_DeviceLock_err   err = NONE;
    uint32_t                exisitingCode;
    //
    // Pin code must be a valid pointer
    //
    if (!pUserCode) {
        return INVALID_PARAM;
    }
    //
    // Read the existing pin code stored in flash
    //
    LS_FS_ReadU32(_pinCodeAddr, &exisitingCode);
    uint32_t userPinCode        = _Convert_UInt8_To_UInt32(pUserCode);
    if (exisitingCode == LS_DEFAULT_PIN_CODE) {
        //
        // This device is not locked
        //
        err = IS_DEFAULT;
    } else if (exisitingCode != userPinCode) {
        //
        // Pin is not a match and the pin is not default so we must be locked to a device. 
        //
        err = IS_NOTMATCH;
    } else if (exisitingCode == userPinCode) {
        //
        // Pin is a match
        //
        err = IS_MATCH;
    }
    return err; 
}
////////////////////////////////////////////////////////////////////////////////
///
///                              API Functions
///
////////////////////////////////////////////////////////////////////////////////
void LS_DeviceLock_ResetState() {
    if (lockState != DISABLED) {
        lockState = LOCKED;
    }
}
LS_DeviceLock_States LS_DeviceLock_CheckState() {
    return lockState;
}
void LS_DeviceLock_Action(LS_DeviceLock_Actions action) {
    if (action < aNONE) {
        currentAction = action;
    } else {
    }
}
void LS_DeviceLock_HandleAction(uint8_t *pUserCode) {
    LS_DeviceLock_err   err = NONE;
    //
    // Need to make sure device locking is disabled
    //
    switch(currentAction) {
        case aLOCK:
            if (lockState == DISABLED) {
                err = _StorePin(pUserCode);
                if (err == NONE) {
                    lockState = LOCKED;
                } 
            }
            break; 
        case aUNLOCK:
            if (lockState == LOCKED) {
                err = _CheckPin(pUserCode);
                if (err == IS_MATCH) {
                    lockState = UNLOCKED;
                } 
            }
            break;
        case aCHANGEPIN:
            if (lockState == UNLOCKED) {
                err = _StorePin(pUserCode);
                if (err == NONE) {
                    pinStatus = 2;
                } else {
                    pinStatus = 1;
                }
            } 
            break;
        case aDISABLE:
            if (lockState == UNLOCKED) {
                _ResetPin();
                lockState = DISABLED;
            }
            break;
        default:
            break;
            
    }
    currentAction = NONE;
}
uint8_t LS_DeviceLock_ReadPinSatus() {
    return pinStatus;
}

void LS_DeviceLock_Init(){
    //
	EE_Status ee_status = EE_OK;
    //
    //
    uint32_t dummyPin = 0;
    LS_Flash_Init();
    //LS_FS_WriteU32(_pinCodeAddr, 5555);
    ee_status = LS_FS_ReadU32(_pinCodeAddr, &dummyPin);
    //
    // if there is no pin, this might be the first time someone has connected to us
    // and want to setup a pin
    //
    if (ee_status == EE_NO_DATA || dummyPin == LS_DEFAULT_PIN_CODE) {
        lockState = DISABLED;
    } else {
        lockState = LOCKED;
    }
}

