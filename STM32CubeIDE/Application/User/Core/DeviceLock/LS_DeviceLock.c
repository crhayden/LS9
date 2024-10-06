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
///
/// Device is unlocked from mobile device
///
#define LS_DEVICE_LOCK_IS_UNLOCKED            0			      
///
/// Device is locked to a mobile device
///
#define LS_DEVICE_LOCK_IS_LOCKED              1			      
///
/// Pin entered is not valid
///
#define LS_DEVICE_LOCK_PINSTATUS_IS_NOT_VALID 0			      
///
/// Pin entered is valid
///
#define LS_DEVICE_LOCK_PINSTATUS_IS_VALID     1	
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
uint8_t pinStatus = 0;
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
    LS_DeviceLock_err   RF_err = NONE;
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
        RF_err = IS_DEFAULT;
    } else if (exisitingCode != userPinCode) {
        //
        // Pin is not a match and the pin is not default so we must be locked to a device. 
        //
        RF_err = IS_NOTMATCH;
    } else if (exisitingCode == userPinCode) {
        //
        // Pin is a match
        //
        RF_err = IS_MATCH;
    }
    return RF_err; 
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
    LS_DeviceLock_err   err = aNONE;
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
        case aDISABLE:
            if (lockState == UNLOCKED) {
                _ResetPin();
                lockState = DISABLED;
            }
            break;
        case aCHANGEPIN:
            if (lockState == UNLOCKED) {
                err = _StorePin(pUserCode);
                if (err == NONE) {
                    pinStatus = 1;
                } 
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
    //
    //
    uint32_t dummyPin = 0;
    LS_Flash_Init();
    //LS_FS_WriteU32(_pinCodeAddr, 5555);
    LS_FS_ReadU32(_pinCodeAddr, &dummyPin);
    //
    // If there is a pin set then handle to pin reset counter
    //
    if (dummyPin == LS_DEFAULT_PIN_CODE) {
        lockState = DISABLED;
    }
}

