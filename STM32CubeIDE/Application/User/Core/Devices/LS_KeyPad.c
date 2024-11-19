/*
 * LS_KeyPad.c
 * @brief Manages keypad communication
 *
 *  Created on: Aug 24, 2024
 *      Author: chris
 */
#include <LS_KeyPad.h>
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Constants
///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Macros
///
////////////////////////////////////////////////////////////////////////////////
#define MAX_ALLOWED_FAILED_ATTEMPTS	2
#define NUM_OF_PINCODE_DIGITS		4
#define DEBOUNCE_INTERVAL 			10
#define POLL_INTERVAL 				50
#define KEYPAD_LED_TIMEOUT_VAL 		POLL_INTERVAL * 100 // If no buttons are pressed within 5 seconds set LED to "idle"
#define USER_LOCKOUT_TIMER_VAL		POLL_INTERVAL * 20 * 60 * 5 // if user enters pin wrong 2 or more times lock them out for 5 mins 
////////////////////////////////////////////////////////////////////////////////
////
///
///                           Internal Types
///
////////////////////////////////////////////////////////////////////////////////
typedef struct {
	uint8_t 	keyPadVals[4];
	uint8_t 	index;
	uint8_t 	numFailedAttempts;
	bool		shouldLockUserOut;
	uint32_t 	LEDTimerVal;
	uint32_t 	UserTimerVal;
} keypad_info_t;
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Data
///
////////////////////////////////////////////////////////////////////////////////
uint8_t b1_pressed = 0;
uint8_t b2_pressed = 0;
uint8_t b3_pressed = 0;
keypad_info_t kp_info = {.keyPadVals = {0}, .index = 0, .numFailedAttempts = 0, .shouldLockUserOut = false};
osThreadId_t keypadTaskHandle;
const osThreadAttr_t keypadTask_attributes = {
  .name = "keypadTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Functions
///
////////////////////////////////////////////////////////////////////////////////
static void _recordButton(uint8_t val) {
	if (kp_info.index <= NUM_OF_PINCODE_DIGITS-1) {
		kp_info.keyPadVals[kp_info.index] = val;
	    kp_info.index++;
	}
}
static GPIO_PinState _ReadButton(GPIO_TypeDef *pPort, uint16_t pin) {
    osDelay(DEBOUNCE_INTERVAL);  
    if (HAL_GPIO_ReadPin(pPort, pin) == GPIO_PIN_RESET) {
        return 1;  // Valid release
    }
    return 0;  // Still pressed or noisy signal
}
static bool _PollButtons() {


    if (HAL_GPIO_ReadPin(GPIOB, MB_SW1_Pin) == GPIO_PIN_SET) {
        if (b1_pressed == 0) { // Button was previously not pressed
            b1_pressed = 1; // Mark it as pressed
        }
    } else {
        if (b1_pressed == 1) { // Button was pressed, now released
            if (_ReadButton(GPIOB, MB_SW1_Pin)) {
                // Button 1 release event
                //printf("Button 1 Released!\n");
                _recordButton('1');
 	            b1_pressed = 0; // Mark it as not pressed
				return true;
            }
            b1_pressed = 0; // Mark it as not pressed
        }
    }

    // Button 2
    if (HAL_GPIO_ReadPin(GPIOB, MB_SW2_Pin) == GPIO_PIN_SET) {
        if (b2_pressed == 0) {
            b2_pressed = 1;
        }
    } else {
        if (b2_pressed == 1) {
            if (_ReadButton(GPIOB, MB_SW2_Pin)) {
                //printf("Button 2 Released!\n");
                _recordButton('2');
 	            b2_pressed = 0;
				return true;
            }
            b2_pressed = 0;
        }
    }

    // Button 3
    if (HAL_GPIO_ReadPin(GPIOB, MB_SW3_Pin) == GPIO_PIN_SET) {
        if (b3_pressed == 0) {
            b3_pressed = 1;
        }
    } else {
        if (b3_pressed == 1) {
            if (_ReadButton(GPIOB, MB_SW3_Pin)) {
                //printf("Button 3 Released!\n");
                _recordButton('3');
 	            b3_pressed = 0;
				return true;
            }
            b3_pressed = 0;
        }
    }
 	return false;
}
///
/// @brief  Function implementing the keypad task.
///
/// @param  argument: Hardcoded to 0.
///
/// @return void
///
static void StartKeypadTask(void * argument) {
	bool didPressKP = 	false;
	app_message_t		evt;
  	osStatus_t			ret;
	for (;;) {
		//
		// lock the user out for USER_LOCKOUT_TIMER_VAL seconds if they've entired pin wrong twice 
		//
		if (!kp_info.shouldLockUserOut) {
			//
			// Poll for button presses
			//
			didPressKP = _PollButtons();
			//
			// if we recieved 4 digits - process 
			//
			if (kp_info.index == NUM_OF_PINCODE_DIGITS) {
				if (LS_DeviceLock_CheckState() == LOCKED ) {
					LS_DeviceLock_Action(aUNLOCK);
					LS_DeviceLock_HandleAction(&kp_info.keyPadVals[0]);
					if (LS_DeviceLock_CheckState() == UNLOCKED){
						//
						// success. update LED. unlock weapon?
						// 
						kp_info.shouldLockUserOut = false;
				        evt.type      = MOTOR_UNLOCK;
				        ret           = osMessageQueuePut(motorQueueHandle, &evt, 0, 0);
				        if (ret != osOK) {
				            Error_Handler();
				        }
						memset(&kp_info.keyPadVals[0], 0, sizeof(kp_info.keyPadVals));
						kp_info.index = 0;
					} else if (++kp_info.numFailedAttempts == MAX_ALLOWED_FAILED_ATTEMPTS) {
						kp_info.shouldLockUserOut = true;
						kp_info.numFailedAttempts = 0;
						//
						// set lockout LED color
						//
					} else {
						memset(&kp_info.keyPadVals[0], 0, sizeof(kp_info.keyPadVals));
						kp_info.index = 0;
					}
				} else if (LS_DeviceLock_CheckState() == UNLOCKED) {
					LS_DeviceLock_Action(aLOCK);
					LS_DeviceLock_HandleAction(&kp_info.keyPadVals[0]);
					if (LS_DeviceLock_CheckState() == LOCKED){
						//
						// success. update LED. lock weapon?
						// 
						kp_info.shouldLockUserOut = false;
				        evt.type      = MOTOR_LOCK;
				        ret           = osMessageQueuePut(motorQueueHandle, &evt, 0, 0);
				        if (ret != osOK) {
				            Error_Handler();
				        }
						memset(&kp_info.keyPadVals[0], 0, sizeof(kp_info.keyPadVals));
						kp_info.index = 0;
					} else if (++kp_info.numFailedAttempts == MAX_ALLOWED_FAILED_ATTEMPTS) {
						kp_info.shouldLockUserOut = true;
						kp_info.numFailedAttempts = 0;
						//
						// set lockout LED color
						//
					} else {
						memset(&kp_info.keyPadVals[0], 0, sizeof(kp_info.keyPadVals));
						kp_info.index = 0;
					}
				}
			}
			//
			// Reset or advance LED timer
			//
			if (didPressKP) {
				kp_info.LEDTimerVal = 0;
			} else {
				kp_info.LEDTimerVal += POLL_INTERVAL;
			}
			//
			// If timer expired. Clean up
			//
			if (kp_info.LEDTimerVal >= KEYPAD_LED_TIMEOUT_VAL) {
				//
				// set LED to idle. Discard pressed digits
				//
				memset(&kp_info.keyPadVals[0], 0, sizeof(kp_info.keyPadVals));
				kp_info.index = 0;
				kp_info.LEDTimerVal = 0;
			}
		} else {
			//
			// user is locked out for USER_LOCKOUT_TIMER_VAL
			//
			kp_info.UserTimerVal += POLL_INTERVAL;
			if (kp_info.UserTimerVal >= USER_LOCKOUT_TIMER_VAL) {
				kp_info.UserTimerVal = 0;
				kp_info.shouldLockUserOut = false;
			}
		}
		osDelay(POLL_INTERVAL);
	}
}
//------------------------------------------------------------------------------
//
//                           Callback Handlers
//
//------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////
///
///                           External Functions
///
////////////////////////////////////////////////////////////////////////////////
void LS_KeyPad_Init() {
	  keypadTaskHandle = osThreadNew(StartKeypadTask, NULL, &keypadTask_attributes);
}
////////////////////////////////////////////////////////////////////////////////
///
///                              Global Data
///
////////////////////////////////////////////////////////////////////////////////

