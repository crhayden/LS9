/**
 * @file LS_Biometric_Lite.c
 * @brief Manages communication to/from host & biometric scanner
 *
 * @date 4/30/2024
 *
 **/
#include <LS_Biometric_Lite.h>
#include <math.h>
#include "hal_config.h"
#include "app_ble.h"
#include "bmlite_if.h"
#include "hcp_tiny.h"
#include "platform.h"
#include "bmlite_hal.h"
#include "LS_System.h"
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
#define DATA_BUFFER_SIZE (1024*5)
//#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define DMA_BUF_SIZE   128
#define DMA_TIMEOUT_MS 2
/** Program specific commands base number */
#define CMD_APP_BASE_VAL 0xE000

/** Program specific arguments base number */
#define ARG_APP_BASE_VAL 0x7000
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Types
///
////////////////////////////////////////////////////////////////////////////////

typedef struct
{
    volatile uint8_t flag;      /* Timeout event flag */
    uint16_t prev_cndtr;        /* Holds previous value of DMA_CNDTR (counts down) */
    uint32_t idle_irq_count;    /* Debug */
    uint32_t rx_complete_count; /* Debug */
    uint32_t rx_half_count;     /* Debug */
} dma_event_t;
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Data
///
////////////////////////////////////////////////////////////////////////////////
osMutexId_t biometricOpMutexID;
UART_HandleTypeDef huart_host = { 0 };
static DMA_HandleTypeDef hdma_rx = { 0 };
static DMA_HandleTypeDef hdma_tx = { 0 };

static volatile bool tx_half;
static volatile bool tx_done;
static volatile bool error;
static volatile bool rx_available;

static uint8_t uart_rx_fifo[DMA_BUF_SIZE];
static dma_event_t dma_uart_rx = { 0, DMA_BUF_SIZE, 0, 0, 0 };


static uint8_t hcp_txrx_buffer[MTU];
static uint8_t hcp_data_buffer[DATA_BUFFER_SIZE];

static HCP_comm_t hcp_chain = {
#ifdef BMLITE_ON_UART
    .read = platform_bmlite_uart_receive,
    .write = platform_bmlite_uart_send,
#else
//    .read = platform_bmlite_spi_receive,
//    .write = platform_bmlite_spi_send,
#endif
    .pkt_buffer = hcp_data_buffer,
    .txrx_buffer = hcp_txrx_buffer,
    .pkt_size = 0,
    .pkt_size_max = sizeof(hcp_data_buffer),
    .phy_rx_timeout = 2000,
};

osThreadId_t biometricTaskHandle;
const osThreadAttr_t biometricTask_attributes = {
  .name = "biometricTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 16
};
osMessageQueueId_t biometricQueueHandle; 
const osMessageQueueAttr_t biometricQueue_attributes = { 
    .name = "biometricQueue"
};
///
/// Debug counters
/// 
static uint32_t bmlite_on_errorCNT = 0;
static uint32_t bmlite_on_start_captureCNT = 0;
static uint32_t bmlite_on_finish_captureCNT = 0;
static uint32_t bmlite_on_start_enrollCNT = 0;
static uint32_t bmlite_on_finish_enrollCNT = 0;
static uint32_t bmlite_on_start_enrollcaptureCNT = 0;
static uint32_t bmlite_on_finish_enrollcaptureCNT = 0;
static uint32_t bmlite_on_identify_startCNT = 0;
static uint32_t bmlite_on_identify_finishCNT = 0;
////////////////////////////////////////////////////////////////////////////////
///
///                           Internal Functions
///
////////////////////////////////////////////////////////////////////////////////
static void _bmLiteLock() {
    osMutexAcquire(biometricOpMutexID, osWaitForever);
}
static void _bmLiteUnLock() {
    osMutexRelease(biometricOpMutexID);
}


///
/// @brief  Function implementing the biometric task.
///
/// @param  argument: Hardcoded to 0.
///
/// @return void
//
static void StartBiometricTask(void * argument) {

    platform_init(NULL);
    char version[100];
    //uint16_t template_id;
//    uint32_t current_id = 0;
    //bool match = false;
    app_message_t   evt;
    // These two lines for debug purpose only
    memset(version, 0, 100);

	//fpc_bep_result_t res;
    fpc_bep_result_t res = bep_version(&hcp_chain, version, 99);
    if (res != FPC_BEP_RESULT_OK) {
    	Error_Handler();
    }
//    uint16_t tempCount = 0;
//    uint16_t tempIDs[50] = {};
//    bep_template_get_count(&hcp_chain, &tempCount);
//    bep_template_get_ids(&hcp_chain);
//	uint16_t i = 0;
//	uint16_t l = 0;
//    if (tempCount > 0) {
//    	do {
//    		tempIDs[i] = hcp_chain.arg.data[l];
//    		l = l+2;
//    		i++;
//    	} while (i < tempCount && i < 50);
//    }
//    if (res != FPC_BEP_RESULT_OK) {
//    	Error_Handler();
//    }
//    res = bep_capture(&hcp_chain, 65000);
//    if (res != FPC_BEP_RESULT_OK) {
//    	Error_Handler();
//    }
//    uint16_t template_id= 0;
//    bool match = false;
//    res = bep_identify_finger(&hcp_chain, &template_id, &match);
//    if (res != FPC_BEP_RESULT_OK) {
//    	Error_Handler();
//    }
//    res = bep_enroll_finger(&hcp_chain);
//    res = bep_template_save(&hcp_chain, 9);

//    while (!match) {
//        res = bep_identify_finger(&hcp_chain, 0, &template_id, &match);
//        osDelay(100);
//    }
    
//	if (res != FPC_BEP_RESULT_OK) {
//		Error_Handler();
//	}
    bmlite_error_t 				err                 = 0;
    bmlite_callback_evt_t 		cb                  = 0;
    biometric_control_action_t 	action              = 0;
    uint16_t 					tempID              = 0;
    uint8_t                     fingerprintOffset   = 50;
    int8_t 						retVal 				= 0;

    for (;;) {
        if (osMessageQueueGet(biometricQueueHandle, &evt, 0, osWaitForever) ==  osOK){
        	//
        	// handle control events first
        	//
        	//
        	// to do: only send if BLE connected
        	//
        	//if (BleApplicationContext.BleApplicationContext_legacy.connectionHandle != 0xFFFF) {
				if(evt.bioControlVal) {
					action = evt.bioControlVal;
                    //
                    // Must be in the valid range
                    //
                    if (action > 0 && action <=100){
    					switch (action) {
                            case a_USER1_ENROLL_LEFT_THUMB:
                            case a_USER1_ENROLL_LEFT_INDEX:
                            case a_USER1_ENROLL_LEFT_MIDDLE:
                            case a_USER1_ENROLL_LEFT_RING:
                            case a_USER1_ENROLL_LEFT_PINKY:
                            case a_USER1_ENROLL_RIGHT_THUMB:
                            case a_USER1_ENROLL_RIGHT_INDEX:
                            case a_USER1_ENROLL_RIGHT_MIDDLE:
                            case a_USER1_ENROLL_RIGHT_RING:
                            case a_USER1_ENROLL_RIGHT_PINKY:
                            case a_USER2_ENROLL_LEFT_THUMB:
                            case a_USER2_ENROLL_LEFT_INDEX:
                            case a_USER2_ENROLL_LEFT_MIDDLE:
                            case a_USER2_ENROLL_LEFT_RING:
                            case a_USER2_ENROLL_LEFT_PINKY:
                            case a_USER2_ENROLL_RIGHT_THUMB:
                            case a_USER2_ENROLL_RIGHT_INDEX:
                            case a_USER2_ENROLL_RIGHT_MIDDLE:
                            case a_USER2_ENROLL_RIGHT_RING:
                            case a_USER2_ENROLL_RIGHT_PINKY:
                            case a_USER3_ENROLL_LEFT_THUMB:
                            case a_USER3_ENROLL_LEFT_INDEX:
                            case a_USER3_ENROLL_LEFT_MIDDLE:
                            case a_USER3_ENROLL_LEFT_RING:
                            case a_USER3_ENROLL_LEFT_PINKY:
                            case a_USER3_ENROLL_RIGHT_THUMB:
                            case a_USER3_ENROLL_RIGHT_INDEX:
                            case a_USER3_ENROLL_RIGHT_MIDDLE:
                            case a_USER3_ENROLL_RIGHT_RING:
                            case a_USER3_ENROLL_RIGHT_PINKY:
                            case a_USER4_ENROLL_LEFT_THUMB:
                            case a_USER4_ENROLL_LEFT_INDEX:
                            case a_USER4_ENROLL_LEFT_MIDDLE:
                            case a_USER4_ENROLL_LEFT_RING:
                            case a_USER4_ENROLL_LEFT_PINKY:
                            case a_USER4_ENROLL_RIGHT_THUMB:
                            case a_USER4_ENROLL_RIGHT_INDEX:
                            case a_USER4_ENROLL_RIGHT_MIDDLE:
                            case a_USER4_ENROLL_RIGHT_RING:
                            case a_USER4_ENROLL_RIGHT_PINKY:
                            case a_USER5_ENROLL_LEFT_THUMB:
                            case a_USER5_ENROLL_LEFT_INDEX:
                            case a_USER5_ENROLL_LEFT_MIDDLE:
                            case a_USER5_ENROLL_LEFT_RING:
                            case a_USER5_ENROLL_LEFT_PINKY:
                            case a_USER5_ENROLL_RIGHT_THUMB:
                            case a_USER5_ENROLL_RIGHT_INDEX:
                            case a_USER5_ENROLL_RIGHT_MIDDLE:
                            case a_USER5_ENROLL_RIGHT_RING:
                            case a_USER5_ENROLL_RIGHT_PINKY:
                                _bmLiteLock();
                                //
                                // Terminate any current operations
                                //
                                platform_bmlite_reset();
    							res = bep_enroll_finger(&hcp_chain);
    							res = bep_template_save(&hcp_chain, evt.bioControlVal);
                                _bmLiteUnLock();
                                retVal = (int8_t)res;
                                App_Update_Char(BIOMETRIC_STATUS_READ_EVT, (uint8_t*)&retVal);
    							break;
                            case a_USER1_DELETE_LEFT_THUMB:
                            case a_USER1_DELETE_LEFT_INDEX:
                            case a_USER1_DELETE_LEFT_MIDDLE:
                            case a_USER1_DELETE_LEFT_RING:
                            case a_USER1_DELETE_LEFT_PINKY:
                            case a_USER1_DELETE_RIGHT_THUMB:
                            case a_USER1_DELETE_RIGHT_INDEX:
                            case a_USER1_DELETE_RIGHT_MIDDLE:
                            case a_USER1_DELETE_RIGHT_RING:
                            case a_USER1_DELETE_RIGHT_PINKY:
                            case a_USER2_DELETE_LEFT_THUMB:
                            case a_USER2_DELETE_LEFT_INDEX:
                            case a_USER2_DELETE_LEFT_MIDDLE:
                            case a_USER2_DELETE_LEFT_RING:
                            case a_USER2_DELETE_LEFT_PINKY:
                            case a_USER2_DELETE_RIGHT_THUMB:
                            case a_USER2_DELETE_RIGHT_INDEX:
                            case a_USER2_DELETE_RIGHT_MIDDLE:
                            case a_USER2_DELETE_RIGHT_RING:
                            case a_USER2_DELETE_RIGHT_PINKY:
                            case a_USER3_DELETE_LEFT_THUMB:
                            case a_USER3_DELETE_LEFT_INDEX:
                            case a_USER3_DELETE_LEFT_MIDDLE:
                            case a_USER3_DELETE_LEFT_RING:
                            case a_USER3_DELETE_LEFT_PINKY:
                            case a_USER3_DELETE_RIGHT_THUMB:
                            case a_USER3_DELETE_RIGHT_INDEX:
                            case a_USER3_DELETE_RIGHT_MIDDLE:
                            case a_USER3_DELETE_RIGHT_RING:
                            case a_USER3_DELETE_RIGHT_PINKY:
                            case a_USER4_DELETE_LEFT_THUMB:
                            case a_USER4_DELETE_LEFT_INDEX:
                            case a_USER4_DELETE_LEFT_MIDDLE:
                            case a_USER4_DELETE_LEFT_RING:
                            case a_USER4_DELETE_LEFT_PINKY:
                            case a_USER4_DELETE_RIGHT_THUMB:
                            case a_USER4_DELETE_RIGHT_INDEX:
                            case a_USER4_DELETE_RIGHT_MIDDLE:
                            case a_USER4_DELETE_RIGHT_RING:
                            case a_USER4_DELETE_RIGHT_PINKY:
                            case a_USER5_DELETE_LEFT_THUMB:
                            case a_USER5_DELETE_LEFT_INDEX:
                            case a_USER5_DELETE_LEFT_MIDDLE:
                            case a_USER5_DELETE_LEFT_RING:
                            case a_USER5_DELETE_LEFT_PINKY:
                            case a_USER5_DELETE_RIGHT_THUMB:
                            case a_USER5_DELETE_RIGHT_INDEX:
                            case a_USER5_DELETE_RIGHT_MIDDLE:
                            case a_USER5_DELETE_RIGHT_RING:
                            case a_USER5_DELETE_RIGHT_PINKY:
    							tempID = action-fingerprintOffset;
                                _bmLiteLock();
                                //
                                // Terminate any current operations
                                //
                                platform_bmlite_reset();
    							res = bep_template_remove(&hcp_chain, tempID);
                                _bmLiteUnLock(); 
                                retVal = (int8_t)res;
                                App_Update_Char(BIOMETRIC_STATUS_READ_EVT, (uint8_t*)&retVal);
    							break;
    						default:
    							break;
                        }
					}
				}
				//
				// Errors
				//
				else if (evt.isBioError) {
					switch (evt.bioError) {
						case BMLITE_ERROR_OK:
						case BMLITE_ERROR_CAPTURE:
						case BMLITE_ERROR_CAPTURE_START:
						case BMLITE_ERROR_ENROLL_START:
						case BMLITE_ERROR_ENROLL_ADD:
						case BMLITE_ERROR_ENROLL_FINISH:
						case BMLITE_ERROR_WRONG_ANSWER:
						case BMLITE_ERROR_FINGER_WAIT:
						case BMLITE_ERROR_IDENTYFY:
						case BMLITE_ERROR_TEMPLATE_SAVE:
						case BMLITE_ERROR_TEMPLATE_DELETE:
						case BMLITE_ERROR_TEMPLATE_COUNT:
						case BMLITE_ERROR_TEMPLATE_GETIDS:
						case BMLITE_ERROR_IMAGE_EXTRACT:
						case BMLITE_ERROR_IMAGE_GETSIZE:
						case BMLITE_ERROR_IMAGE_GET:
						case BMLITE_ERROR_GETVERSION:
						case BMLITE_ERROR_SW_RESET:
						case BMLITE_ERROR_CALIBRATE:
						case BMLITE_ERROR_CALIBRATE_DELETE:
						case BMLITE_ERROR_SEND_CMD:
						case BMLITE_ERROR_GET_ARG:
							  err = evt.bioError;
							  App_Update_Char(BIOMETRIC_STATUS_READ_EVT, (uint8_t*)&err);
							  //App_Update_Char(CUSTOM_STM_BIOMETRIC_STATUS, (uint8_t*)&err);
//							break;
						default:
							break;
					}
				}
				//
				// Callback
				//
				else {
					switch (evt.cb) {
						case    BIOMETRIC_ON_START_CAPTURE:
						case    BIOMETRIC_ON_FINISH_CAPTURE:
						case    BIOMETRIC_ON_START_ENROLL:
						case    BIOMETRIC_ON_FINISH_ENROLL:
						case    BIOMETRIC_ON_START_ENROLLCAPTURE:
						case    BIOMETRIC_ON_FINISH_ENROLLCAPTURE:
						case    BIOMETRIC_ON_IDENTIFY_START:
						case    BIOMETRIC_ON_IDENTIFY_FINISH:

								cb = evt.cb;
								//App_Update_Char(CUSTOM_STM_BIOMETRIC_STATUS, (uint8_t*)&cb);
								App_Update_Char(BIOMETRIC_STATUS_READ_EVT, (uint8_t*)&cb);
							break;
						default:
							break;

					}
				}
        	//}
        }
    }
}
//------------------------------------------------------------------------------
//
//                           Callback Handlers
//
//------------------------------------------------------------------------------
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart_host.Instance) {
        rx_available = true;
        dma_uart_rx.rx_half_count++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart_host.Instance) {
        uint16_t curr_cndtr = __HAL_DMA_GET_COUNTER(huart->hdmarx);

        /*
         * Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and
         * DMA Rx Complete Interrupt is generated, but there is no new character during timeout.
         */
        if (dma_uart_rx.flag && curr_cndtr == DMA_BUF_SIZE) {
            dma_uart_rx.flag = 0;
            return;
        }

        if (!dma_uart_rx.flag) {
            dma_uart_rx.rx_complete_count++;
        }
        dma_uart_rx.flag = 0;

        rx_available = true;
    }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart_host.Instance) {
        tx_half = true;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart_host.Instance) {
        tx_done = true;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart_host.Instance) {
        error = true;
    }
}
void FPC_BMLITE_USART_IRQ_HANDLER(void)
{
    HAL_UART_IRQHandler(&huart_host);

    /* UART IDLE Interrupt */
    if ((huart_host.Instance->ISR & USART_ISR_IDLE) != RESET) {
        huart_host.Instance->ICR = UART_CLEAR_IDLEF;
        dma_uart_rx.flag = 1;
        dma_uart_rx.idle_irq_count++;
        if(hdma_rx.XferCpltCallback != NULL) {
            hdma_rx.XferCpltCallback(&hdma_rx);
        }
    }
}

/**
 * This function handles Host USART DMA RX global interrupt.
 */
void FPC_BMLITE_USART_DMA_IRQ_HANDLER_RX(void)
{
    HAL_DMA_IRQHandler(huart_host.hdmarx);
}

/**
 * This function handles Host USART DMA TX global interrupt.
 */
void FPC_BMLITE_USART_DMA_IRQ_HANDLER_TX(void)
{
    HAL_DMA_IRQHandler(huart_host.hdmatx);
}

void bmlite_on_error(bmlite_error_t error, int32_t value) {
    osStatus_t      ret;
    app_message_t   evt;
    memset(&evt, 0, sizeof(evt));
    evt.isBioError  = true;
    evt.bioError    = error;
    evt.bioErrorVal	= value;
    uint8_t v       = (uint8_t)error;
    //ret             = osMessageQueuePut(biometricQueueHandle, &evt, 0, 0);
    App_Update_Char(BIOMETRIC_STATUS_READ_EVT, (uint8_t*)&v);
    if (ret != osOK) {
        //Error_Handler();
    }
	bmlite_on_errorCNT++;
}
void bmlite_on_start_capture() {
    bmlite_on_start_captureCNT++;
}
void bmlite_on_finish_capture() {
    bmlite_on_finish_captureCNT++;
}
void bmlite_on_start_enroll() {
    bmlite_on_start_enrollCNT++;
}
void bmlite_on_finish_enroll() {
    bmlite_on_finish_enrollCNT++;
}
void bmlite_on_start_enrollcapture() {
    bmlite_on_start_enrollcaptureCNT++;
}
void bmlite_on_finish_enrollcapture() {
    bmlite_on_finish_enrollcaptureCNT++;
}
void bmlite_on_identify_start() {
    bmlite_on_identify_startCNT++;
}
void bmlite_on_identify_finish() {
    bmlite_on_identify_finishCNT++;
}
////////////////////////////////////////////////////////////////////////////////
///
///                           External Functions
///
////////////////////////////////////////////////////////////////////////////////
//fpc_bep_result_t LS_BM_Lite_Capture(){
//
//    //uint16_t template_id= 0;
//    //bool match = false;
//    _bmLiteLock();
//    fpc_bep_result_t res = bep_capture(&hcp_chain, 0);
//    _bmLiteUnLock();
////    int8_t v = (int8_t)res;
////    App_Update_Char(BIOMETRIC_STATUS_READ_EVT, (uint8_t*)&v);
//    return res;
//}
bool LS_BM_Lite_Identify() {
    uint16_t template_id= 0;
    bool match = false;
    //_bmLiteLock();
    fpc_bep_result_t res = bep_identify_finger(&hcp_chain, 1000, &template_id, &match);
    //_bmLiteUnLock();
    if (res == FPC_BEP_RESULT_TIMEOUT || res == FPC_BEP_RESULT_IO_ERROR) {
        platform_bmlite_reset();
    }
    int8_t v = (int8_t)res;
    if (match && res == FPC_BEP_RESULT_OK) {
        App_Update_Char(BIOMETRIC_STATUS_READ_EVT, (uint8_t*)&v);
    }
//    if (res != FPC_BEP_RESULT_OK) {
//    	Error_Handler();
//    }
    return match;
}
void LS_BM_Lite_Wait_For_Finger_Present(){
    //_bmLiteLock();
    sensor_wait_finger_not_present(&hcp_chain, 1000);
    //_bmLiteUnLock();
}
/**
 * USART init function.
 * @param[in] baudrate Initial baud rate.
 */
void stm_uart_init(uint32_t baudrate)
{
    // HAL_StatusTypeDef status;

    huart_host.Instance           = FPC_BMLITE_USART;
    huart_host.Init.BaudRate      = baudrate;
    huart_host.Init.WordLength    = UART_WORDLENGTH_8B;
    huart_host.Init.StopBits      = UART_STOPBITS_1;
    huart_host.Init.Parity        = UART_PARITY_NONE;
    huart_host.Init.Mode          = UART_MODE_TX_RX;
    huart_host.Init.HwFlowCtl     = UART_HWCONTROL_NONE;
    huart_host.Init.OverSampling  = UART_OVERSAMPLING_16;
    huart_host.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart_host.Init.OneBitSampling          = UART_ONE_BIT_SAMPLE_DISABLE;
    huart_host.AdvancedInit.AdvFeatureInit  = UART_ADVFEATURE_NO_INIT;

    // huart_host.Init.BaudRate = 921600;

    HAL_UART_Init(&huart_host);
    HAL_UARTEx_EnableFifoMode(&huart_host);

    /* UART1 IDLE Interrupt Configuration */
    SET_BIT(FPC_BMLITE_USART->CR1, USART_CR1_IDLEIE);

    HAL_Delay(100);

    /* Start UART RX */
    HAL_UART_Receive_DMA(&huart_host, uart_rx_fifo, DMA_BUF_SIZE);
}

void uart_host_reinit(void)
{
    if (huart_host.Init.BaudRate) {
        huart_host.Instance->BRR = (uint16_t)(UART_DIV_SAMPLING16(SystemCoreClock,
                huart_host.Init.BaudRate, huart_host.Init.ClockPrescaler));
    }
}

/**
 * Initialize UART GPIO.
 * @param[in, out] huart UART Handle.
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    // HAL_StatusTypeDef status;

    if (huart->Instance == FPC_BMLITE_USART) {
        /* Peripheral clock enable */
        FPC_BMLITE_USART_CLK_ENABLE();
        FPC_BMLITE_USART_RX_PORT_CLK_ENABLE();
        FPC_BMLITE_USART_TX_PORT_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull        = GPIO_PULLUP;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate   = FPC_BMLITE_USART_AF;

        GPIO_InitStruct.Pin         = FPC_BMLITE_USART_RX_PIN;
        HAL_GPIO_Init(FPC_BMLITE_USART_RX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin         = FPC_BMLITE_USART_TX_PIN;
        HAL_GPIO_Init(FPC_BMLITE_USART_TX_PORT, &GPIO_InitStruct);

        /* Peripheral DMA init*/
        hdma_rx.Instance = FPC_BMLITE_USART_DMA_CHANNEL_RX;
        hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_rx.Init.Mode = DMA_CIRCULAR;
        hdma_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
        hdma_rx.Init.Request = FPC_BMLITE_USART_DMA_REQ_RX;

        HAL_DMA_Init(&hdma_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_rx);

        hdma_tx.Instance = FPC_BMLITE_USART_DMA_CHANNEL_TX;
        hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_tx.Init.Mode = DMA_NORMAL;
        hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_tx.Init.Request = FPC_BMLITE_USART_DMA_REQ_TX;

        HAL_DMA_Init(&hdma_tx);
        __HAL_LINKDMA(huart, hdmatx, hdma_tx);

        /* UART Host RX DMA interrupt configuration */
        HAL_NVIC_SetPriority(FPC_BMLITE_USART_DMA_IRQn_RX, 6, 0);
        HAL_NVIC_EnableIRQ(FPC_BMLITE_USART_DMA_IRQn_RX);

        /* UART Host TX DMA interrupt configuration */
        HAL_NVIC_SetPriority(FPC_BMLITE_USART_DMA_IRQn_TX, 6, 0);
        HAL_NVIC_EnableIRQ(FPC_BMLITE_USART_DMA_IRQn_TX);

        /* Peripheral interrupt init*/
        HAL_NVIC_SetPriority(FPC_BMLITE_USART_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(FPC_BMLITE_USART_IRQn);

        HAL_GPIO_WritePin(BMLITE_RST_PORT, BMLITE_RST_PIN, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin         = BMLITE_RST_PIN;
        GPIO_InitStruct.Mode        = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull        = GPIO_PULLUP;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate   = 0;
        HAL_GPIO_Init(BMLITE_RST_PORT, &GPIO_InitStruct);
    }
}

size_t hal_bmlite_uart_write(const uint8_t *data, size_t size)
{
    HAL_StatusTypeDef result = HAL_ERROR;

    tx_half = false;
    tx_done = false;
    error = false;

    result = HAL_UART_Transmit_DMA(&huart_host, data, size);
    if (result != HAL_OK) {
        return 0;
    }

    while (!tx_done) {

        if (error) {
            result = HAL_ERROR;
            return 0;
        }
    }

    return size;
}

size_t hal_bmlite_uart_read(uint8_t *data, size_t size)
{
    uint32_t n_sent = 0;

    /* Restart Host UART RX DMA transfer if it has stopped for some reason */
    if (huart_host.RxState != HAL_UART_STATE_BUSY_RX) {
        dma_uart_rx.prev_cndtr = DMA_BUF_SIZE;
        HAL_UART_Receive_DMA(&huart_host, uart_rx_fifo, DMA_BUF_SIZE);
    }

    if (!size) {
        return 0;
    }

    uint16_t curr_cndtr = __HAL_DMA_GET_COUNTER(&hdma_rx);

    if (curr_cndtr != dma_uart_rx.prev_cndtr) {
        uint32_t cur_pos = DMA_BUF_SIZE - curr_cndtr;
        uint32_t prev_pos;
        uint32_t length;

        /* Determine start position in DMA buffer based on previous CNDTR value */
        prev_pos = DMA_BUF_SIZE - dma_uart_rx.prev_cndtr;
        if (prev_pos < cur_pos) {
            length = MIN(cur_pos - prev_pos, size);
        } else {
            /* Copy until end of buffer first */
            length = MIN(DMA_BUF_SIZE - prev_pos, size);
        }
        memcpy(data, &uart_rx_fifo[prev_pos], length);
        data += length;
        n_sent += length;
        dma_uart_rx.prev_cndtr -= length;
        if (dma_uart_rx.prev_cndtr == 0) {
            dma_uart_rx.prev_cndtr = DMA_BUF_SIZE;
        }
        if (prev_pos > cur_pos) {
            /* Copy from start of buffer */
            length = MIN(cur_pos, size);
            memcpy(data, uart_rx_fifo, length);
            data += length;
            n_sent += length;
            dma_uart_rx.prev_cndtr -= length;
        }
    } 

    return n_sent;

}

bool uart_host_rx_data_available(void)
{
    return rx_available;
}
void LS_BM_Lite_Init() {
	biometricOpMutexID		= osMutexNew(NULL);
    biometricQueueHandle    = osMessageQueueNew(16, sizeof(app_message_t), &biometricQueue_attributes);
    biometricTaskHandle     = osThreadNew(StartBiometricTask, NULL, &biometricTask_attributes);
}
////////////////////////////////////////////////////////////////////////////////
///
///                              Global Data
///
////////////////////////////////////////////////////////////////////////////////
