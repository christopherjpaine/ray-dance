
/* == INCLUDES ============================================================= */
#include "stm32f7xx_hal.h"

/* == DEFINES ============================================================== */
#define DMX_MAX_RX_BUFFER_SIZE  513

#define DMX_CHANNEL_START   0

#define DMX_NUM_CHANNELS    51

#define DMX_CHANNEL_END     DMX_CHANNEL_START + DMX_NUM_CHANNELS

/* bytes to read = channels needed + 1 start byte. */
#define DMX_BYTES_TO_RX   DMX_CHANNEL_END + 1

/* == TYPES ================================================================ */
typedef enum dmx_State_etag {
	dmx_STATE_RESET,
    dmx_STATE_UNSYNC, // Waiting for BREAK condition
    dmx_STATE_RX,        // Receiving DMX Data
	dmx_STATE_FAILED,
}dmx_State_e;

/* == FILE STATIC ========================================================== */

// Receiver State
static dmx_State_e dmx_state = dmx_STATE_UNSYNC;

UART_HandleTypeDef* dmx_uart;

static uint8_t dmx_errors = 0;

// Data ready flag
static uint8_t dmx_data_ready = 0;

static uint8_t dmx_buffer[DMX_MAX_RX_BUFFER_SIZE] = {0};

static uint8_t dmx_data[DMX_BYTES_TO_RX] = {0};

/* RX Complete Interrupt 
 * This will occur when we have received the number of bytes that care about
 * in the DMX frame. Copy them and notify the task. */
static void dmx_RxCompleteCallback (UART_HandleTypeDef* huart) {
    /* If we are in the RX State */
    if ( dmx_state == dmx_STATE_RX ) {
        /* Store Received Data and signal task to print it. */
        dmx_data_ready = 1;
        /* Move to Unsynced and Enable Receiver to detect next BREAK */
        dmx_state = dmx_STATE_UNSYNC;
        memcpy(dmx_data, dmx_buffer, DMX_BYTES_TO_RX);
        HAL_UART_Receive_DMA(huart, dmx_buffer, DMX_MAX_RX_BUFFER_SIZE);
    /* If we are not synced, then just re-enable the receiver. This will 
     * happen if we are being given data outside of DMX framing. */
    } else if (dmx_state == dmx_STATE_UNSYNC) {
        HAL_UART_Receive_DMA(huart, dmx_buffer, DMX_MAX_RX_BUFFER_SIZE);
    } else {
        __asm("nop");
    }
}

/* Framing Error Interrupt 
 * This should be called when the DMX USART detects a break condition.
 * It assumes that any existing transfer has been aborted.
 * It will notify the task to begin receiving a new DMX frame.
 */
static void dmx_ErrorCallback (UART_HandleTypeDef* huart) {
	/* If we are in the reset state, we received a error before the rx was
	 * even set up... Ignore it. */
	if (dmx_state == dmx_STATE_RESET) {
		dmx_errors = 1;
		return;
	}
    /* If the error is a framing error */
    if ( huart->ErrorCode & HAL_UART_ERROR_FE ) {
        /* If we are unsynced, move into the receive state and start 
         * the receiver. If we are in the receive state, then we got a framing 
         * error before we expected, so just restart the receiver. */
        if ( dmx_state == dmx_STATE_UNSYNC || dmx_state == dmx_STATE_RX ) {
            dmx_state = dmx_STATE_RX;
            if (HAL_OK != HAL_UART_Receive_DMA(huart, dmx_buffer, DMX_MAX_RX_BUFFER_SIZE)) {
            	__NOP();
            }
            return;
        }
    }
	if (huart->ErrorCode) {
		dmx_state = dmx_STATE_FAILED;
	}
}

static void dmx_RegisterCallbacks(UART_HandleTypeDef* huart) {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_UART_RegisterCallback(huart, HAL_UART_RX_COMPLETE_CB_ID, dmx_RxCompleteCallback);
    status = HAL_UART_RegisterCallback(huart, HAL_UART_ERROR_CB_ID, dmx_ErrorCallback);
}

/* DMX Init 
 * Currently assumes both config and GPIO mapping have been taken care 
 * of by cube mx generater. 
 * TODO handle this internally or with a compilation flag. */
void DMX_Init (UART_HandleTypeDef* huart) {

	dmx_uart = huart;
	dmx_state = dmx_STATE_RESET;
    // TODO hardware init

    dmx_RegisterCallbacks(huart);

    /* We always start unsynced so just enable the receiver. This will mean
     * that a framing error is generated on the next BREAK condition and we
     * can start reception */
    HAL_UART_Receive_DMA(huart, dmx_buffer, DMX_MAX_RX_BUFFER_SIZE);
    dmx_state = dmx_STATE_UNSYNC;

}

#include "led.h"

/* DMX Task */
void DMX_Task (UART_HandleTypeDef* output_huart) {
	if (dmx_state == dmx_STATE_FAILED) {
		_BKPT();
	}
    if (dmx_data_ready) {
    	dmx_data_ready = 0;
    	if (dmx_errors) {
			dmx_data[0] = 'e';
		}
        dmx_data[4] = '\n';
        HAL_UART_Transmit_DMA(output_huart, dmx_data, 5);
        LED_SetAll(dmx_data[1], dmx_data[2], dmx_data[3]);
        LED_Sync();
    }
}
