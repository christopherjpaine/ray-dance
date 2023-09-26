
/* == INCLUDES ============================================================= */
#include "stm32f7xx_hal.h"

/* == DEFINES ============================================================== */
#define DMX_MAX_RX_BUFFER_SIZE  513

#define DMX_CHANNEL_START   0

#define DMX_NUM_CHANNELS    6

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

// Data ready flag
static uint8_t dmx_data_ready = 0;

static uint8_t dmx_buffer[DMX_MAX_RX_BUFFER_SIZE] = {0};

static uint8_t dmx_data[DMX_BYTES_TO_RX] = {0};

/* RX Complete Interrupt 
 * This will occur when we have received a full frame of DMX data. 
 * (Or a buffer of the same size) */
static void dmx_RxCompleteCallback (UART_HandleTypeDef* huart) {
    HAL_StatusTypeDef s = HAL_OK;

    switch (dmx_state) {
        case dmx_STATE_RESET:
            /* This should not be possible as the transfer hasn't started.
             * We just ignore it. */
            return;

        case dmx_STATE_RX:
            /* We received a full frame. Copy the data, mark it as ready,
             * move the unsynced state and start a new xfer. Any data rx
             * between now and the frame sync will be discarded. Usually
             * there is nothing but it could cause a problem. */
            memcpy(dmx_data, dmx_buffer, DMX_BYTES_TO_RX);
            dmx_data_ready = 1;
            s = HAL_UART_Receive_DMA(huart, dmx_buffer, DMX_MAX_RX_BUFFER_SIZE);
            if (HAL_OK != s) {
                dmx_state = dmx_STATE_FAILED;
            }
            return;
        
        case dmx_STATE_UNSYNC:
            /* If we receive DMX_MAX_BYTES without getting a sync then
             * it's probably not DMX data so we can discard it and start a 
             * fresh xfer instead. */
            s = HAL_UART_Receive_DMA(huart, dmx_buffer, DMX_MAX_RX_BUFFER_SIZE);
            if (HAL_OK != s) {
                dmx_state = dmx_STATE_FAILED;
            }
            return;

        case dmx_STATE_FAILED:
            /* Catch with breakpoint but not expected to happen. */
            __BKPT();
            return;
    }
}

/* Framing Error Interrupt 
 * This should be called when the DMX USART detects a break condition.
 * It assumes that any existing transfer has been aborted and due to the
 * oddities of ST HAL, this will only work if you disable overrun errors.
 * It will notify the task to begin receiving a new DMX frame. */
static void dmx_ErrorCallback (UART_HandleTypeDef* huart) {
    HAL_StatusTypeDef s = HAL_OK;

    switch (dmx_state) {
        case dmx_STATE_RESET:
            /* We have not finished initialising yet so ignore this interrupt.
             * Unfortunately we have to do this, and disable OVR error because
             * the ST HAL doesn't really cope with this case. */
            return;

        case dmx_STATE_UNSYNC:
            /* We are unsynced, awating a framing error to start reception. 
             * Regardless of the error we should always start a new transfer.
             * But we only move to the RX state if the frame error is set. 
             * We do this in quite a "tolerant" way by ignoring all other errors
             * if FE is set. */
            if ( huart->ErrorCode & HAL_UART_ERROR_FE ) {
                dmx_state = dmx_STATE_RX;
            }
            s = HAL_UART_Receive_DMA(huart, dmx_buffer, DMX_MAX_RX_BUFFER_SIZE);
            if (HAL_OK != s) {
                dmx_state = dmx_STATE_FAILED;
            }
            return;
        
        case dmx_STATE_RX:
            /* We did not receive a full frame of data. If it was due to framing 
             * error EXCLUSIVELY then we can assume that it was just a short 
             * frame. In that case, copy the data and mark it as ready. If it 
             * was due to a different error then move to unsync. In both cases 
             * start a new xfer. */
            if ( (huart->ErrorCode & HAL_UART_ERROR_FE) == HAL_UART_ERROR_FE) {   
                /* If any data hasn't been rx'd then it will just remain as it 
                 * was previously in dmx_buffer. */
                memcpy(dmx_data, dmx_buffer, DMX_BYTES_TO_RX);
                dmx_data_ready = 1;
            } else {
                dmx_state = dmx_STATE_UNSYNC;
            }
            s = HAL_UART_Receive_DMA(huart, dmx_buffer, DMX_MAX_RX_BUFFER_SIZE);
            if (HAL_OK != s) {
                dmx_state = dmx_STATE_FAILED;
            }
            return;

        case dmx_STATE_FAILED:
            /* In the case we failed to init a transfer, we would need to re-init
             * right now just panic. */
            __BKPT();
            return;
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
		__BKPT();
	}
    if (dmx_data_ready) {
    	dmx_data_ready = 0;
        dmx_data[4] = '\n';
        HAL_UART_Transmit_DMA(output_huart, dmx_data, 5);
        LED_SetAll(dmx_data[1], dmx_data[2], dmx_data[3]);
        LED_Sync();
    }
}
