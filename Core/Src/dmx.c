
/* == INCLUDES ============================================================= */
#include "dmx.h"

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

#include "led.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/* == CONFIGURATION ======================================================== */

/* Start address of the fixture (remember this is not 0 indexed. )*/
#define DMX_CHANNEL_START   1

/* Number of channels required by the fixture. This should be atleast as 
 * big as the size of the DMX_Data type in the interface. */
#define DMX_NUM_CHANNELS    10

/* == DEFINES ============================================================== */
#define DMX_MAX_RX_BUFFER_SIZE  513 // 512 + start byte

/* == TYPES ================================================================ */
typedef enum dmx_State_etag {
	dmx_STATE_RESET,
    dmx_STATE_UNSYNC, // Waiting for BREAK condition
    dmx_STATE_RX,        // Receiving DMX Data
	dmx_STATE_FAILED,
}dmx_State_e;

/* == FILE SCOPE FUNCTIONS ================================================= */
static int8_t ConvertToSigned(uint8_t value);

/* == FILE STATIC ========================================================== */

// Receiver State
static dmx_State_e dmx_state = dmx_STATE_UNSYNC;

UART_HandleTypeDef* dmx_uart;

// Data ready flag
static uint8_t dmx_data_ready = 0;

static uint8_t dmx_buffer[DMX_MAX_RX_BUFFER_SIZE] = {0};

static uint8_t dmx_data[DMX_NUM_CHANNELS] = {0};

static void (*dmx_data_ready_callback) (DMX_Data*) = NULL; 

static osThreadId_t dmx_task_handle = NULL;

/* RX Complete Interrupt 
 * This will occur when we have received a full frame of DMX data. 
 * (Or a buffer of the same size) */
static void dmx_RxCompleteCallback (UART_HandleTypeDef* huart) {
    HAL_StatusTypeDef s = HAL_OK;

    /* Debug the case of under-reading */
    if (dmx_data_ready) {
        __BKPT();
    }
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
            memcpy(dmx_data, &dmx_buffer[DMX_CHANNEL_START], DMX_NUM_CHANNELS);
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
                memcpy(dmx_data, &dmx_buffer[DMX_CHANNEL_START], DMX_NUM_CHANNELS);
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
    if (status != HAL_OK){
    	__BKPT();
	}
    status = HAL_UART_RegisterCallback(huart, HAL_UART_ERROR_CB_ID, dmx_ErrorCallback);
    if (status != HAL_OK){
		__BKPT();
	}
}

static void dmx_debug_log(char* str, uint8_t len) {
    #if defined DMX_DEBUG_UART
        HAL_UART_Transmit_DMA(&DMX_DEBUG_UART, str, len);
    #endif
}

static void dmx_task(void* params) {
    (void) params;
    

    for (;;) {
        osDelay(10);
        switch (dmx_state) {
            case dmx_STATE_RESET:
                continue;

            case dmx_STATE_FAILED:
                static char dmx_failed_str[12] = "[dmx] fail\n";
                dmx_debug_log(dmx_failed_str, 12);
                continue;

            case dmx_STATE_UNSYNC:
            case dmx_STATE_RX:
                if (dmx_data_ready) {
                    /* Reset data ready */
                    dmx_data_ready = 0;

                    /* Run callback if enabled - Interface level data type 
                     * should be byte-wise equivalent of raw data. */
                    if (dmx_data_ready_callback) {
                        dmx_data_ready_callback((DMX_Data*)dmx_data);
                    }

                    /* Create our debug string - We just print chars directly
                     * very quick and dirty. */
					#define DEBUG_STRING_SIZE (5 + (DMX_NUM_CHANNELS*4))
                    static char debug_string[DEBUG_STRING_SIZE] = "[dmx] ";
                    char * debug_string_ptr = debug_string + 5;
                    memset(debug_string_ptr, ' ', DEBUG_STRING_SIZE-5);
                    for (int i = 0; i < DMX_NUM_CHANNELS; i++) {
                    	char temp[4] = {0};
                        itoa(((uint8_t*)dmx_data)[i], temp, 10);
                        memcpy(&debug_string_ptr[i*4], temp, strlen(temp));
                        debug_string_ptr[(i*4)+3] = ',';
                    }
                    debug_string[DEBUG_STRING_SIZE-1] = '\n';
                    dmx_debug_log(debug_string, (DEBUG_STRING_SIZE));
                }
                continue;
        }
    }
}

/* DMX Init 
 * Currently assumes both config and GPIO mapping have been taken care 
 * of by cube mx generater. 
 * TODO handle this internally or with a compilation flag. */
void DMX_Init (UART_HandleTypeDef* huart, void(*data_ready_callback)(DMX_Data*)) {

    /* file static vars*/
	dmx_uart = huart;
	dmx_state = dmx_STATE_RESET;
    dmx_data_ready = 0;
    dmx_data_ready_callback = data_ready_callback;

    /* Register our internal callbacks */
    dmx_RegisterCallbacks(huart);
    
    /* We should also do the hardware initialisation here but we rely on 
     * cube mx to generate it at the moment. */

    /* Create a task to receive and publish the data. */
    osThreadAttr_t thread_params = {
        .name = "dmx",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    dmx_task_handle = osThreadNew(dmx_task, NULL, &thread_params);

    /* We always start unsynced so just enable the receiver. This will mean
     * that a framing error is generated on the next BREAK condition and we
     * can start reception */
    HAL_UART_Receive_DMA(huart, dmx_buffer, DMX_MAX_RX_BUFFER_SIZE);
    dmx_state = dmx_STATE_UNSYNC;

}

float DMX_LinMap (uint8_t input, float min_output, float max_output) {
    // Calculate the ratio of the input value within the uint8_t range (0-255)
    float ratio = (float)input / 255.0;
    
    // Map the ratio to the custom output range
    float mapped_val = min_output + ratio * (max_output - min_output);
    return mapped_val;
}

float DMX_LogMap(uint8_t value, float min_output, float max_output) {
    // Ensure the input value is within the specified range
    if (value == 0) {
        value = 1; // Avoid logarithm of 0
    }

    // Calculate ratio
    float ratio = log10f((float)value) / log10f(255.0);

    // Calculate the logarithmic mapping
    float mapped_val = min_output + ratio * (max_output - min_output);
    return mapped_val;
}

float DMX_SymmetricLogMap(uint8_t value, float min_output, float max_output) {
    /* Convert to signed value. */
    int8_t signed_value = ConvertToSigned(value);
    
    // Handle the special case where value is 0
    if (signed_value == 0) {
        return 0.0f;
    }

    // Calculate the logarithmic mapping for non-zero values
    float log_value = log10f(fabsf((float)signed_value));
    float mapped_value = copysignf(log_value, signed_value) / log10f(128.0f) * (max_output - min_output);

    // Map the value to the output range
    return mapped_value;
}

float DMX_SymmetricExpMap(uint8_t value, float exponent_base, float max_output) {
    // Convert uint8_t to int8_t where 128 represents 0
    int8_t signed_value = ConvertToSigned(value);

    // Calculate the custom exponential mapping for non-zero values
    float exp_value = ((powf(exponent_base, fabsf((float)signed_value / 128.0f)) - 1) / (exponent_base - 1));
    float mapped_value = copysignf(exp_value, signed_value) * max_output;

    // Map the value to the output range
    return mapped_value;
}

float DMX_ExpMap(uint8_t value, float exponent_base, float min_output, float max_output) {
    // Calculate the exponential mapping for the given input value (0 to 255)
    float exp_value = ((powf(exponent_base, (float)value / 255.0f) - 1) / (exponent_base - 1));

    // Map the value to the output range
    float mapped_value = min_output + (exp_value * (max_output - min_output));

    return mapped_value;
}


static int8_t ConvertToSigned(uint8_t value) {
    // Subtract 128 to map 128 to 0
    return (int8_t)(value - 128);
}

