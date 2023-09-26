/* == INCLUDES ============================================================= */

#include "stm32f7xx_hal.h"

/* == CONFIGURATION ======================================================== */

#define DMX_DEBUG_UART (huart5)
#if defined DMX_DEBUG_UART
extern UART_HandleTypeDef DMX_DEBUG_UART;
#endif

/* == TYPES ================================================================ */

/* Representation of the dmx data that can be directly typecast onto the 
 * received data. */
typedef struct DMX_Data_s {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
}DMX_Data;

/* == INTERFACE FUNCTIONS ================================================== */

/* DMX Init */
void DMX_Init (UART_HandleTypeDef* huart, void (*data_ready_callback) (DMX_Data*));
