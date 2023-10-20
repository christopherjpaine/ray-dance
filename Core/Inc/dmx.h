/* == INCLUDES ============================================================= */

#include "stm32f7xx_hal.h"

/* == CONFIGURATION ======================================================== */

// #define DMX_DEBUG_UART (huart5)
#if defined DMX_DEBUG_UART
extern UART_HandleTypeDef DMX_DEBUG_UART;
#endif

/* == TYPES ================================================================ */

typedef struct DMX_DataRGB_s {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
}DMX_DataRGB;

typedef struct DMX_DataAlgo_s{
    uint8_t gain;
    uint8_t band_compensation;
    uint8_t contrast;
    uint8_t smoothing;
    uint8_t animation_speed;
}DMX_DataAlgo;

/* Representation of the dmx data that can be directly typecast onto the 
 * received data. */
typedef DMX_DataAlgo DMX_Data;

/* == INTERFACE FUNCTIONS ================================================== */

/* DMX Init */
void DMX_Init (UART_HandleTypeDef* huart, void (*data_ready_callback) (DMX_Data*));

/* DMX Mapping Helper */
float DMX_LinMap (uint8_t input, float min_output, float max_output);

float DMX_LogMap(uint8_t value, float min_output, float max_output);