/* == INCLUDES ============================================================= */

#include "stm32f7xx_hal.h"

/* == CONFIGURATION ======================================================== */

// #define AUDIO_DEBUG_UART (huart5)
#if defined AUDIO_DEBUG_UART
extern UART_HandleTypeDef AUDIO_DEBUG_UART;
#endif

/* == INTERFACE FUNCTIONS ================================================== */

void AUDIO_Start (SAI_HandleTypeDef* audio_in_sai, SAI_HandleTypeDef* audio_out_sai);