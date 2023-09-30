/* == INCLUDES ============================================================= */

#include "stm32f7xx_hal.h"

#include "stdint.h"

/* == CONFIGURATION ======================================================== */

#define AUDIO_DEBUG_UART (huart5)
#if defined AUDIO_DEBUG_UART
extern UART_HandleTypeDef AUDIO_DEBUG_UART;
#else
#define AUDIO_DEBUG_UART (NULL)
#endif

/* Number of Frequency Bands */
#define AUDIO_NUM_FREQ_BANDS        (20)

/* Buffer Size */
#define AUDIO_BUFFER_SAMPLES_PER_CHANNEL	1024U

/* FFT Size 
 * This should be greater than the audio samples per channel such that we get
 * a decent amount of padding. However it must align to a power of 2. */
#define AUDIO_FFT_SIZE 2048U

/* System Clock Rate 
 * provide a */
#define AUDIO_CORE_CLOCK_Hz SystemCoreClock
extern uint32_t SystemCoreClock;

/* == INTERFACE FUNCTIONS ================================================== */

void AUDIO_Init(SAI_HandleTypeDef* input_sai, SAI_HandleTypeDef* output_sai);

void AUDIO_Start (void);
