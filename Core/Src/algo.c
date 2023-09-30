/* == INCLUDES ============================================================= */

#include "algo.h"

#define ARM_MATH_CM7
#include "arm_math.h"

#include <stdint.h>

/* == CONFIGURATION ======================================================== */

#define PRINT_BUFFER_SIZE   256

/* == DEFINES ============================================================== */

/* == TYPES ================================================================ */


/* == FILE STATIC FUNCTIONS ================================================ */

/* == FILE STATIC VARIABLES ================================================ */

static char algo_print_buffer[PRINT_BUFFER_SIZE];


/* == INTERFACE FUNCTIONS ================================================== */

void ALGO_CalculateFrequencyRanges (ALGO_FreqRange* range_array, float min_freq, float max_freq, uint32_t num_ranges) {

    /* Calculate the logarithmic scale factor */
    double s = (log(max_freq) - log(min_freq)) / num_ranges;

    /* Loop through the required number of ranges */
    int i;
    for (i = 0; i < num_ranges; i++) {
        /* Calculate the start and end point of each group according to our
         * logarithmic function. */
        range_array[i].start_freq = min_freq * exp(i * s);
        range_array[i].end_freq = min_freq * exp((i + 1) * s);
    }
}

void ALGO_CalculateFftProperties (ALGO_FftProperties* fft_properties) {
    
    /* Calculate the number of bins. 
     * This would normally be (num_samples_in/2) + 1 but we discard the highest
     * freq bin to keep our buffer sizes radix 2. */
    fft_properties->out.num_bins = fft_properties->in.fft_size/2;

    /* Calculate Frequency Resolution. 
     * This is the true resolution of the fft, which discards any padding 
     * present in the input. */
    fft_properties->out.freq_resolution = fft_properties->in.sampling_rate / fft_properties->in.samples_per_channel_per_buffer;

    /* Calculate the Frequency Precision. 
     * This is the frequency spacing of the resulting bins once you account for
     * the full input buffer including padding. Can be referred to as bin-range. */
    fft_properties->out.freq_precision = fft_properties->in.sampling_rate / fft_properties->in.fft_size;

}

void ALGO_Print(ALGO_PrintType type, void* data, UART_HandleTypeDef* huart) {
    if (huart == NULL) {
        return;
    }

    int written = 0;

    switch (type) {
        case ALGO_PRINT_TYPE_FFT_PROPERTIES:
            ALGO_FftProperties* fft_properties = (ALGO_FftProperties*)data;
            written = snprintf_(algo_print_buffer, PRINT_BUFFER_SIZE,
                "Algo: FFT Properties \n"
                "\t fs: %dHz\n"
                "\t bins: %d\n"
                "\t resolution: %fHz\n"
                "\t precision: %fHz\n",
                fft_properties->in.sampling_rate,
                fft_properties->out.num_bins,
                fft_properties->out.freq_resolution,
                fft_properties->out.freq_precision);
            break;
    }

    if (written < 0) {
        return;
    } else if (written >= PRINT_BUFFER_SIZE) {
        char term[] = "...\n";
        memcpy(&algo_print_buffer[PRINT_BUFFER_SIZE-4],term,4);
        written = PRINT_BUFFER_SIZE;
    } else {
        int pos = strlen(algo_print_buffer);
        algo_print_buffer[pos] = '\n';
    }

    HAL_StatusTypeDef s = HAL_UART_Transmit_DMA(huart, algo_print_buffer, written);
    if (HAL_OK != s) {
    	__BKPT();
    }
    
}
