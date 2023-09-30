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

static void CalculateFreqRanges (ALGO_FreqBand* band_array, float min_freq, float max_freq, uint32_t num_ranges);

/* == FILE STATIC VARIABLES ================================================ */

static char algo_print_buffer[PRINT_BUFFER_SIZE];


/* == INTERFACE FUNCTIONS ================================================== */

void ALGO_InitFreqAnalysis (ALGO_FreqAnalysis* freq_analysis) {
    CalculateFreqRanges (freq_analysis->freq_bands, freq_analysis->min_freq, 
                         freq_analysis->max_freq, freq_analysis->num_bands);
}

void ALGO_InitPipelineProperties (ALGO_PipelineProperties* pipeline) {

    /* Buffer latency is just the amount of audio we have to buffer 
     * before we begin processing. = num samples * sample period. */
    pipeline->out.buffer_latency_ms = 
        pipeline->init.samples_per_channel_per_buffer / pipeline->init.sampling_rate;
    
    /* Processing related parameters can only be populated after we have called
     * the pipeline properties run function multiple times. */
    pipeline->out.processing_latency_ms = 0.0f;
    pipeline->out.total_latency_ms = pipeline->out.buffer_latency_ms;
    pipeline->out.processing_cpu_usage_percent;

}

void ALGO_InitFftProperties (ALGO_FftProperties* fft) {
    
    /* Calculate the number of bins. 
     * This would normally be (num_samples_in/2) + 1 but we discard the highest
     * freq bin to keep our buffer sizes radix 2. */
    fft->out.num_bins = fft->init.fft_size/2;

    /* Calculate Frequency Resolution. 
     * This is the true resolution of the fft, which discards any padding 
     * present in the input. */
    fft->out.freq_resolution = fft->init.sampling_rate / fft->init.samples_per_channel_per_buffer;

    /* Calculate the Frequency Precision. 
     * This is the frequency spacing of the resulting bins once you account for
     * the full input buffer including padding. Can be referred to as bin-range. */
    fft->out.freq_precision = fft->init.sampling_rate / fft->init.fft_size;

}

void ALGO_RunFreqAnalysis (ALGO_FreqAnalysis* freq_analysis,
                           ALGO_FftProperties* fft,
                           float* mag_buf,
                           float* results) {

    float bin_freq = 0.0f;
    uint32_t band_index = 0;
    uint32_t bin_count = 0;

    /* Reset results buffer. */
    memset(results, 0, sizeof(float)*freq_analysis->num_bands);

    for (int i = 0; i < fft->out.num_bins; i++){
        ALGO_FreqBand* band = &freq_analysis->freq_bands[band_index];
        /* If bin freq is above current band, check there is a higher band... */
        if (bin_freq > band->end_freq) {

            /* ... If it exists then average down the current band and then start
             * accumulating in the next one after resetting the bin count. */
            if ((band_index+1) < freq_analysis->num_bands) {
                results[band_index] = results[band_index] / bin_count;
                bin_count = 0;
                band_index++;
                results[band_index] += mag_buf[i];
            /* ... If it does not exist then average down the final band and
             * exit the loop. */
            } else {
                results[band_index] = results[band_index] / bin_count;
                break;
            }   
        }
        /* If the frequency is within the current band then just accumulate. */
        else if (bin_freq >= band->start_freq 
           && bin_freq <= band->end_freq) {
            results[band_index] += mag_buf[i];  
        }
        
        /* Add bin range on each loop and update bin count. */
        bin_freq += fft->out.freq_precision;
        bin_count++;
    }

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
                fft_properties->init.sampling_rate,
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

static void CalculateFreqRanges (ALGO_FreqBand* band_array, float min_freq, float max_freq, uint32_t num_ranges) {

    /* Calculate the logarithmic scale factor */
    double s = (log(max_freq) - log(min_freq)) / num_ranges;

    /* Loop through the required number of ranges */
    int i;
    for (i = 0; i < num_ranges; i++) {
        /* Calculate the start and end point of each group according to our
         * logarithmic function. */
        band_array[i].start_freq = min_freq * exp(i * s);
        band_array[i].end_freq = min_freq * exp((i + 1) * s);
    }
}
