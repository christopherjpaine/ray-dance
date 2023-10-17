/* == INCLUDES ============================================================= */

#include "algo.h"

#define ARM_MATH_CM7
#include "arm_math.h"

#include <stdint.h>
#include <math.h>

#include "mpaland/printf.h"

/* == CONFIGURATION ======================================================== */

#define algo_MAX_COMPENSATION_GAIN  7.0f

#define PRINT_BUFFER_SIZE   256

/* == DEFINES ============================================================== */

/* == TYPES ================================================================ */


/* == FILE STATIC FUNCTIONS ================================================ */

/* Calculating */
static void CalculateFreqRanges (ALGO_FreqBand* band_array, float min_freq, float max_freq, uint32_t num_ranges);
static void InitialiseSmoothingFilters (ALGO_FreqAnalysis* analysis);

/* Processing */
static void AccumulateBands(ALGO_FreqAnalysis* analysis, ALGO_FftProperties* fft, float* bin_mags);
static float BandingCompensationFactor (ALGO_FreqAnalysis* analysis, int band_index);
static void ApplySmoothing (ALGO_FreqAnalysis* analysis);
static inline float ApplyGain(float input, float gain_dB);
static inline float CalculateLogarithmicMagnitude(float linear_magnitude, float contrast);
static inline float ApplyLimit (float input);

/* == FILE STATIC VARIABLES ================================================ */

static char algo_print_buffer[PRINT_BUFFER_SIZE];

/* == INTERFACE FUNCTIONS ================================================== */

void ALGO_InitPipelineProperties (ALGO_PipelineProperties* pipeline) {

    /* Buffer latency is just the amount of audio we have to buffer 
     * before we begin processing. = num samples * sample period. */
    pipeline->out.buffer_latency_ms = 
        pipeline->init.samples_per_channel_per_buffer / pipeline->init.sampling_rate;
    
    /* Processing related parameters can only be populated after we have called
     * the pipeline properties run function multiple times. */
    pipeline->out.processing_latency_ms = 0.0f;
    pipeline->out.total_latency_ms = pipeline->out.buffer_latency_ms;
    // pipeline->out.processing_cpu_usage_percent;

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
    fft->out.precision = fft->init.sampling_rate / fft->init.fft_size;

}
void ALGO_InitFreqAnalysis (ALGO_FreqAnalysis* freq_analysis) {
    CalculateFreqRanges (freq_analysis->freq_bands, freq_analysis->min_freq, 
                         freq_analysis->max_freq, freq_analysis->num_bands);
    InitialiseSmoothingFilters(freq_analysis);
    freq_analysis->calc.frame_rate_Hz = freq_analysis->sampling_rate_Hz/freq_analysis->buffer_size;
}

float* ALGO_RunFreqAnalysis (ALGO_FreqAnalysis* analysis,
                           ALGO_FftProperties* fft,
                           float* bin_mags) {

    AccumulateBands(analysis, fft, bin_mags);

#if 1 // smoothing off
    float* results = analysis->data.band_mags_f32;
#else // smoothing on
    ApplySmoothing(analysis);
    float* results = analysis->data.smoothed_band_mags_f32;
#endif

    /* TODO Apply smoothing filters */

    /* Apply gain and contrast parameters to band magnitudes. The contrast 
     * function allow you to adjust the mapping from lin->log such that we
     * exagerrate or minimize the variance */
    for (int i = 0; i < analysis->num_bands; i++) {
        results[i] = ApplyGain(results[i], analysis->dynamic.gain_dB);
        results[i] = ApplyLimit(results[i]);
        // results[i] = CalculateLogarithmicMagnitude(results[i], analysis->dynamic.gain_dB);
    }

    return results;

}

void ALGO_CalculateSmoothingCoeffs (ALGO_FreqAnalysis* analysis, float cutoff_Hz, ALGO_SmoothingCoeffs* dest) {

    /* Setup to calculate Butterworth Lowpass at fc */
    float normalized_fc = cutoff_Hz/analysis->calc.frame_rate_Hz;
    const float butterworthQ = 0.70710678118;
    float w0 = 2.0 * M_PI * normalized_fc;
    float alpha = arm_sin_f32(w0) / (2.0 * butterworthQ);

    /* Compute Coefficients */
    dest->b0 = (1.0 - arm_cos_f32(w0)) / 2.0;
    dest->b1 = 1.0 - arm_cos_f32(w0);
    dest->b2 = (1.0 - arm_cos_f32(w0)) / 2.0;
    float a0 = 1.0 + alpha;
    dest->a1 = -2.0 * arm_cos_f32(w0);
    dest->a2 = 1.0 - alpha;

    /* Normalise such that a0 = 1. Taking into consideration that the 
     * a1 and a2 coefficients must be negated to work in the direct form 2
     * CMSIS implementation. */
    float normFactor = 1.0 / a0;
    dest->b0 *= normFactor;
    dest->b1 *= normFactor;
    dest->b2 *= normFactor;
    dest->a1 *= -normFactor;
    dest->a2 *= -normFactor;

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
                "[algo_info] FFT Properties \n"
                "\t fs: %uHz\n"
                "\t bins: %u\n"
                "\t resolution: %fHz\n"
                "\t precision: %fHz\n",
                fft_properties->init.sampling_rate,
                fft_properties->out.num_bins,
                fft_properties->out.freq_resolution,
                fft_properties->out.precision);
            break;
        
        case ALGO_PRINT_TYPE_BAND_MAGS:
            ALGO_FreqAnalysis* freq_analysis = (ALGO_FreqAnalysis*)data;
            written = snprintf_(algo_print_buffer, PRINT_BUFFER_SIZE, "[algo]");
            for (int i = 0; i < freq_analysis->num_bands; ++i) {
                int offset = snprintf_(algo_print_buffer + written,
                                       PRINT_BUFFER_SIZE - written,
                                       " %.3f",
                                       freq_analysis->data.band_mags_f32[i]);
                written += offset;
                if (offset < 0 || written >= PRINT_BUFFER_SIZE) {
                    // Break out on error or overflow
                    break;
                }
            }
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
        written+=1;
    }

    HAL_StatusTypeDef s = HAL_UART_Transmit_DMA(huart, (uint8_t*)algo_print_buffer, written);
    if (HAL_OK != s) {
    	__BKPT();
    }
    
}

/**
 * @brief Accumulates band magnitudes from FFT bin magnitudes.
 *
 * This function accumulates band magnitudes from FFT output bins considering the specified frequency bands 
 * and compensation factors.
 *
 * @param analysis Pointer to the ALGO_FreqAnalysis instance.
 * @param fft Pointer to the ALGO_FftProperties instance that produced the bin_mags.
 * @param bin_mags Array of FFT bin magnitudes.
 */
static void AccumulateBands(ALGO_FreqAnalysis* analysis, ALGO_FftProperties* fft, float* bin_mags) {
    int num_bins = fft->out.num_bins;
    int num_bands = analysis->num_bands;
    float* results = analysis->data.band_mags_f32;
    results[0] = 0.0f;
    float bin_freq = 0;
    int band_index = 0;
    int bin_count = 0;
    float band_comp = BandingCompensationFactor(analysis, band_index);

    for (int i = 0; i < num_bins; ++i) {
        float band_start_freq = analysis->freq_bands[band_index].start_freq;
        float band_end_freq = analysis->freq_bands[band_index].end_freq;

        // If the frequency is within the current band, accumulate
        if (band_start_freq <= bin_freq && bin_freq <= band_end_freq) {
            results[band_index] += bin_mags[i] * band_comp;
        }

        // If bin freq is above current band, average the current band and set up the next one
        else if (bin_freq > band_end_freq) {
            results[band_index] /= bin_count;
            bin_count = 0;
            if (band_index + 1 < num_bands) {
                band_index++;
                band_comp = BandingCompensationFactor(analysis, band_index);  // Recalculate for the next band
                results[band_index] = bin_mags[i] * band_comp;
            } else {
                break;
            }
        }

        // Add bin range on each loop
        bin_freq += fft->out.precision;
        ++bin_count;
    }

    // Catch the case that the freq_ranges is greater than that of the fft
    if (bin_freq < analysis->freq_bands[num_bands - 1].end_freq) {
        results[band_index] /= bin_count;
    }
}

/**
 * @brief Calculate the bands compensation factor based on the selected compensation gain.
 *
 * The compensation factor is calculated using the generic exponential form: y = a * exp(b * x),
 * where x is the band's normalized value and b is the compensation gain.
 *
 * @param analysis Pointer to the ALGO_FreqAnalysis instance.
 * @param band_index Index of the band for which the compensation factor is calculated.
 * @return The calculated compensation factor for the specified band.
 */
static float BandingCompensationFactor (ALGO_FreqAnalysis* analysis, int band_index) {
    float b = analysis->dynamic.band_compensation * algo_MAX_COMPENSATION_GAIN;
    float x = (float)band_index / analysis->num_bands;
    return expf(b * x);
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

static void InitialiseSmoothingFilters (ALGO_FreqAnalysis* analysis) {
    /* Calculate filter params for arbitrary cutoff (20Hz will be close to off)*/
    ALGO_CalculateSmoothingCoeffs(analysis, 20, &analysis->dynamic.smoothing_coeffs);

    /* For each band initialise a smoother and point it at the coeffs in our 
    * analysis dynamic properties. */
    for (int i = 0; i < analysis->num_bands; i++) {
        arm_biquad_cascade_df2T_init_f32(&analysis->data.smoothers[i].inst,
                                         1, // Single stage
                                         &analysis->dynamic.smoothing_coeffs,
                                         &analysis->data.smoothers[i].states);
    }
}

static void ApplySmoothing (ALGO_FreqAnalysis* analysis) {
    float* sources = analysis->data.band_mags_f32;
    float* results = analysis->data.smoothed_band_mags_f32;
    ALGO_SmoothingFilter* filters = analysis->data.smoothing_filters;
    
    /* Run the filter on a single sample for each band. */
    for (int i = 0; i < analysis->num_bands; i++) {
        arm_biquad_cascade_df2T_f32(&filters[i].inst,
                                    &sources[i],
                                    &results[i],
                                    1);
    }
    
}

static inline float ApplyGain(float input, float gain_dB) {
    // Convert dB gain to linear scale and apply gain to the input
    float output = input * pow(10, gain_dB / 20.0);
    return output;
}

/**
 * @brief Calculates the logarithmic magnitude from a linear magnitude for a 
 * given linear contrast.
 *
 * @param linear_magnitude The input linear magnitude between 0 and 1.
 * @param scale_factor The scale factor applied to the calculated logarithmic magnitude.
 * @param contrast The contrast factor (+/-1.0f range) used in the exponential
 * transformation. Increasing contrast will increase the difference between 
 * high and low magnitudes and vice-versa.
 * @return The calculated logarithmic magnitude. 
 */
static inline float CalculateLogarithmicMagnitude(float linear_magnitude, float contrast) {
    // Catch invalid input (clipping)
    if (linear_magnitude > 1.0f || linear_magnitude < 0.0f) {
        __BKPT();
    }
    
    // Handle the case where linear_magnitude is zero
    if (linear_magnitude == 0) {
        return 0;
    }
    
    // Linear value for contrast is transformed onto a 2^x exponential scale...
    float contrast_sensitivity = 1;
    float transformed_exponent = powf(2, contrast * contrast_sensitivity);
    
    // ..which is then used to define the exponential mapping of the linear
    // input onto an exponential scale giving us the resulting log-magnitude
    float logarithmic_magnitude = powf(linear_magnitude, transformed_exponent);
    
    return logarithmic_magnitude;
}

static inline float ApplyLimit (float input) {
    return fminf(1.0f, input);
}
