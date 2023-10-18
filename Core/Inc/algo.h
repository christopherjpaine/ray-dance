/* == INCLUDES ============================================================= */

#include "stm32f7xx_hal.h"
#define ARM_MATH_CM7
#include "arm_math.h"
#include "stdint.h"

/* == CONFIGURATION ======================================================== */

typedef enum ALGO_PrintType_e {
    ALGO_PRINT_TYPE_FFT_PROPERTIES = 0,
    ALGO_PRINT_TYPE_FREQ_BANDS,
    ALGO_PRINT_TYPE_PIPELINE_PROPERTIES,
    ALGO_PRINT_TYPE_BAND_MAGS,
    ALGO_PRINT_TYPE_COEFFS

}ALGO_PrintType;

typedef struct ALGO_FreqBand_s{
    float start_freq;
    float end_freq;
}ALGO_FreqBand;

typedef struct ALGO_SmoothingFilter_s{
    /* CMSIS Precision Biquad Instance */
    arm_biquad_cascade_df2T_instance_f32 inst;
    /* States - 2 for a Second Order Filter Using Direct Form 2 */
    float32_t states[2];
}ALGO_SmoothingFilter;

typedef struct ALGO_SmoothingCoeffs_s{
    float32_t b0;
    float32_t b1;
    float32_t b2;
    float32_t a1;
    float32_t a2; 
}ALGO_SmoothingCoeffs;

typedef struct ALGO_FreqAnalysis_s {
    uint32_t num_bands;
    float min_freq;
    float max_freq;
    ALGO_FreqBand* freq_bands;
    uint32_t sampling_rate_Hz;
    uint32_t buffer_size;
    struct {
        float frame_rate_Hz;
    }calc;
    struct {
        float* band_mags_f32;
        float* smoothed_band_mags_f32;
        float* band_mag_results_f32;
        ALGO_SmoothingFilter* smoothing_filters;
    }data;
    struct {
        float gain_dB;
        float band_compensation; // 0.0f to 1.0f
        float contrast; // +/-1.0f
        ALGO_SmoothingCoeffs smoothing_coeffs; // Calculated using ALGO_CalculateSmoothingCoeffs
    }dynamic;
}ALGO_FreqAnalysis;

typedef struct ALGO_FftProperties_s {
    struct {
        uint32_t sampling_rate;
        uint32_t samples_per_channel_per_buffer;
        uint32_t fft_size; // including any padding
    }init;
    struct {
        uint32_t num_bins; // fft_size/2 (as we discard the bin at exactly nyquist.)
        float freq_resolution;  // sampling_rate/samples_per_buffer
        float precision;   // AKA bin range. sampling_rate/ff_size
    }out;
}ALGO_FftProperties;

typedef struct ALGO_PipelineProperties_s {
    struct {
        uint32_t sampling_rate;
        uint32_t samples_per_channel_per_buffer;
        uint32_t sys_clock_rate_Hz;
    }init;
    // data in = pipeline cpu cycles.
    struct {
        uint32_t last_pipeline_complete_cycles;
        // todo smoothing
    } priv;
    struct {
        float buffer_latency_ms;
        float processing_latency_ms;
        float total_latency_ms;
        float processing_cpu_usage_percent;
    }out;
}ALGO_PipelineProperties;

/* == INTERFACE FUNCTIONS ================================================== */

/** 
 * @brief Initialise a pipeline properties object
 * @param pipeline pointer to structure describing the pipeline. This object
 * allows you to run after each pipeline run to track latency and processor load.
*/
void ALGO_InitPipelineProperties (ALGO_PipelineProperties* pipeline);

/** @brief Caculate the properties of an FFT */
void ALGO_InitFftProperties (ALGO_FftProperties* fft);

/**
 * @brief Initialise a frequency analysis module.
 * @param freq_analysis pointer to structure describing properties of analysis
 * and providing pointers to the required memory.
 * @details You will then be able to use this object to perform frequency 
 * analysis given a buffer containing the raw magnitude data from an fft.
 * In this context "analysis" means getting the magnitude of each band. 
 */
void ALGO_InitFreqAnalysis (ALGO_FreqAnalysis* freq_analysis);

/** 
 * @brief performs frequency analysis based on a buffer of magnitudes from 
 * an fft.
 * @param freq_analysis [in] frequency analysis object
 * @param fft [in] Properties of the fft that produce the buffer
 * @param mag_buf [in] Buffer of magnitudes to perform analysis one per bin
 * @return Pointer to a buffer of band magnitudes produced by analysis. The 
 * address returned allows easy access to the results which are actually
 * stored as part of analysis object.
*/
float* ALGO_RunFreqAnalysis (ALGO_FreqAnalysis* freq_analysis,
                           ALGO_FftProperties* fft,
                           float* mag_buf);

/** 
 * @brief Calculate smoothing filter coeffs.
 * 
 * Given a coefficients buffer and a cutoff frequency in Hz this function 
 * calculates the smoothing coeffs. It is a separate function so that you
 * can calculate the coeffs outside of the main processing loop and then
 * upload them "instantly" 
 */
void ALGO_CalculateSmoothingCoeffs (ALGO_FreqAnalysis* analysis,
                                    float cutoff_Hz, 
                                    ALGO_SmoothingCoeffs* dest);

void ALGO_Print(ALGO_PrintType type, void* data, UART_HandleTypeDef* huart);

