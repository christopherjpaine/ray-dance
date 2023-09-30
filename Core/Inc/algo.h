/* == INCLUDES ============================================================= */

#include "stm32f7xx_hal.h"
#include "stdint.h"

/* == CONFIGURATION ======================================================== */

// #define ALGO_DEBUG_UART (huart5)
#if defined ALGO_DEBUG_UART
extern UART_HandleTypeDef ALGO_DEBUG_UART;
#endif

typedef enum ALGO_PrintType_e {
    ALGO_PRINT_TYPE_FFT_PROPERTIES = 0,
    ALGO_PRINT_TYPE_FREQ_BANDS,
    ALGO_PRINT_TYPE_PIPELINE_PROPERTIES,

}ALGO_PrintType;

typedef struct ALGO_FreqBand_s{
    float start_freq;
    float end_freq;
}ALGO_FreqBand;

typedef struct ALGO_FreqAnalysis_s {
    uint32_t num_bands;
    float min_freq;
    float max_freq;
    ALGO_FreqBand* freq_bands;
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
        float freq_precision;   // AKA bin range. sampling_rate/ff_size
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
 * @brief Initialise a frequency analysis module.
 * @param freq_analysis pointer to structure describing properties of analysis
 * and providing pointers to the required memory.
 * @details You will then be able to use this object to perform frequency 
 * analysis given a buffer containing the raw magnitude data from an fft.
 * In this context "analysis" means getting the magnitude of each band. 
 */
void ALGO_InitFreqAnalysis (ALGO_FreqAnalysis* freq_analysis);

/** 
 * @brief Initialise a pipeline properties object
 * @param pipeline pointer to structure describing the pipeline. This object
 * allows you to run after each pipeline run to track latency and processor load.
*/
void ALGO_InitPipelineProperties (ALGO_PipelineProperties* pipeline);

/** @brief Caculate the properties of an FFT */
void ALGO_InitFftProperties (ALGO_FftProperties* fft);


/** 
 * @brief performs frequency analysis based on a buffer of magnitudes from 
 * an fft.
 * @param freq_analysis [in] frequency analysis object
 * @param fft [in] Properties of the fft that produce the buffer
 * @param mag_buf [in] Buffer of magnitudes to perform analysis one per bin
 * @param results [out] Buffer of magnitudes produced by analysis, one per band
*/
void ALGO_RunFreqAnalysis (ALGO_FreqAnalysis* freq_analysis,
                           ALGO_FftProperties* fft,
                           float* mag_buf,
                           float* results);

void ALGO_Print(ALGO_PrintType type, void* data, UART_HandleTypeDef* huart);

