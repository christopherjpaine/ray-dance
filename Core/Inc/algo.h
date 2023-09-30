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
    ALGO_PRINT_TYPE_PIPELINE_PROPERTIES = 1,
}ALGO_PrintType;

typedef struct ALGO_FreqRange_s{
    float start_freq;
    float end_freq;
}ALGO_FreqRange;

typedef struct ALGO_FftProperties_s {
    struct {
        uint32_t sampling_rate;
        uint32_t samples_per_channel_per_buffer;
        uint32_t fft_size; // including any padding
    }in;
    struct {
        uint32_t num_bins; // fft_size/2 (as we discard the bin at exactly nyquist.)
        float freq_resolution;  // sampling_rate/samples_per_buffer
        float freq_precision;   // AKA bin range. sampling_rate/ff_size
    }out;
}ALGO_FftProperties;

typedef struct ALGO_PipelineProperties {
    struct {
        uint32_t sampling_rate;
        uint32_t samples_per_channel_per_buffer;
        uint32_t sys_clock_rate_Hz;
        uint32_t processing_sys_clock_cycles;
    }in;
    struct {
        float buffer_latency_ms;
        float processing_latency_ms;
        float total_latency_ms;
        float processing_cpu_usage_percent;
    }out;
};

/* == INTERFACE FUNCTIONS ================================================== */


/**
 * @brief Given a frequency range and a required number of bands, return an array of 
 * bands that have logarithmically equal spacing. */
void ALGO_CalculateFrequencyRanges (ALGO_FreqRange* range_array, float min_freq, float max_freq, uint32_t num_ranges);

/** @brief Caculate the properties of an FFT */
void ALGO_CalculateFftProperties (ALGO_FftProperties* fft_properties);

void ALGO_Print(ALGO_PrintType type, void* data, UART_HandleTypeDef* huart);

