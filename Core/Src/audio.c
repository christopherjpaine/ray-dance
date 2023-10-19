/* == INCLUDES ============================================================= */

#include "audio.h"

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

#include "wm8994.h"
#include "algo.h"
#include "led.h"
#include "animate.h"

#define ARM_MATH_CM7
#include "arm_math.h"

#include <stdint.h>

/* == CONFIGURATION ======================================================== */

/* The minimum frequency in the spectrum we wish to analyse */
#define audio_MINIMUM_ANALYSIS_FREQUENCY    20.0f

/* The maximum frequency in the spectrum we wish to analyse.
 * Could be better visually to have finer detail in mid range */
#define audio_MAXIMUM_ANALYSIS_FREQUENCY    20000.0f

/* == DEFINES ============================================================== */

/* Sample Rate 
 * Changing this requires changes to the */
#define audio_SAMPLE_RATE_Hz        SAI_AUDIO_FREQUENCY_48K

/* I2C Address of codec */
#define audio_I2C_ADDRESS                ((uint16_t)0x34)

/* Audio Channels 
 * Normally 2 for Left and Right interleaved. */
#define audio_BUFFER_CHANNELS	2

/* Size of the audio buffer in total samples */
#define audio_BUFFER_SIZE AUDIO_BUFFER_SAMPLES_PER_CHANNEL * audio_BUFFER_CHANNELS

/* Audio Buffer Total Length 
 * n samples per channel, multiplied by 2 such that both of the ping-pong 
 * buffers are equal. */
#define audio_BUFFER_RAW_MEMORY	audio_BUFFER_SIZE * 2

/* FFT Padding */
#define audio_FFT_PADDING   (AUDIO_FFT_SIZE - AUDIO_BUFFER_SAMPLES_PER_CHANNEL)

/* Audio FFT Bins 
 * Number of bins is half that of the fft size */
#define audio_FFT_NUM_BINS      AUDIO_FFT_SIZE / 2

/* == TYPES ================================================================ */

typedef enum audio_Event_e {
    audio_EVENT_NONE,
    audio_EVENT_BUFFER_A_READY,
    audio_EVENT_BUFFER_B_READY,
    audio_EVENT_PARAMS_UPDATE,
}audio_Event;

typedef enum audio_State_e {
    audio_STATE_AWAITING_BUFFER_A,
    audio_STATE_AWAITING_BUFFER_B
}audio_State;

/* == FILE STATIC FUNCTIONS ================================================ */

static void audio_InitCodec (uint32_t sampleRate, uint32_t volumePercent);

static void audio_RxHalfCompleteCallback(SAI_HandleTypeDef* hsai);
static void audio_RxCompleteCallback(SAI_HandleTypeDef* hsai);

static void audio_task(void* params);

static void audio_Algorithm (int16_t *audio_lr);

static void audio_DebugResults (float *mag_f32, UART_HandleTypeDef* huart);

/* == FILE STATIC VARIABLES ================================================ */

static int16_t audio_raw_buffer[audio_BUFFER_RAW_MEMORY] = {0};

static osThreadId_t audio_task_handle = NULL;
static osMessageQueueId_t audio_queue_handle = NULL;
static SAI_HandleTypeDef* audio_input_sai = NULL;
static SAI_HandleTypeDef* audio_output_sai = NULL; 

audio_State audio_state = audio_STATE_AWAITING_BUFFER_A;

static uint8_t audio_params_pending_flag = 0;
ALGO_DynamicParams audio_algo_params_pending = {0};

arm_rfft_fast_instance_f32 audio_fft_instance;

ALGO_FftProperties audio_fft_properties = {0};

/* Algo Frequency Analysis Memory */
ALGO_FreqAnalysis audio_freq_analysis = {0};
static float audio_band_mags_f32[AUDIO_NUM_FREQ_BANDS] = {0.0f};
static float audio_smoothed_band_mags_f32[AUDIO_NUM_FREQ_BANDS] = {0.0f};
static ALGO_SmoothingFilter audio_smoothing_filters[AUDIO_NUM_FREQ_BANDS] = {0};

/* Animation Instance */
static ANIMATE_Instance audio_animate_instance;

/* == INTERFACE FUNCTIONS ================================================== */

void AUDIO_Init (SAI_HandleTypeDef* input_sai, SAI_HandleTypeDef* output_sai) {

    /* Store sai handles */
    audio_input_sai = input_sai;
    audio_output_sai = output_sai;

    /* TODO Hardware Init that is currently handled by cube mx IOC. */

    /* Start the SAI Output Block to generate MCLK */
    __HAL_SAI_ENABLE(audio_output_sai);

    /* Set the callbacks */
    HAL_SAI_RegisterCallback(audio_input_sai, HAL_SAI_RX_HALFCOMPLETE_CB_ID, audio_RxHalfCompleteCallback);
    HAL_SAI_RegisterCallback(audio_input_sai, HAL_SAI_RX_COMPLETE_CB_ID, audio_RxCompleteCallback);

}

void AUDIO_Start (void) {

	/* Init codec once RTOS is running */
	audio_InitCodec(audio_SAMPLE_RATE_Hz, 100);

    /* Unmute the Audio CODEC 
     * Note that the API says it needs buffer and size, this is not the case
     * so we send NULL and 0. */
    if (wm8994_drv.Play(audio_I2C_ADDRESS, NULL, 0) != 0)
    {
        __BKPT();
    }

    /* Create the audio task and it's event queue */
    osThreadAttr_t thread_params = {
        .name = "audio",
        .stack_size = 512 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    audio_task_handle = osThreadNew(audio_task, NULL, &thread_params);
    osMessageQueueAttr_t queue_params = {
        .name = "audio",
    };
    audio_queue_handle = osMessageQueueNew (3, sizeof(audio_Event), &queue_params);

    ANIMATE_Init(&audio_animate_instance);

}

void AUDIO_UpdateParams(AUDIO_DynamicParams* dynamic) {
    /* If dynamic pending flag is set return immediately. */
    if (audio_params_pending_flag) {
        return;
    }

    /* Convert Audio Dynamics into ALGO dynamics. */
    audio_algo_params_pending.gain_dB = dynamic->gain_dB;
    audio_algo_params_pending.band_compensation = dynamic->band_compensation;
    audio_algo_params_pending.contrast = dynamic->contrast;
    ALGO_CalculateSmoothingCoeffs(&audio_freq_analysis, dynamic->smoothing_factor, &audio_algo_params_pending.smoothing_coeffs);

    /* Check the queue is ready for events.  */
    if (!audio_queue_handle) {
        return;
    }

     /* Set dynamics pending flag and notify the task. */
    audio_params_pending_flag = 1;
    audio_Event event = audio_EVENT_PARAMS_UPDATE;
    osStatus_t s = osMessageQueuePut (audio_queue_handle, &event, 0, 0);
    if (s != osOK) {
        __BKPT();
    }

}

static void audio_task(void* params) {
    (void) params;

    /* Initialise an arm fft instance for 32 bit floats. */
    arm_status fft_stat = arm_rfft_fast_init_f32 (&audio_fft_instance, AUDIO_FFT_SIZE);
    if (fft_stat != ARM_MATH_SUCCESS) {
        __BKPT();
    }

    /* Print the FFT Properties */
    audio_fft_properties.init.sampling_rate = audio_SAMPLE_RATE_Hz;
    audio_fft_properties.init.samples_per_channel_per_buffer = AUDIO_BUFFER_SAMPLES_PER_CHANNEL;
    audio_fft_properties.init.fft_size = AUDIO_FFT_SIZE;
    ALGO_InitFftProperties(&audio_fft_properties);
    ALGO_Print(ALGO_PRINT_TYPE_FFT_PROPERTIES, &audio_fft_properties, &AUDIO_DEBUG_UART);
    osDelay(100); // print delay

    /* Initialise the Analysis Object */
    ALGO_FreqBand freq_bands[AUDIO_NUM_FREQ_BANDS] = {0};
    audio_freq_analysis.num_bands = AUDIO_NUM_FREQ_BANDS;
    audio_freq_analysis.min_freq = audio_MINIMUM_ANALYSIS_FREQUENCY;
    audio_freq_analysis.max_freq = audio_MAXIMUM_ANALYSIS_FREQUENCY;
    audio_freq_analysis.sampling_rate_Hz = audio_SAMPLE_RATE_Hz;
    audio_freq_analysis.buffer_size = AUDIO_BUFFER_SAMPLES_PER_CHANNEL;
    audio_freq_analysis.freq_bands = freq_bands;
    audio_freq_analysis.data.band_mags_f32 = audio_band_mags_f32;
    audio_freq_analysis.data.smoothed_band_mags_f32 = audio_smoothed_band_mags_f32;
    audio_freq_analysis.data.smoothing_filters = audio_smoothing_filters;
    ALGO_InitFreqAnalysis(&audio_freq_analysis);

    /* After initialising the analysis object set it's dynamic parameters */
    audio_freq_analysis.dynamic.gain_dB = 12.0f;
    audio_freq_analysis.dynamic.contrast = 0.0f;
    audio_freq_analysis.dynamic.band_compensation = 0.6f;
    // TODO Set smoothing coeffs

    /* Print any Analysis Init Info */
    ALGO_Print(ALGO_PRINT_TYPE_COEFFS, &audio_freq_analysis, &AUDIO_DEBUG_UART);
    osDelay(100); // Print delay

    ALGO_PipelineProperties pipeline = {
        .init = {
            .sampling_rate = audio_SAMPLE_RATE_Hz,
            .samples_per_channel_per_buffer = AUDIO_BUFFER_SAMPLES_PER_CHANNEL,
            .sys_clock_rate_Hz = AUDIO_CORE_CLOCK_Hz
        }
    };
    ALGO_InitPipelineProperties(&pipeline);

    /* Start audio driver */

    /* Begin Transmitting on the Output SAI. This is used for debug purposes
     * really, makes it easy to verify audio is being rx'd correctly. And
     * gives you a usable audio output if you need it. */
    HAL_StatusTypeDef s = HAL_SAI_Transmit_DMA(audio_output_sai, (uint8_t*)audio_raw_buffer, audio_BUFFER_RAW_MEMORY);
    if (HAL_OK != s) {
        __BKPT();
    }

    /* Start receiving on the Input SAI. We will then drive the audio task
     * from the interrupts of this peripheral. */
    s = HAL_SAI_Receive_DMA(audio_input_sai, (uint8_t*)audio_raw_buffer, audio_BUFFER_RAW_MEMORY);
    if(HAL_OK != s) {
        __BKPT();
    }

    for (;;) {
        /* Read queue */
        audio_Event event = audio_EVENT_NONE;
        osStatus_t s = osMessageQueueGet (audio_queue_handle, &event, NULL, osWaitForever);
        if (osErrorTimeout == s) {
            /* We want to wait forever unless we add a watchdog. */
            continue;
        } else if (osOK != s) {
            /* Something has gone wrong */
            __BKPT();
        }

        /* Handle event */
        switch (event) {
            case audio_EVENT_NONE:
                __BKPT();
                continue;

            case audio_EVENT_BUFFER_A_READY:
                /* Run processing algo with pointer to beginning of buffer */
                audio_Algorithm(&audio_raw_buffer[0]);

                /* Set state to awaiting buffer B */
                audio_state = audio_STATE_AWAITING_BUFFER_B;
                continue;

            case audio_EVENT_BUFFER_B_READY:
                /* Run processing algo with pointer to mid-point of buffer */
                audio_Algorithm(&audio_raw_buffer[audio_BUFFER_RAW_MEMORY>>1]);

                /* Set state to awating Buffer A */
                audio_state = audio_STATE_AWAITING_BUFFER_A;
                continue;

            case audio_EVENT_PARAMS_UPDATE:
                /* Reconfigure algorithm parameters and then clear the update pending flag. */
                ALGO_UpdateFreqAnalysis(&audio_freq_analysis, &audio_algo_params_pending);
                // ALGO_Print(ALGO_PRINT_TYPE_DYNAMIC, &audio_freq_analysis, &AUDIO_DEBUG_UART);
                audio_params_pending_flag = 0;
                continue;
        }
        /* Code here will not execute. */
    }

}

static void audio_RxHalfCompleteCallback(SAI_HandleTypeDef* hsai){
    if (audio_state != audio_STATE_AWAITING_BUFFER_A) {
        /* Underflow - we are not processing fast enough. */
       __BKPT();
        return;
    }
    audio_Event event = audio_EVENT_BUFFER_A_READY;
    osStatus_t s = osMessageQueuePut (audio_queue_handle, &event, 0, 0);
    if (s != osOK) {
        __BKPT();
    }
} 

static void audio_RxCompleteCallback(SAI_HandleTypeDef* hsai){
    if (audio_state != audio_STATE_AWAITING_BUFFER_B) {
        /* Underflow - we are not processing fast enough. */
       __BKPT();
    	return;
    }
    audio_Event event = audio_EVENT_BUFFER_B_READY;
    osStatus_t s = osMessageQueuePut (audio_queue_handle, &event, 0, 0);
    if (s != osOK) {
        __BKPT();
    }
}

static void audio_InitCodec (uint32_t sampleRate, uint32_t volumePercent) {

    /* Verify codec is there*/
    if(wm8994_drv.ReadID(audio_I2C_ADDRESS) != WM8994_ID){
        __BKPT();
        while(1);
    }

    /* Reset the Codec Registers */
    wm8994_drv.Reset(audio_I2C_ADDRESS);
 
    /* Initialize the codec internal registers */
    wm8994_drv.Init(audio_I2C_ADDRESS, INPUT_DEVICE_INPUT_LINE_1 | OUTPUT_DEVICE_HEADPHONE, volumePercent, sampleRate);

}

/* Mono-ized audio data in q15 format */
static int16_t audio_m_q15[AUDIO_BUFFER_SAMPLES_PER_CHANNEL];
/* Mono-ized audio data in f32 format */
static float audio_m_f32_padded[AUDIO_FFT_SIZE] = {0.0f};
/* Interleaved real/imag fft result data */
static float audio_fft_f32[AUDIO_FFT_SIZE] = {0.0f};
/* Magntiudes from fft */
static float audio_mag_f32[audio_FFT_NUM_BINS] = {0.0f};

/* audio_lr - buffer of interleaved audio samples. */
static void audio_Algorithm (int16_t *audio_lr) {

    /* Convert to Mono */
    for (int i = AUDIO_BUFFER_SAMPLES_PER_CHANNEL-1; i >= 0; i--){
        int16_t sample_l_q14 = audio_lr[i*2] >> 1;
        int16_t sample_r_q14 = audio_lr[(i*2)+1] >> 1;
        audio_m_q15[i] = sample_l_q14 + sample_r_q14;
    }

    /* Convert to float and place in padded array, reset the padding to zero
     * as the cmsis fft function messes with it. */
    memset(audio_m_f32_padded, 0, audio_FFT_PADDING*sizeof(float));
    float* audio_m_f32 = &audio_m_f32_padded[audio_FFT_PADDING];
    arm_q15_to_float(audio_m_q15, audio_m_f32, AUDIO_BUFFER_SAMPLES_PER_CHANNEL);

    /* Run fft */
    uint8_t inverse_fft_flag = 0;
    arm_rfft_fast_f32(&audio_fft_instance, audio_m_f32_padded, audio_fft_f32, inverse_fft_flag);

    /* Compute magnitude and scale
     * Due to the way that cmsis-dsp packs the output buffer the first bin is 
     * purely real so store it directly in the mag buffer. And the "second" bin
     * is actually the pure magnitude value at nyquist freq so just discard it.
     * The rest of the data is real/imaginary interleaved so compute the 
     * magintude and store in our mag buffer. */
    float scale_factor = 1.0f / AUDIO_BUFFER_SAMPLES_PER_CHANNEL;
    audio_mag_f32[0] = fabs(audio_fft_f32[0]) * scale_factor;
    for (int i = AUDIO_FFT_SIZE/2-1; i > 0; i--) {
        float real_squared = audio_fft_f32[i*2] * audio_fft_f32[i*2];
        float imag_squared = audio_fft_f32[(i*2)+1] * audio_fft_f32[(i*2)+1];
        audio_mag_f32[i] = sqrtf(real_squared + imag_squared) * scale_factor;
    }

    // audio_DebugResults(audio_mag_f32, &AUDIO_DEBUG_UART);
    
    /* Run the Frequency Analysis and then print the resulting band magnitudes. */
    float* band_mags = ALGO_RunFreqAnalysis(&audio_freq_analysis, &audio_fft_properties, 
                         audio_mag_f32);
    ALGO_Print(ALGO_PRINT_TYPE_BAND_MAGS, &audio_freq_analysis, &AUDIO_DEBUG_UART);

    /* Basic LED Animation to View Brightness */
    // for (int i = 0; i < AUDIO_NUM_FREQ_BANDS; i++) {
    //     for ( int j = 0; j < 23; j++ ) {
    //         uint8_t red = 0xF6 * band_mags[AUDIO_NUM_FREQ_BANDS-i];
    //         uint8_t green = 0x26 * band_mags[AUDIO_NUM_FREQ_BANDS-i];
    //         uint8_t blue = 0x81 * band_mags[AUDIO_NUM_FREQ_BANDS-i];
    //         int offset = i * 23;
    //         LED_SetPixel(offset+j, 0, 0, blue);
    //     }
    	
    // }
    // LED_Sync();

    /* Update Animation instance with latest magnitudes */
    ANIMATE_UpdateMagnitudes(&audio_animate_instance, band_mags);
    ANIMATE_Run(&audio_animate_instance);

}



static void audio_DebugResults (float *mag_f32, UART_HandleTypeDef* huart) {
    
    if (!huart) {
        return;
    }

    /* Limit this to printing every half second */
    const uint32_t rate_per_second = 3;
    const uint32_t debug_count_max = (SAI_AUDIO_FREQUENCY_48K/AUDIO_BUFFER_SAMPLES_PER_CHANNEL)/rate_per_second;
    static int debug_count=50;
    if (debug_count-- > 0) {
    	return;
    }
    debug_count = debug_count_max;

    /* static debug buffers */
    #define AUDIO_DEBUG_NUMBER_OF_BINS  128
    static const uint16_t debug_str_len = (AUDIO_DEBUG_NUMBER_OF_BINS/2)*4;
    static char debug_str[(AUDIO_DEBUG_NUMBER_OF_BINS/2)*4];

    /* Start with empty space */
    memset(debug_str, ' ', debug_str_len);
    
    /* For each value, multiply by 1000 then convert to int, this should give
     * up to 3 sig figs for each. Then convert and add into the string.*/
    for (int i = 0; i < AUDIO_DEBUG_NUMBER_OF_BINS; i++) {
        int temp_int = (int)(mag_f32[i] * 1000.0f);
        char* temp_str[4] = {0};
        itoa(temp_int, temp_str, 10);
        memcpy(&debug_str[i*4], temp_str, strlen(temp_str));
        debug_str[(i*4)+3] = ',';
    }

    /* End with new line */
    memset(&debug_str[debug_str_len-1], '\n', 1);

	/* Transmit */
    HAL_StatusTypeDef s = HAL_UART_Transmit_DMA(huart, debug_str, debug_str_len);
    if (HAL_OK != s) {
    	__BKPT();
    }

}
