/* == INCLUDES ============================================================= */

#include "audio.h"

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

#include "wm8994.h"

#define ARM_MATH_CM7
#include "arm_math.h"

#include <stdint.h>

/* == CONFIGURATION ======================================================== */

#define AUDIO_BUFFER_SAMPLES_PER_CHANNEL	1024U
#define AUDIO_BUFFER_CHANNELS	2

/* FFT Size 
 * This should be greater than the audio samples per channel such that we get
 * a decent amount of padding. However it must align to a power of 2. */
#define AUDIO_FFT_SIZE 2048U

/* == DEFINES ============================================================== */

/* I2C Address of codec */
#define AUDIO_I2C_ADDRESS                ((uint16_t)0x34)

/* Size of the audio buffer in total samples */
#define AUDIO_BUFFER_SIZE AUDIO_BUFFER_SAMPLES_PER_CHANNEL * AUDIO_BUFFER_CHANNELS

/* Audio Buffer Total Length 
 * n samples per channel, multiplied by 2 such that both of the ping-pong 
 * buffers are equal. */
#define AUDIO_BUFFER_RAW_MEMORY	AUDIO_BUFFER_SIZE * 2

/* FFT Padding */
#define AUDIO_FFT_PADDING   (AUDIO_FFT_SIZE - AUDIO_BUFFER_SAMPLES_PER_CHANNEL)

/* Audio FFT Bins 
 * Number of bins is half that of the fft size */
#define AUDIO_FFT_BINS      AUDIO_FFT_SIZE / 2

/* == TYPES ================================================================ */

typedef enum audio_Event_e {
    audio_EVENT_NONE,
    audio_EVENT_BUFFER_A_READY,
    audio_EVENT_BUFFER_B_READY,
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

static void audio_DebugResults (float *mag_f32);

/* == FILE STATIC VARIABLES ================================================ */

static int16_t audio_raw_buffer[AUDIO_BUFFER_RAW_MEMORY] = {0};

static osThreadId_t audio_task_handle = NULL;
static osMessageQueueId_t audio_queue_handle = NULL;
static SAI_HandleTypeDef* audio_input_sai = NULL;
static SAI_HandleTypeDef* audio_output_sai = NULL; 

audio_State audio_state = audio_STATE_AWAITING_BUFFER_A;

arm_rfft_fast_instance_f32 audio_fft_instance;

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

}

void AUDIO_Start (void) {

	uint32_t sampleRate = SAI_AUDIO_FREQUENCY_48K;
	/* Init codec once RTOS is running */
	audio_InitCodec(sampleRate, 100);

    /* Unmute the Audio CODEC 
     * Note that the API says it needs buffer and size, this is not the case
     * so we send NULL and 0. */
    if (wm8994_drv.Play(AUDIO_I2C_ADDRESS, NULL, 0) != 0)
    {
        __BKPT();
    }

    /* Begin Transmitting on the Output SAI. This is used for debug purposes
     * really, makes it easy to verify audio is being rx'd correctly. And
     * gives you a usable audio output if you need it. */
    HAL_StatusTypeDef s = HAL_SAI_Transmit_DMA(audio_output_sai, (uint8_t*)audio_raw_buffer, AUDIO_BUFFER_RAW_MEMORY);
    if (HAL_OK != s) {
        __BKPT();
    }

    /* Start receiving on the Input SAI. We will then drive the audio task
     * from the interrupts of this peripheral. */
    s = HAL_SAI_Receive_DMA(audio_input_sai, (uint8_t*)audio_raw_buffer, AUDIO_BUFFER_RAW_MEMORY);
    if(HAL_OK != s) {
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
                audio_Algorithm(&audio_raw_buffer[AUDIO_BUFFER_RAW_MEMORY>>1]);

                /* Set state to awating Buffer A */
                audio_state = audio_STATE_AWAITING_BUFFER_A;
                continue;
        }
    }

}

static void audio_RxHalfCompleteCallback(SAI_HandleTypeDef* hsai){
    if (audio_state != audio_STATE_AWAITING_BUFFER_A) {
        /* Underflow - we are not processing fast enough. */
//        __BKPT();
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
//        __BKPT();
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
    if(wm8994_drv.ReadID(AUDIO_I2C_ADDRESS) != WM8994_ID){
        __BKPT();
        while(1);
    }

    /* Reset the Codec Registers */
    wm8994_drv.Reset(AUDIO_I2C_ADDRESS);
 
    /* Initialize the codec internal registers */
    wm8994_drv.Init(AUDIO_I2C_ADDRESS, INPUT_DEVICE_INPUT_LINE_1 | OUTPUT_DEVICE_HEADPHONE, volumePercent, sampleRate);

}

/* Mono-ized audio data in q15 format */
static int16_t audio_m_q15[AUDIO_BUFFER_SAMPLES_PER_CHANNEL];
/* Mono-ized audio data in f32 format */
static float audio_m_f32_padded[AUDIO_FFT_SIZE] = {0.0f};
/* Interleaved real/imag fft result data */
static float audio_fft_f32[AUDIO_FFT_SIZE] = {0.0f};
/* Magntiude of fft */
static float audio_mag_f32[AUDIO_FFT_SIZE/2] = {0.0f};

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
    memset(audio_m_f32_padded, 0, AUDIO_FFT_PADDING*sizeof(float));
    float* audio_m_f32 = &audio_m_f32_padded[AUDIO_FFT_PADDING];
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
    
    audio_DebugResults(audio_mag_f32);

}



static void audio_DebugResults (float *mag_f32) {


#if defined AUDIO_DEBUG_UART
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
    HAL_StatusTypeDef s = HAL_UART_Transmit_DMA(&AUDIO_DEBUG_UART, debug_str, debug_str_len);
    if (HAL_OK != s) {
    	__BKPT();
    }

#endif

}

static void audio_DebugProcessingRate () {

}
