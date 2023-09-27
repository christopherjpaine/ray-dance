/* == INCLUDES ============================================================= */

#include "audio.h"

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

#include "wm8994.h"

#include <stdint.h>

/* == CONFIGURATION ======================================================== */

#define AUDIO_BUFFER_SAMPLES	1024
#define AUDIO_BUFFER_CHANNELS	2
#define AUDIO_BUFFER_LEN	AUDIO_BUFFER_SAMPLES * AUDIO_BUFFER_CHANNELS

/* == DEFINES ============================================================== */

#define AUDIO_I2C_ADDRESS                ((uint16_t)0x34)

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

/* == FILE STATIC VARIABLES ================================================ */
static uint32_t count = 0;
static int16_t inputBufferLR[AUDIO_BUFFER_LEN] = {0};
static int16_t outputBufferLR[AUDIO_BUFFER_LEN] = {0};

static int16_t* audio_data_lr;

static osThreadId_t audio_task_handle = NULL;
static osMessageQueueId_t audio_queue_handle = NULL;
static SAI_HandleTypeDef* audio_input_sai = NULL;
static SAI_HandleTypeDef* audio_output_sai = NULL; 

audio_State audio_state = audio_STATE_AWAITING_BUFFER_A;

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
    HAL_StatusTypeDef s = HAL_SAI_Transmit_DMA(audio_output_sai, inputBufferLR, AUDIO_BUFFER_SAMPLES);
    if (HAL_OK != s) {
        __BKPT();
    }

    /* Start receiving on the Input SAI. We will then drive the audio task
     * from the interrupts of this peripheral. */
    s = HAL_SAI_Receive_DMA(audio_input_sai, inputBufferLR, AUDIO_BUFFER_SAMPLES);
    if(HAL_OK != s) {
        __BKPT();
    }

}

static void audio_task(void* params) {
    (void) params;


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
                audio_data_lr = inputBufferLR[0];

                /* Set state to awaiting buffer B */
                audio_state = audio_STATE_AWAITING_BUFFER_B;
                continue;

            case audio_EVENT_BUFFER_B_READY:
                /* Run processing algo with pointer to mid-point of buffer */
                audio_data_lr = inputBufferLR[AUDIO_BUFFER_LEN>>1];

                /* Set state to awating Buffer A */
                audio_state = audio_STATE_AWAITING_BUFFER_A;
                continue;
        }
    }

}

static void audio_RxHalfCompleteCallback(SAI_HandleTypeDef* hsai){
    if (audio_state != audio_STATE_AWAITING_BUFFER_A) {
        /* Underflow - we are not processing fast enough. */
        __BKPT();
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
