/* == INCLUDES ============================================================= */

#include "stdint.h"
#include "stm32f7xx_hal.h"
#include "wm8994.h"

/* == CONFIGURATION ======================================================== */

#define AUDIO_BUFFER_SAMPLES	1024
#define AUDIO_BUFFER_CHANNELS	2
#define AUDIO_BUFFER_LEN	AUDIO_BUFFER_SAMPLES * AUDIO_BUFFER_CHANNELS

/* == DEFINES ============================================================== */

#define AUDIO_I2C_ADDRESS                ((uint16_t)0x34)

/* == FILE STATIC FUNCTIONS ================================================ */

static void audio_InitCodec (uint32_t sampleRate, uint32_t volumePercent);

static void audio_RxHalfCompleteCallback(SAI_HandleTypeDef* hsai);
static void audio_RxCompleteCallback(SAI_HandleTypeDef* hsai);

/* == FILE STATIC VARIABLES ================================================ */
static uint32_t count = 0;
static int16_t inputBufferLR[AUDIO_BUFFER_LEN] = {0};
static int16_t outputBufferLR[AUDIO_BUFFER_LEN] = {0};

static int16_t* audio_data_lr;

void AUDIO_Start (SAI_HandleTypeDef* audio_in_sai, SAI_HandleTypeDef* audio_out_sai) {


    /* TODO Hardware Init that is currently handled by cube mx IOC. */

    /* Start the SAI Output Block to generate MCLK */
    __HAL_SAI_ENABLE(audio_out_sai);

    uint32_t sampleRate = SAI_AUDIO_FREQUENCY_48K;
    audio_InitCodec(sampleRate, 100);

    /* Set the callbacks */
    HAL_SAI_RegisterCallback(audio_in_sai, HAL_SAI_RX_HALFCOMPLETE_CB_ID, audio_RxHalfCompleteCallback);
    HAL_SAI_RegisterCallback(audio_in_sai, HAL_SAI_RX_COMPLETE_CB_ID, audio_RxCompleteCallback);

    /* End of init - onto start */

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
    HAL_StatusTypeDef s = HAL_SAI_Transmit_DMA(audio_out_sai, inputBufferLR, AUDIO_BUFFER_SAMPLES);
    if (HAL_OK != s) {
        __BKPT();
    }

    /* Start receiving on the Input SAI. We will then drive the audio task
     * from the interrupts of this peripheral. */
    s = HAL_SAI_Receive_DMA(audio_in_sai, inputBufferLR, AUDIO_BUFFER_SAMPLES);
    if(HAL_OK != s) {
        __BKPT();
    }

}

static void audio_RxHalfCompleteCallback(SAI_HandleTypeDef* hsai){
    audio_data_lr = inputBufferLR[0];
} 

static void audio_RxCompleteCallback(SAI_HandleTypeDef* hsai){
    audio_data_lr = inputBufferLR[AUDIO_BUFFER_LEN>>1];
}

static void audio_Init( void ) {

    uint32_t sampleRate = SAI_AUDIO_FREQUENCY_48K;

    audio_InitCodec(sampleRate, 100);

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
