// /* Basic premise */

// // Initialise audio in from 3.5mm 
// BSP_AUDIO_IN_InitEx(INPUT_DEVICE_ANALOG_MIC)
//     // SAIx_In_Init is what sets up the peripheral

// // Start "recording"
// BSP_AUDIO_IN_Record(buffer, buf_len)
//     // This calls to the hal to start filling the buffer.

// /* Need to implement */
// void    BSP_AUDIO_IN_TransferComplete_CallBack(void);
// void    BSP_AUDIO_IN_HalfTransfer_CallBack(void);
// // Guess we can just have a "notify" function that tells us which half of the buffer to process.


// /* For debug: */
// uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq);
// // where output device is OUTPUT_DEVICE_HEADPHONE? - Think so

// // To play - this uses circular buffer to play so we should just be able to always copy from in buff
// // to out buff and start them at opposite
// // IN xfer half irq - copy to out buff A
// // IN xfer full irq - press play - copy to out buff B 
//     BSP_AUDIO_OUT_Play()

#include "stm32f769i_discovery_audio.h"
#include "stdint.h"

#define AUDIO_BUFFER_SAMPLES	1024
#define AUDIO_BUFFER_CHANNELS	2
#define AUDIO_BUFFER_LEN	AUDIO_BUFFER_SAMPLES * AUDIO_BUFFER_CHANNELS

static void audio_Init( void );
static void audio_InitCodec (uint32_t sampleRate, uint32_t volumePercent);

static uint32_t count = 0;
static int16_t inputBufferLR[AUDIO_BUFFER_LEN] = {0};
static int16_t outputBufferLR[AUDIO_BUFFER_LEN] = {0};

void AUDIO_Start (SAI_HandleTypeDef* audio_in_sai, SAI_HandleTypeDef* audio_out_sai) {

    uint32_t sampleRate = BSP_AUDIO_FREQUENCY_48K;
    uint32_t bitRes = 16;
    uint32_t unusedParam = 0;

    /* TODO Hardware Init that is currently handled by cube mx IOC. */

    /* Start the SAI Output Block to generate MCLK */
    __HAL_SAI_ENABLE(audio_out_sai);

    audio_Init();

    /* End of init - onto start */

    /* Call the audio Codec Play function - The interface says it needs buffer 
     * And size, this is not the case. */
    if (wm8994_drv.Play(AUDIO_I2C_ADDRESS, NULL, 0) != 0)
    {
        __BKPT();
    }
    /* Update the Media layer and enable it for play */
    HAL_StatusTypeDef s = HAL_SAI_Transmit_DMA(audio_out_sai, inputBufferLR, AUDIO_BUFFER_SAMPLES);
    if (HAL_OK != s) {
        __BKPT();
    }

    /* Start the process receive DMA */
    s = HAL_SAI_Receive_DMA(audio_in_sai, inputBufferLR, AUDIO_BUFFER_SAMPLES);
    if(HAL_OK != s) {
        __BKPT();
    }

}

void    BSP_AUDIO_IN_TransferComplete_CallBack(void) {
	count++;
}
void    BSP_AUDIO_IN_HalfTransfer_CallBack(void){
	static uint8_t output = 0;
	if (!output) {
		//BSP_AUDIO_OUT_Play(inputBufferLR, AUDIO_BUFFER_SAMPLES);
		output = 1;
	}
	count++;
}

void    BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	count--;
}
void    BSP_AUDIO_OUT_HalfTransfer_CallBack(void){
	count--;
}

#if 1

static void audio_Init( void ) {

    uint32_t sampleRate = BSP_AUDIO_FREQUENCY_48K;

    /* BSP_AUDIO_IN_InitEx() */

    /* De-initialise both sai blocks. */
    // SAIx_In_DeInit();   

    /* Configure the PLL to generate the output clock. */
    /* I beleive that this is already configured by our generated code. */
    // this function doesn't actually use the handle. Just the frequency.
    // BSP_AUDIO_OUT_ClockConfig(&haudio_in_sai, sampleRate, NULL);

    /* Configure the output side pins. 
     * Because we pass the in handle to this out function it does not actually
     * set the DMA at all. */
    // BSP_AUDIO_OUT_MspInit(&haudio_in_sai, NULL);

    /* Configure the input side pins and dma. 
     * Just calls directly to SAI_AUDIO_IN_MspInit(&haudio_in_sai, NULL); under the hood. 
     * Which we will need to use if we have the handle stored in this scope. */
    // probably all set up by our own configuration in the ioc
    // BSP_AUDIO_IN_MspInit();

    /* This sets up both the input and output blocks of the SAI such that 
     * the output is master tx and the input is slave rx. */
    // could be replaced by our own configuration?
//    SAIx_In_Init(sampleRate);                                               // This is necessary for it to work?

    /* Initialise the codec with full volume */
    // audio_InitCodec(sampleRate, 100);

    /* BSP_AUDIO_IN_OUT_Init() */

    /* Disable SAI */
//   SAIx_In_DeInit();                                                       // REPEAT

    /* Initialise the PLL to generate the output clock */
//    BSP_AUDIO_OUT_ClockConfig(&haudio_in_sai, sampleRate, NULL);             // REPEAT

    /* Initialise the output pins and dma */
    // BSP_AUDIO_OUT_MspInit(&haudio_out_sai, NULL);

    /* As above in other function, call BSP_AUDIO_IN_MspInit but because it's
     * the line in it just calls direct to sai init. */
    // SAI_AUDIO_IN_MspInit(&haudio_in_sai, NULL);                             // REPEAT

    /* As above - set up both the input and output blocks for I2S */
    // SAIx_In_Init(sampleRate);                                                // REPEAT

    /* Initialise the codec with full volume */
    audio_InitCodec(sampleRate, 100);                                       // REPEAT

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

#endif
