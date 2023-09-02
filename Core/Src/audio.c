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

void AUDIO_Start (void) {

    uint32_t sampleRate = 48000;
    uint32_t bitRes = 16;  
    uint32_t unusedParam = 0;
    // Instead of using BSP - we use the BSP as a guide to then configure our own
    // setup using cube.
    // Clocks and ouput SAI have been configured to handle 48kHz audio.
    // Clock is 49MHz though.. how do we configure the adc for that...
    // MCKDIV is configured in the SAI HAL :) 
    // Config from bsp is becoming verified and being transferred into ioc.

    // Output is master
    // Input is slave. It is a synchronous slave so we use the clock from the output.
    // DMA channels have been configured.
    // Next step is to literally add the buffers! 
    BSP_AUDIO_IN_InitEx(INPUT_DEVICE_ANALOG_MIC, sampleRate, bitRes, unusedParam);
}

