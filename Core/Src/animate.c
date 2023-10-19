
/* == INCLUDES ============================================================= */
#include "animate.h"

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

#include "led.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/* == CONFIGURATION ======================================================== */

#define animate_GROUP_SIZE  23

/* == DEFINES ============================================================== */

/* == TYPES ================================================================ */

/* == FILE STATIC ========================================================== */

/* == INTERFACE FUNCTINS =================================================== */

void ANIMATE_Init(ANIMATE_Instance* instance) {
    // Set the global animation mode
    instance->dynamic.mode = ANIMATE_MODE_STATIC;

    // Initialize groups property in a loop
    for (uint32_t i = 0; i < ANIMATE_NUM_GROUPS; ++i) {
        // Set the RGB color and magnitude for each group
        instance->groups[i].dynamic.colour.red = 0xF6;
        instance->groups[i].dynamic.colour.green = 0x26;
        instance->groups[i].dynamic.colour.blue = 0x81;
        instance->groups[i].magnitude = 0.0;
        instance->groups[i].size = animate_GROUP_SIZE;
        instance->groups[i].pos = animate_GROUP_SIZE * i;
    }

    LED_Init();
}

void ANIMATE_Run(ANIMATE_Instance* instance) {
    // Loop through each group
    for (uint32_t i = 0; i < ANIMATE_NUM_GROUPS; ++i) {
        uint32_t currentPosition = instance->groups[i].pos;
        // Loop from 0 to size to set each pixel individually.
        for (uint32_t j = 0; j < instance->groups[i].size; ++j) {
            // Calculate pixel index
            uint32_t index = currentPosition + j;

            // Get RGB values from the current group's colour
            uint8_t red = instance->groups[i].dynamic.colour.red;
            uint8_t green = instance->groups[i].dynamic.colour.green;
            uint8_t blue = instance->groups[i].dynamic.colour.blue;

            // Multiply RGB values by the magnitude of the current group
            float magnitude = instance->groups[i].magnitude;
            red = (uint8_t)(red * magnitude);
            green = (uint8_t)(green * magnitude);
            blue = (uint8_t)(blue * magnitude);

            // Call LED_SetPixel function to set the pixel color
            LED_SetPixel(index, red, green, blue);
        }
    }

    /* Synchronize LEDs */
    LED_Sync();
}


void ANIMATE_UpdateMagnitudes(ANIMATE_Instance* instance, float* mags) {
    // Update magnitudes for each group
    for (uint32_t i = 0; i < ANIMATE_NUM_GROUPS; ++i) {
        instance->groups[i].magnitude = mags[i];
    }
}
