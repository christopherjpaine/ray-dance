
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

#define animate_MAXIMUM_OFFSET_INCREMENT    50

/* == DEFINES ============================================================== */

/* == TYPES ================================================================ */

/* == FILE STATIC ========================================================== */

/* == INTERFACE FUNCTINS =================================================== */

void ANIMATE_Init(ANIMATE_Instance* instance) {
    /* Set dynamics to their defaults */
    instance->dynamic.mode = ANIMATE_MODE_STATIC;
    instance->dynamic.speed = 0.0f;

    /* Configure the initial state, both internal params and those driven
     * by the dynamic parameters. */
    instance->state.offset = 0.0f;
    ANIMATE_UpdateGlobalDynamics(instance, &instance->dynamic);

    /* Initialize groups property in a loop */
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

    // Increment the offset by the specified amount (as a floating-point value)
    instance->state.offset += instance->state.offset_increment;

    // Wrap the offset at the number of LEDs
    if (instance->state.offset >= LED_NUM_LEDS) {
        instance->state.offset -= LED_NUM_LEDS;
    } else if (instance->state.offset < 0) {
        instance->state.offset += LED_NUM_LEDS;
    }

    // Round the offset to the nearest integer and store it as offset_u32
    uint32_t offset_u32 = (uint32_t)(instance->state.offset + 0.5);


    // Loop through each group
    for (uint32_t i = 0; i < ANIMATE_NUM_GROUPS; ++i) {

        // Calculate current position by summing global offset with group's position
        uint32_t currentPosition = instance->groups[i].pos + offset_u32;

        // Get RGB values from the current group's colour
        uint8_t red = instance->groups[i].dynamic.colour.red;
        uint8_t green = instance->groups[i].dynamic.colour.green;
        uint8_t blue = instance->groups[i].dynamic.colour.blue;

        // Multiply RGB values by the magnitude of the current group
        float magnitude = instance->groups[i].magnitude;
        red = (uint8_t)(red * magnitude);
        green = (uint8_t)(green * magnitude);
        blue = (uint8_t)(blue * magnitude);

        // Loop from 0 to size to set each pixel with the precomputed RGB values
        for (uint32_t j = 0; j < instance->groups[i].size; ++j) {
            // Calculate pixel index and wrap it at LED_NUM_LEDS
            uint32_t index = (currentPosition + j) % LED_NUM_LEDS;

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

void ANIMATE_UpdateGlobalDynamics(ANIMATE_Instance* instance, ANIMATE_GlobalDynamics* dynamics){

    /* Store copy in instance */
    memcpy(&instance->dynamic, dynamics, sizeof(*dynamics));

    /* Use speed to calculate offset increment. */
    instance->state.offset_increment = animate_MAXIMUM_OFFSET_INCREMENT * dynamics->speed;
}
