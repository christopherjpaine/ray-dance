
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

static void InterpolateRGBGradient (ANIMATE_Colour* input_a, ANIMATE_Colour* input_b, ANIMATE_Colour* output, float ratio);
static void GradientColouredGroups (ANIMATE_Instance* instance, ANIMATE_Colour* colour_start, ANIMATE_Colour* colour_end);


/* == INTERFACE FUNCTINS =================================================== */

void ANIMATE_Init(ANIMATE_Instance* instance) {
    /* Set dynamics to their defaults */
    instance->dynamic.mode = ANIMATE_MODE_STATIC;
    instance->dynamic.speed = 0.0f;
    #if CONFIG_DMX_COLOUR_RGB == 1
        instance->dynamic.colour.red = 0xF6;
        instance->dynamic.colour.green = 0x26;
        instance->dynamic.colour.blue = 0x81;
    #else
        instance->dynamic.hue = 0.0f;
    #endif

    /* Configure the initial state, both internal params and those driven
     * by the dynamic parameters. */
    instance->state.offset = 0.0f;
    ANIMATE_UpdateGlobalDynamics(instance, &instance->dynamic);

    /* Initialize groups property in a loop */
    for (uint32_t i = 0; i < ANIMATE_NUM_GROUPS; ++i) {
        // Set the RGB color and magnitude for each group
        // instance->groups[i].dynamic.colour.red = 0xF6;
        // instance->groups[i].dynamic.colour.green = 0x26;
        // instance->groups[i].dynamic.colour.blue = 0x81;
        instance->groups[i].magnitude = 0.0;
        instance->groups[i].size = animate_GROUP_SIZE-5;
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

    /* Set the colour of all the groups using the current colour input. */
    LED_RGB rgb;
    #if CONFIG_DMX_COLOUR_RGB == 1
        rgb.r = dynamics->colour.red;
        rgb.g = dynamics->colour.green;
        rgb.b = dynamics->colour.blue;
    #else 
        LED_HsvToRgb(dynamics->hue, 1.0, 1.0, &rgb);
    #endif

    #if 1
        ANIMATE_Colour rgb_end = {
            .red = 0xF6,
            .green = 0x26,
            .blue = 0x81,
        };
        GradientColouredGroups(instance, &dynamics->colour, &rgb_end);
    #else
        for (uint32_t i = 0; i < ANIMATE_NUM_GROUPS; ++i) {
            // Set the RGB color and magnitude for each group
            instance->groups[i].dynamic.colour.red = rgb.r;
            instance->groups[i].dynamic.colour.green = rgb.g;
            instance->groups[i].dynamic.colour.blue = rgb.b;
        }

    #endif


}


static void InterpolateRGBGradient (ANIMATE_Colour* input_a, ANIMATE_Colour* input_b, ANIMATE_Colour* output, float ratio) {
    /* Calculate colour diff */
    float diff_red = (input_b->red - input_a->red);
    float diff_green = (input_b->green - input_a->green);
    float diff_blue = (input_b->blue - input_a->blue);

    /* Return interpolated value at the given point between the two colours. */
    output->red = input_a->red + (uint8_t)(diff_red * ratio);
    output->green = input_a->green + (uint8_t)(diff_green * ratio);
    output->blue = input_a->blue + (uint8_t)(diff_blue * ratio);
    
}


static void GradientColouredGroups (ANIMATE_Instance* instance, ANIMATE_Colour* colour_start, ANIMATE_Colour* colour_end){

    /* Interpolate the first half going from start to end... */
    uint32_t halfway = (ANIMATE_NUM_GROUPS+1)/2;
    for (uint32_t i = 0; i < halfway; i++) {
        float ratio = i/ANIMATE_NUM_GROUPS;
        InterpolateRGBGradient(colour_start, colour_end, &instance->groups[i].dynamic.colour, ratio);
    }

    /* Then the other half going back so we get a full gradient.*/
    for (uint32_t i = halfway; i < ANIMATE_NUM_GROUPS; i++){
        float ratio = i/ANIMATE_NUM_GROUPS;
        InterpolateRGBGradient(colour_end, colour_start, &instance->groups[i].dynamic.colour, ratio);
    }

}
