
#include "led.h"

#include <string.h>
#include <stdbool.h>
#include <math.h>

/* == CONFIGURATION ======================================================== */

/* The reset pulse is equivalent to the number of bytes we need to send at the
 * configured SPI frequency in order to exceed the WS2812B frame reset period. */
#define led_RESET_PULSE_BYTES 60

/* Buffer size is 24 bytes per LED, as each bit is represented by a byte so 
 * that we can use the SPI peripheral for sending the data. Plus the reset
 * pulse bytes. */
#define led_BUFFER_SIZE_BYTES (LED_NUM_LEDS * 24 + led_RESET_PULSE_BYTES)

/* == TYPES ================================================================ */


/* == FILE SCOPE VARIABLES ================================================= */

#if defined PIXEL_DEBUG
    static LED_RGB led_debug_pixel_buffer[LED_NUM_LEDS] = {0};
#endif

/* Pixel buffer and associated semaphore. */
static uint8_t led_pixel_buffer[led_BUFFER_SIZE_BYTES];
static bool led_busy = false;

/* == FILE SCOPE FUNCTION DECLARATIONS ===================================== */

static void SyncCompleteCallback (SPI_HandleTypeDef* hSpi);
static inline void SetPixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b);

/* == INTERFACE FUNCTIONS ================================================== */


void LED_Init(void) {
    // TODO set callback
    HAL_SPI_RegisterCallback(&(LED_SPI_HANDLE), HAL_SPI_TX_COMPLETE_CB_ID, SyncCompleteCallback);

    LED_SetAll(100, 100, 50);
    LED_SetPixel(LED_NUM_LEDS/2, 0, 0, 100);
    LED_Sync();
}

void LED_SetPixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b) {
    /* Ignore out of bounds pixels */
    if (LED_NUM_LEDS <= pixel) {
        return;
    }
    SetPixel(pixel, r, g, b);
}

void LED_SetAll(uint8_t r, uint8_t g, uint8_t b) {
    for( uint16_t i = 0; i < LED_NUM_LEDS; ++i) {
        SetPixel(i, r, g, b);
    }
}

void LED_Sync(void) {
    if (led_busy) {
        return;
    }
    led_busy = true;
    HAL_SPI_Transmit_DMA(&LED_SPI_HANDLE, led_pixel_buffer, led_BUFFER_SIZE_BYTES);
}

void LED_HsvToRgb(float hue, float saturation, float value, LED_RGB* rgb) {
    // Ensure hue is in the range [0, 360) degrees
    while (hue >= 360.0f) {
        hue -= 360.0f;
    }
    while (hue < 0.0f) {
        hue += 360.0f;
    }

    // Convert HSV to RGB
    float c = value * saturation;
    float x = c * (1 - fabsf(fmodf(hue / 60.0f, 2) - 1));
    float m = value - c;

    float r, g, b;

    if (hue >= 0.0f && hue < 60.0f) {
        r = c;
        g = x;
        b = 0;
    } else if (hue >= 60.0f && hue < 120.0f) {
        r = x;
        g = c;
        b = 0;
    } else if (hue >= 120.0f && hue < 180.0f) {
        r = 0;
        g = c;
        b = x;
    } else if (hue >= 180.0f && hue < 240.0f) {
        r = 0;
        g = x;
        b = c;
    } else if (hue >= 240.0f && hue < 300.0f) {
        r = x;
        g = 0;
        b = c;
    } else {
        r = c;
        g = 0;
        b = x;
    }

    rgb->r = (uint8_t)((r + m) * 255);
    rgb->g = (uint8_t)((g + m) * 255);
    rgb->b = (uint8_t)((b + m) * 255);
}

/* == FILE SCOPE FUNCTIONS ================================================= */

static void SyncCompleteCallback (SPI_HandleTypeDef* hSpi) {
    (void) hSpi;
    led_busy = false;
}

static inline void SetPixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b) {

    #if defined PIXEL_DEBUG
        led_debug_pixel_buffer[pixel].r = r;
        led_debug_pixel_buffer[pixel].g = g;
        led_debug_pixel_buffer[pixel].b = b;
    #endif
    uint16_t pixel_pos = (24 * pixel) + led_RESET_PULSE_BYTES;
    uint8_t* ptr_g = &led_pixel_buffer[pixel_pos];
    uint8_t* ptr_r = &led_pixel_buffer[pixel_pos+8];
    uint8_t* ptr_b = &led_pixel_buffer[pixel_pos+16];
    for( uint8_t mask = 0x80; mask; mask >>= 1 ){
        if (g & mask){
            *ptr_g++ = 0xfc;
        } else {
            *ptr_g++ = 0x80;
        }
        if (r & mask){
            *ptr_r++ = 0xfc;
        } else {
            *ptr_r++ = 0x80;
        }
        if (b & mask) {
            *ptr_b++ = 0xfc;
        } else {
            *ptr_b++ = 0x80;
        }
    }
}
