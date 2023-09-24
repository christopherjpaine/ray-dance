
#include "led.h"

#include <string.h>
#include <stdbool.h>

uint8_t byte_buffer[LED_BUFFER_SIZE_BYTES];
bool led_busy = false;

static void sync_complete_callback (SPI_HandleTypeDef hSpi);

void LED_Init(void) {
    // TODO set callback
    HAL_SPI_RegisterCallback(&(LED_SPI_HANDLE), HAL_SPI_TX_COMPLETE_CB_ID, sync_complete_callback);

    LED_SetAll(100, 100, 50);
    LED_SetPixel(LED_NUM_LEDS/2, 0, 0, 100);
    LED_Sync();
}

// #define WS2812_FILL_BUFFER(value) \
//     for( uint8_t mask = 0x80; mask; mask >>= 1 ) { \
//         if( value & mask ) { \
//             *ptr++ = 0xfc; \
//         } else { \
//             *ptr++ = 0x80; \
//         } \
//     }

// void set_pixel(uint16_t led_no, uint8_t r, uint8_t g, uint8_t b) {
//     uint8_t * ptr = &byte_buffer[24 * led_no];
//     WS2812_FILL_BUFFER(g);
//     WS2812_FILL_BUFFER(r);
//     WS2812_FILL_BUFFER(b);
// }

static inline void set_pixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b) {
    uint16_t pixel_pos = (24 * pixel) + LED_RESET_PULSE_BYTES;
    uint8_t* ptr_g = &byte_buffer[pixel_pos];
    uint8_t* ptr_r = &byte_buffer[pixel_pos+8];
    uint8_t* ptr_b = &byte_buffer[pixel_pos+16];
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

void LED_SetPixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b) {
    /* Ignore out of bounds pixels */
    if (LED_NUM_LEDS <= pixel) {
        return;
    }
    set_pixel(pixel, r, g, b);
}

void LED_SetAll(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t * ptr = byte_buffer;
    for( uint16_t i = 0; i < LED_NUM_LEDS; ++i) {
        set_pixel(i, r, g, b);
    }
}

void LED_Sync(void) {
    if (led_busy) {
        return;
    }
    led_busy = true;
    HAL_SPI_Transmit_DMA(&LED_SPI_HANDLE, byte_buffer, LED_BUFFER_SIZE_BYTES);
}

static void sync_complete_callback (SPI_HandleTypeDef hSpi) {
    (void) hSpi;
    led_busy = false;
}