#ifndef INC_LED_H_
#define INC_LED_H_

/* == INCLUDES ============================================================= */

#include "stm32f7xx_hal.h"

#include "stdint.h"
#include "stddef.h"

/* == CONFIGURATION ======================================================== */

#define LED_NUM_LEDS 207

/* Handle for the SPI Port that we will use. */
#define LED_SPI_HANDLE hspi2
extern SPI_HandleTypeDef LED_SPI_HANDLE;

/* == TYPES ================================================================ */

typedef struct LED_RGB_s {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} LED_RGB;


/* == INTERFACE FUNCTIONS ================================================== */


void LED_Init(void);

void LED_SetPixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b);

void LED_SetAll(uint8_t r, uint8_t g, uint8_t b);

void LED_Sync (void);

void LED_HsvToRgb(float hue, float saturation, float value, LED_RGB* rgb);

#endif /* INC_LED_H_ */