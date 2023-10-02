#ifndef INC_LED_H_
#define INC_LED_H_

/* == INCLUDES ============================================================= */

#include "stm32f7xx_hal.h"

#include "stdint.h"
#include "stddef.h"

/* == CONFIGURATION ======================================================== */

/* == TYPES ================================================================ */

// typedef struct LED_Controller_s {
    
// }LED_Controller;

/* == INTERFACE FUNCTIONS ================================================== */

#define LED_NUM_LEDS 100
#define LED_SPI_HANDLE hspi2



extern SPI_HandleTypeDef LED_SPI_HANDLE;

void LED_Init(void);

void LED_SetPixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b);

void LED_SetAll(uint8_t r, uint8_t g, uint8_t b);

void LED_Sync (void);

#endif /* INC_LED_H_ */