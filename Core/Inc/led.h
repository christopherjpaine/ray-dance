#ifndef INC_LED_H_
#define INC_LED_H_

#include "stdint.h"
#include "stddef.h"

#include "stm32f7xx_hal.h"

#define LED_NUM_LEDS 8
#define LED_SPI_HANDLE hspi2

#define LED_RESET_PULSE_BYTES 60
#define LED_BUFFER_SIZE_BYTES (LED_NUM_LEDS * 24 + LED_RESET_PULSE_BYTES)

extern SPI_HandleTypeDef LED_SPI_HANDLE;

void LED_Init(void);

void LED_SetPixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b);

void LED_SetAll(uint8_t r, uint8_t g, uint8_t b);

void LED_Sync (void);

#endif /* INC_LED_H_ */