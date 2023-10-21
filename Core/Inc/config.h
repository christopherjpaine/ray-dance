
#ifndef RAYDANCE_CONFIG_H_
#define RAYDANCE_CONFIG_H_

#include <stdint.h>
/* Use RGB Colour when set to 1 we do colour as RGB, otherwise we go by hue. */
#define CONFIG_DMX_COLOUR_RGB       0

typedef struct CONFIG_Rgb_s{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
}CONFIG_Rgb;


#endif
