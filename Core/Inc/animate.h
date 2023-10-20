/* == INCLUDES ============================================================= */

#include "stm32f7xx_hal.h"

#include "stdint.h"

/* == CONFIGURATION ======================================================== */

#define ANIMATE_DEBUG_UART (huart5)
#if defined ANIMATE_DEBUG_UART
extern UART_HandleTypeDef ANIMATE_DEBUG_UART;
#else
#define ANIMATE_DEBUG_UART (NULL)
#endif

/* Number of Frequency Bands */
#define ANIMATE_NUM_GROUPS        (9)

/* System Clock Rate */
#define ANIMATE_CORE_CLOCK_Hz SystemCoreClock
extern uint32_t SystemCoreClock;

/* == TYPES ================================================================ */

typedef enum ANIMATE_Mode_e {
    ANIMATE_MODE_STATIC = 0,
} ANIMATE_Mode;

typedef struct ANIMATE_Colour_s {
    uint8_t red;
    uint8_t blue;
    uint8_t green;
}ANIMATE_Colour;

typedef struct ANIMATE_GroupDynamics_s{
    ANIMATE_Colour colour;
}ANIMATE_GroupDynamics;

typedef struct ANIMATE_Group_s{
    uint32_t size;
    uint32_t pos;
    float magnitude; // 0.0 to 1.0
    ANIMATE_GroupDynamics dynamic;
}ANIMATE_Group;

typedef struct ANIMATE_GlobalDynamics_s {
    ANIMATE_Mode mode;
    float speed; // -1.0 to 1.0
    float hue;  // 0.0 to 360.0
}ANIMATE_GlobalDynamics;

typedef struct ANIMATE_GlobalState_s {
    float offset;
    float offset_increment;
}ANIMATE_GlobalState;

typedef struct ANIMATE_Instance_s {
    ANIMATE_Group groups[ANIMATE_NUM_GROUPS];
    ANIMATE_GlobalDynamics dynamic;
    ANIMATE_GlobalState state;
}ANIMATE_Instance;

/* == INTERFACE FUNCTIONS ================================================== */

void ANIMATE_Init(ANIMATE_Instance* instance);

void ANIMATE_Run(ANIMATE_Instance* instance);

void ANIMATE_UpdateMagnitudes(ANIMATE_Instance* instance, float* mags);

void ANIMATE_UpdateGlobalDynamics(ANIMATE_Instance* instance, ANIMATE_GlobalDynamics* dynamic);

