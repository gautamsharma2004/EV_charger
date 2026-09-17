/* ================== fan_control.h ================== */
#ifndef FAN_CONTROL_H
#define FAN_CONTROL_H
#define FAN_GPIO_PORT       GPIOA
#define FAN_GPIO_PIN        GPIO_PIN_2
#include <stdint.h>
#include "py32f0xx_hal.h"


// Fan state enum
typedef enum {
    FAN_STATE_OFF,
    FAN_STATE_ON,
    FAN_STATE_STOP
}
 Fan_State_t;

// Function prototypes
void Fan_Init(void);
void Fan_On(void);
void Fan_Off(void);
void Fan_Toggle(void);
void Fan_Controller_Init(void);
void Fan_Start(void);
void Fan_Stop(void);
void Fan_Tick_Internal(void);
Fan_State_t Fan_GetState(void);
uint32_t Fan_GetElapsedTime(void);
uint8_t Fan_IsComplete(void);

#endif