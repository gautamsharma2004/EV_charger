/* ================== tm1637.h ================== */
#ifndef TM1637_H
#define TM1637_H

#include <stdint.h>
#include "py32f0xx_hal.h"


// ============================================================================
// TM1637 GPIO Configuration
// ============================================================================

#define TM1637_CLK_PORT     GPIOB
#define TM1637_CLK_PIN      GPIO_PIN_2

#define TM1637_DIO_PORT     GPIOB
#define TM1637_DIO_PIN      GPIO_PIN_1

// ============================================================================
// TM1637 Commands
// ============================================================================

#define TM1637_CMD_SETDATA  0x40
#define TM1637_CMD_DISPLAY  0x80
#define TM1637_CMD_ADDRESS  0xC0

// ============================================================================
// Function Prototypes
// ============================================================================

void TM1637_Init(void);

void TM1637_SetBrightness(uint8_t level);

void TM1637_Clear(void);

void TM1637_DisplayRaw(uint8_t data[4]);

void TM1637_DisplayNumber(uint16_t number);

void TM1637_DisplayVoltage(uint16_t value);

void TM1637_DisplayCurrent(uint16_t value);
void TM1637_DisplayFault(void);
void TM1637_DisplayTime(uint8_t minutes, uint8_t seconds, uint8_t showColon);

#endif