#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "py32f0xx.h"
#include "py32f0xx_hal.h"

/* ================== main.h ================== */
// System clock from 24MHz external crystal
#define SYSTEM_CLOCK_HZ     24000000U
#define SYSTICK_FREQ_MS     1           // 1ms per tick

// SysTick configuration for 1ms interrupt
#define SYSTICK_RELOAD_VAL  (SYSTEM_CLOCK_HZ / 1000U - 1U)  // 23,999

// Fan timing
#define FAN_ON_DURATION_MS  10000U      // 10 seconds in milliseconds

// Fan GPIO configuration (adjust to your actual pin)
#define FAN_GPIO_PORT       GPIOA       // Example: Port A
#define FAN_GPIO_PIN        GPIO_PIN_2      // Example: Pin 2

// Function prototypes
void SystemInit_SysTick(void);
void SysTick_Handler(void);
void SystemClock_Config(void);


/* ================== adc.h ================== */
/******************************************************************************
 * File    : adc.h
 * Author  : Gautam
 * Brief   : ADC Driver
 ******************************************************************************/

/* ADC Channels */
#define ADC_CH_C_SENSE              ADC_CHANNEL_0      // PA0 (Current Sense)
#define ADC_CHANNEL_TRANSFORMER     ADC_CHANNEL_4      // PA4 (Transformer NTC)
#define ADC_CH_CV_SENSE             ADC_CHANNEL_5      // PA5 (Voltage Sense)
#define ADC_CHANNEL_MOSFET          ADC_CHANNEL_6      // PA6 (MOSFET NTC)

/* ADC Resolution */
#define ADC_MAX_VALUE               4095.0f

/* ADC Reference Voltage */
#define ADC_REFERENCE_VOLTAGE       5.0f

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/* Initialize ADC peripheral */
void ADC_Driver_Init(void);

/* Read raw ADC value from selected channel */
uint16_t ADC_ReadChannel(uint32_t channel);

/* Read voltage from selected channel */
float ADC_ReadVoltage(uint32_t channel);

/* Convenience APIs */
uint16_t ADC_ReadMosfetRaw(void);
uint16_t ADC_ReadTransformerRaw(void);

float ADC_ReadMosfetVoltage(void);
float ADC_ReadTransformerVoltage(void);


/* ================== battery_detect.h ================== */
/* Threshold for detecting a battery (in volts) */
#define BATTERY_DETECT_MIN_V (25.0f)

/*
 * Initialize the battery detection module.
 * Ensures the ADC is initialized for CV sensing.
 */
void Battery_Detect_Init(void);

/*
 * Returns true if a battery is detected at the output,
 * false otherwise.
 */
bool Battery_IsDetected(void);

/*
 * Returns true if the battery is fully charged or disconnected,
 * based on the output voltage rising to the open-circuit level.
 */
bool Battery_IsDisconnectedOrFull(void);


/* ================== fan_control.h ================== */
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


/* ================== ntc.h ================== */
/******************************************************************************
 * File    : ntc.h
 * Author  : Gautam
 * Brief   : NTC Thermistor Driver
 *
 * Device:
 *      MF11-103 (10k NTC)
 *
 * Hardware:
 *
 *      +5V
 *       |
 *      NTC
 *       |
 *       +------> ADC (PA4 / PA6)
 *       |
 *      10k
 *       |
 *      GND
 *
 ******************************************************************************/

/******************************************************************************
 * Configuration
 ******************************************************************************/

/* NTC Nominal Resistance (@25°C) */
#define NTC_R25                    (10000.0f)

/* Fixed Divider Resistor */
#define NTC_PULLDOWN_RESISTOR      (10000.0f)

/* Supply Voltage */
#define NTC_SUPPLY_VOLTAGE         (5.0f)

/* ADC Reference Voltage */
#define NTC_ADC_REFERENCE          (3.3f)

/* Beta Constant (MF11-103) */
#define NTC_BETA                   (3950.0f)

/* Reference Temperature */
#define NTC_T25_KELVIN             (298.15f)

/******************************************************************************
 * ADC Channels
 ******************************************************************************/

#define NTC_MOSFET_CHANNEL         ADC_CHANNEL_MOSFET
#define NTC_TRANSFORMER_CHANNEL    ADC_CHANNEL_TRANSFORMER

/******************************************************************************
 * Types
 ******************************************************************************/

typedef struct
{
    uint16_t adc;
    float temperature;

}NTC_Table_t;

/******************************************************************************
 * Initialization
 ******************************************************************************/

void NTC_Init(void);

/******************************************************************************
 * Raw ADC
 ******************************************************************************/

uint16_t NTC_ReadMosfetADC(void);

uint16_t NTC_ReadTransformerADC(void);

/******************************************************************************
 * Voltage
 ******************************************************************************/

float NTC_ReadMosfetVoltage(void);

float NTC_ReadTransformerVoltage(void);

/******************************************************************************
 * Resistance
 ******************************************************************************/

float NTC_ADCToResistance(uint16_t adc);

float NTC_GetMosfetResistance(void);

float NTC_GetTransformerResistance(void);

/******************************************************************************
 * Temperature
 ******************************************************************************/

float NTC_GetMosfetTemperature(void);

float NTC_GetTransformerTemperature(void);

float NTC_GetMaximumTemperature(void);

/******************************************************************************
 * Utility
 ******************************************************************************/

float NTC_GetTemperatureFromADC(uint16_t adc);


/* ================== output_control.h ================== */
// ============================================================================
// output_control.h
//
// Timed active-low output control for PA1
//
// Normal state:
//     PA1 = HIGH
//
// Active timed state:
//     PA1 = LOW for 60 seconds
//
// After timeout:
//     PA1 = HIGH
// ============================================================================




/* ================== thermal.h ================== */
/******************************************************************************
 * File    : thermal.h
 * Author  : Gautam Sharma
 * Brief   : Thermal Protection Module
 ******************************************************************************/

/******************************************************************************
 * Configuration
 ******************************************************************************/

/* Thermal Thresholds (°C) */

#define THERM_DERATE_START_TEMP      (80.0f)

#define THERM_SHUTDOWN_TEMP          (90.0f)

#define THERM_RECOVERY_TEMP          (60.0f)

/******************************************************************************
 * Thermal States
 ******************************************************************************/

typedef enum
{
    THERM_STATE_NORMAL = 0,

    THERM_STATE_DERATING,

    THERM_STATE_FAULT

}THERM_STATE;

/******************************************************************************
 * Thermal Information
 ******************************************************************************/

typedef struct
{
    float mosfetTemp;

    float transformerTemp;

    float maximumTemp;

    float currentScale;

    THERM_STATE state;

}THERM_DATA;

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/* Initialize Thermal Module */
void Thermal_Init(void);

/* Call every 100ms */
void Thermal_Task(void);

/******************************************************************************
 * Getters
 ******************************************************************************/

/* Current Thermal State */
THERM_STATE Thermal_GetState(void);

/* Maximum Temperature */
float Thermal_GetMaximumTemperature(void);

/* Current Scaling (0.0 ~ 1.0) */
float Thermal_GetCurrentScale(void);

/* Thermal Fault Status */
uint8_t Thermal_IsFault(void);

/* Thermal Derating Status */
uint8_t Thermal_IsDerating(void);

/* Complete Thermal Data */
THERM_DATA* Thermal_GetData(void);


/* ================== tm1637.h ================== */
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

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */