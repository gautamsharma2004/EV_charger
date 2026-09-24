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

#ifndef NTC_H
#define NTC_H

#include <stdint.h>
#include <math.h> // Required for Steinhart-Hart logf() function
#include "py32f0xx_hal.h"


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
/* ADC Channels */
#define NTC_MOSFET_CHANNEL         ADC_CHANNEL_MOSFET
#define NTC_TRANSFORMER_CHANNEL    ADC_CHANNEL_TRANSFORMER

/* Function Prototypes */
void NTC_Init(void);
uint16_t NTC_ReadMosfetADC(void);
uint16_t NTC_ReadTransformerADC(void);
float NTC_ReadMosfetVoltage(void);
float NTC_ReadTransformerVoltage(void);
float NTC_ADCToResistance(uint16_t adc);
float NTC_GetTemperatureFromADC(uint16_t adc);
float NTC_GetMosfetResistance(void);
float NTC_GetTransformerResistance(void);
float NTC_GetMosfetTemperature(void);
float NTC_GetTransformerTemperature(void);
float NTC_GetMaximumTemperature(void);
#endif
