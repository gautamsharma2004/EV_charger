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

#endif