/* ================== adc.h ================== */
/******************************************************************************
 * File    : adc.h
 * Author  : Gautam
 * Brief   : ADC Driver
 ******************************************************************************/

#ifndef ADC_H
#define ADC_H
#include "py32f0xx_hal.h"

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

#endif