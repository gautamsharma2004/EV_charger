/* ============================================================================

 * EXPLANATION: NTC Thermistor Module

 *

 * This module calculates real-world temperatures from the raw ADC readings

 * of the NTC (Negative Temperature Coefficient) thermistors attached to the

 * MOSFETs and the Transformer.

 *

 * It uses the Steinhart-Hart equation (or Beta parameter equation) along with

 * a predefined resistor divider network setup to convert raw ADC voltage into

 * a Resistance, and finally into a Temperature in Celsius.

 * ============================================================================ */



#include "ntc.h"
#include "adc.h"
#include <math.h>

void NTC_Init(void) { /* ADC Init handled globally */ }

uint16_t NTC_ReadMosfetADC(void) { return ADC_ReadChannel(NTC_MOSFET_CHANNEL); }

uint16_t NTC_ReadTransformerADC(void) { return ADC_ReadChannel(NTC_TRANSFORMER_CHANNEL); }

float NTC_ReadMosfetVoltage(void) { return ADC_ReadVoltage(NTC_MOSFET_CHANNEL); }

float NTC_ReadTransformerVoltage(void) { return ADC_ReadVoltage(NTC_TRANSFORMER_CHANNEL); }



float NTC_ADCToResistance(uint16_t adc)

{

    /* 1. Open circuit protection (Prevent divide-by-zero) */

    if(adc == 0)

        return 1000000.0f;



    /* 2. Short circuit protection */

    if(adc >= 4095)

        return 1.0f;



    /* 3. Pure ratiometric resistance calculation (Matches old ntc.c exactly) */

    return NTC_PULLDOWN_RESISTOR * (4095.0f - (float)adc) / (float)adc;

}



float NTC_GetTemperatureFromADC(uint16_t adc)

{

    float resistance = NTC_ADCToResistance(adc);

    if(resistance <= 0.0f) return 0.0f;

    float rRatio = resistance / NTC_R25;

    float invT = (1.0f / NTC_T25_KELVIN) + (1.0f / NTC_BETA) * logf(rRatio);

    float kelvin = 1.0f / invT;

    return kelvin - 273.15f;

}



float NTC_GetMosfetResistance(void) { return NTC_ADCToResistance(NTC_ReadMosfetADC()); }

float NTC_GetTransformerResistance(void) { return NTC_ADCToResistance(NTC_ReadTransformerADC()); }

float NTC_GetMosfetTemperature(void) { return NTC_GetTemperatureFromADC(NTC_ReadMosfetADC()); }

float NTC_GetTransformerTemperature(void) { return NTC_GetTemperatureFromADC(NTC_ReadTransformerADC()); }



float NTC_GetMaximumTemperature(void)

{

    float t1 = NTC_GetMosfetTemperature();

    float t2 = NTC_GetTransformerTemperature();

    return (t1 > t2) ? t1 : t2;

}



