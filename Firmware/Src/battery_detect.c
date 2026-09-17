/* ============================================================================

 * EXPLANATION: Battery Detection Module

 *

 * This module contains the logic to detect whether a battery is physically

 * connected to the charger output.

 *

 * It works by observing the output voltage. When the charger is in the

 * WAIT_BATTERY state, it applies an initial output. If the voltage drops

 * to the battery's resting voltage (above a minimum threshold), it detects

 * the battery and proceeds to charge. It also contains logic to detect if

 * the battery has been disconnected or is fully charged (the voltage will

 * float up to the open-circuit maximum voltage).

 * ============================================================================ */


#include "battery_detect.h"
#include "adc.h"
#include "main.h"

void Battery_Detect_Init(void)

{

    // ADC is already initialized in main, which covers CV_SENSE.

}



bool Battery_IsDetected(void)
{
    float raw_voltage = ADC_ReadVoltage(ADC_CH_CV_SENSE);
    float output_voltage = raw_voltage * 17.34f;
    
    /* PASSIVE DETECTION: If the charger is OFF, any voltage > 15V means a battery was plugged in */
    if (output_voltage >= 15.0f) {
        return true;
    }
    return false;
}

bool Battery_IsDisconnectedOrFull(void)
{
    float raw_voltage = ADC_ReadVoltage(ADC_CH_CV_SENSE);
    float output_voltage = raw_voltage * 17.34f;
    return (output_voltage >= (TARGET_VOLTAGE - 0.2f) && g_actual_i < 0.2f);
}

