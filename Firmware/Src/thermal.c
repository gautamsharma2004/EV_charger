#include "thermal.h"
#include "ntc.h"

/* ============================================================================

 * EXPLANATION: Thermal Protection Module

 *

 * This module is a safety supervisory layer. It continuously polls the

 * temperatures from the `ntc.c` module and applies a state machine:

 *

 * - NORMAL: Temperatures are safe.

 * - DERATING: If temperatures exceed `THERM_DERATE_START_TEMP` (e.g. 80°C),

 *   it begins scaling down the maximum charge current (currentScale < 1.0)

 *   to reduce heat generation without stopping the charge completely.

 * - FAULT: If temperatures exceed `THERM_SHUTDOWN_TEMP` (e.g. 90°C), it

 *   triggers a hard fault, forcing the main state machine to shut down

 *   output power completely until temperatures recover below `THERM_RECOVERY_TEMP`.

 * ============================================================================ */



static THERM_DATA thermData;



void Thermal_Init(void)

{

    thermData.mosfetTemp      = 25.0f;

    thermData.transformerTemp = 25.0f;

    thermData.maximumTemp     = 25.0f;

    thermData.currentScale    = 1.0f;

    thermData.state           = THERM_STATE_NORMAL;

}



void Thermal_Task(void)

{

    thermData.mosfetTemp      = NTC_GetMosfetTemperature();

    thermData.transformerTemp = NTC_GetTransformerTemperature();

   

    float maxT = thermData.mosfetTemp;

    if (thermData.transformerTemp > maxT)

    {

        maxT = thermData.transformerTemp;

    }

    thermData.maximumTemp = maxT;

   

    switch (thermData.state)

    {

        case THERM_STATE_NORMAL:

            thermData.currentScale = 1.0f;

            if (maxT >= THERM_SHUTDOWN_TEMP)

            {

                thermData.state = THERM_STATE_FAULT;

            }

            else if (maxT >= THERM_DERATE_START_TEMP)

            {

                thermData.state = THERM_STATE_DERATING;

            }

            break;

           

        case THERM_STATE_DERATING:

            if (maxT >= THERM_SHUTDOWN_TEMP)

            {

                thermData.state = THERM_STATE_FAULT;

                thermData.currentScale = 0.0f;

            }

            else if (maxT < (THERM_DERATE_START_TEMP - 2.0f))

            {

                thermData.state = THERM_STATE_NORMAL;

                thermData.currentScale = 1.0f;

            }

            else

            {

                float range = THERM_SHUTDOWN_TEMP - THERM_DERATE_START_TEMP;

                float over  = maxT - THERM_DERATE_START_TEMP;

                float scale = 1.0f - (over / range);

               

                if (scale < 0.1f) scale = 0.1f;

                if (scale > 1.0f) scale = 1.0f;

                thermData.currentScale = scale;

            }

            break;

           

        case THERM_STATE_FAULT:

            thermData.currentScale = 0.0f;

            if (maxT <= THERM_RECOVERY_TEMP)

            {

                thermData.state = THERM_STATE_NORMAL;

                thermData.currentScale = 1.0f;

            }

            break;

    }

}



THERM_STATE Thermal_GetState(void) { return thermData.state; }

float Thermal_GetMaximumTemperature(void) { return thermData.maximumTemp; }

float Thermal_GetCurrentScale(void) { return thermData.currentScale; }

uint8_t Thermal_IsFault(void) { return (thermData.state == THERM_STATE_FAULT) ? 1 : 0; }

uint8_t Thermal_IsDerating(void) { return (thermData.state == THERM_STATE_DERATING) ? 1 : 0; }

THERM_DATA* Thermal_GetData(void) { return &thermData; }




