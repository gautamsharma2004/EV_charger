/* ================== thermal.h ================== */
/******************************************************************************
 * File    : thermal.h
 * Author  : Gautam Sharma
 * Brief   : Thermal Protection Module
 ******************************************************************************/
#ifndef THERMAL_H
#define THERMAL_H

#include <stdint.h>
#include "py32f0xx_hal.h"

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


#endif