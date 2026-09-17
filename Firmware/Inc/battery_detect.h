/* ================== battery_detect.h ================== */
#ifndef BATTERY_DETECT_H
#define BATTERY_DETECT_H

#include <stdbool.h>
#include "py32f0xx_hal.h"

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


#endif