#ifndef PI_CONTROL_H
#define PI_CONTROL_H

#include <stdint.h>

/* ============================================================================
 * PI Controller Module
 * ============================================================================ */

typedef struct {
    float Kp;
    float Ki;
    float integral_sum;
    float out_max;
    float out_min;
} PI_Controller;

/*
 * Calculates the PI loop response.
 * Parameters:
 *   pi: Pointer to the PI_Controller instance
 *   setpoint: The target value (voltage or current)
 *   actual: The current measured value
 * Returns: The calculated control output (like the PWM duty cycle)
 */
float Calculate_PI(PI_Controller *pi, float setpoint, float actual);

#endif 