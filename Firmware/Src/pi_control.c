#include "pi_control.h"

/* ============================================================================
 * PI Controller Implementation
 * ============================================================================ */

float Calculate_PI(PI_Controller *pi, float setpoint, float actual)
{
    float error = actual - setpoint;
    
    /* ASYMMETRICAL CLAMP: Prevent violent power surges, but allow instant shutoff */
    if (error < -2.0f) error = -2.0f; 
    
    float proportional = pi->Kp * error;
    pi->integral_sum += (pi->Ki * error);
    
    if (pi->integral_sum > pi->out_max) pi->integral_sum = pi->out_max;
    if (pi->integral_sum < pi->out_min) pi->integral_sum = pi->out_min;
    
    float output = proportional + pi->integral_sum;
    
    if (output > pi->out_max) output = pi->out_max;
    if (output < pi->out_min) output = pi->out_min;
    
    return output;
}