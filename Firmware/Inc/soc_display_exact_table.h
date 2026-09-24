#ifndef SOC_DISPLAY_EXACT_TABLE_H
#define SOC_DISPLAY_EXACT_TABLE_H

#include <stdint.h>
#include <stdbool.h>

void SOC_DisplayInit(float battery_voltage, float target_voltage);
void SOC_DisplayUpdateTask(float battery_voltage, float target_voltage, uint32_t sustain_time_ms);
uint8_t SOC_DisplayGetPercentage(void);
void SOC_DisplaySetComplete(void);

#endif
