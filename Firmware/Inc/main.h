#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "py32f0xx.h"
#include "py32f0xx_hal.h"

/* ============================================================================
 * SYSTEM CLOCK CONFIGURATION
 * ============================================================================ */
// System clock from 24MHz external crystal
#define SYSTEM_CLOCK_HZ     24000000U
#define SYSTICK_FREQ_MS     1           // 1ms per tick

// SysTick configuration for 1ms interrupt
#define SYSTICK_RELOAD_VAL  (SYSTEM_CLOCK_HZ / 1000U - 1U)  // 23,999

/* ============================================================================
 * SYSTEM STATE MACHINE ENUMERATIONS
 * ============================================================================ */
typedef enum {
    STATE_INIT = 0,
    STATE_WAIT_BATTERY,
    STATE_CHARGING,
    STATE_CHARGE_COMPLETE,
    STATE_THERMAL_FAULT,
    STATE_IDLE,
    STATE_FAULT_LOCK,
    STATE_MAINS_FAULT,
    STATE_BTNG,        /* Battery Not Good */
    STATE_CHTO,        /* Charge Timeout */
    STATE_SCPT         /* Short Circuit Protection */
} ChargerState_t;

typedef enum {
    MODE_NORMAL = 0,
    MODE_DPDC,
    MODE_BTRM
} ChargeMode_t;

/* ============================================================================
 * GLOBAL VARIABLES (Extern Declarations)
 * These variables are defined in main.c but exposed here so other modules
 * (like battery_detect.c and tm1637.c) can read the real-time system state.
 * ============================================================================ */
extern volatile ChargerState_t gState;
extern volatile ChargeMode_t gChargeMode;

extern float TARGET_VOLTAGE;
extern float TARGET_CURRENT;

extern volatile float ACTIVE_TARGET_CURRENT;
extern volatile float ACTIVE_TARGET_VOLTAGE;

extern volatile float g_actual_v;
extern volatile float g_actual_i;

extern volatile uint8_t currentSOC;
extern uint32_t socEvalTimer;

/* ============================================================================
 * SYSTEM FUNCTION PROTOTYPES
 * ============================================================================ */
void SystemInit_SysTick(void);
void SysTick_Handler(void);
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */