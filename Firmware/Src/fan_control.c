/* ============================================================================

 * EXPLANATION: Fan Control Module

 *

 * This section manages the cooling fan. It provides simple On/Off/Toggle

 * functions (`Fan_On`, `Fan_Off`) to actuate the GPIO pin connected to the fan

 * relay or transistor.

 *

 * The fan is typically turned on when charging starts, or when thermal limits

 * are reached to actively cool the power electronics, preventing over-temperature

 * faults.

 * ============================================================================ */

#include "fan_control.h"


void Fan_Init(void)

{

    __HAL_RCC_GPIOA_CLK_ENABLE();

   

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = FAN_GPIO_PIN;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

   

    HAL_GPIO_Init(FAN_GPIO_PORT, &GPIO_InitStruct);

   

    Fan_Off(); // Ensure fan is off initially

}


void Fan_On(void) { HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_GPIO_PIN, GPIO_PIN_SET); }
void Fan_Off(void) { HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_GPIO_PIN, GPIO_PIN_RESET); }
void Fan_Toggle(void) { HAL_GPIO_TogglePin(FAN_GPIO_PORT, FAN_GPIO_PIN); }
