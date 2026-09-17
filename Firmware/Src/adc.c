/* ============================================================================

 * EXPLANATION: ADC (Analog-to-Digital Converter) Module

 *

 * This section is responsible for reading the analog voltages from the hardware

 * and converting them into digital values. The firmware uses the ADC for multiple

 * critical functions:

 *   1. Measuring Output Voltage (CV - Constant Voltage feedback)

 *   2. Measuring Output Current (CC - Constant Current feedback)

 *   3. Measuring NTC Thermistors (Temperature sensing for MOSFETs and Transformer)

 *

 * The `ADC_ReadVoltage` function converts raw 12-bit ADC values (0-4095) back

 * to actual voltage levels based on the reference voltage (e.g. 5.0V).

 * ============================================================================ */

#include "adc.h"

static ADC_HandleTypeDef hadc;



void ADC_Driver_Init(void)

{

    GPIO_InitTypeDef GPIO_InitStruct = {0};



    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_ADC_CLK_ENABLE();



    /* Configure PA4 (Transformer NTC), PA5 (CV Battery Detect), and PA6 (MOSFET NTC) as Analog */

    GPIO_InitStruct.Pin =

            GPIO_PIN_0 |

            GPIO_PIN_4 |

            GPIO_PIN_5 |

            GPIO_PIN_6;



    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

    GPIO_InitStruct.Pull = GPIO_NOPULL;



    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



    /* ADC Configuration */

    hadc.Instance = ADC1;



    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;

    hadc.Init.Resolution            = ADC_RESOLUTION_12B;

    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;

    hadc.Init.ScanConvMode          = 0;

    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    hadc.Init.LowPowerAutoWait      = DISABLE;

    hadc.Init.ContinuousConvMode    = DISABLE;

    hadc.Init.DiscontinuousConvMode = DISABLE;

    hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;

    hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;

    hadc.Init.Overrun               = ADC_OVR_DATA_PRESERVED;

    hadc.Init.SamplingTimeCommon    = ADC_SAMPLETIME_239CYCLES_5;



    HAL_ADC_Init(&hadc);

    HAL_ADCEx_Calibration_Start(&hadc);

}



uint16_t ADC_ReadChannel(uint32_t channel)

{

    ADC_ChannelConfTypeDef sConfig = {0};



    sConfig.Channel = channel;

    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;



    hadc.Instance->CHSELR = 0;

    HAL_ADC_ConfigChannel(&hadc,&sConfig);



    for(volatile int i=0; i<100; i++)

    {

        __NOP();

    }



    HAL_ADC_Start(&hadc);



    // Check if the conversion actually succeeds

    if (HAL_ADC_PollForConversion(&hadc, 100) != HAL_OK)

    {

        HAL_ADC_Stop(&hadc);

        return 0;/* Silently discard noise timeout to prevent 49A spikes */

    }



    uint16_t value = HAL_ADC_GetValue(&hadc);

    HAL_ADC_Stop(&hadc);

    return value;

}



float ADC_ReadVoltage(uint32_t channel)

{

    uint16_t adc = ADC_ReadChannel(channel);

    return ((float)adc / ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE;

}



uint16_t ADC_ReadMosfetRaw(void) { return ADC_ReadChannel(ADC_CHANNEL_MOSFET); }

uint16_t ADC_ReadTransformerRaw(void) { return ADC_ReadChannel(ADC_CHANNEL_TRANSFORMER); }

float ADC_ReadMosfetVoltage(void) { return ADC_ReadVoltage(ADC_CHANNEL_MOSFET); }

float ADC_ReadTransformerVoltage(void) { return ADC_ReadVoltage(ADC_CHANNEL_TRANSFORMER); }



