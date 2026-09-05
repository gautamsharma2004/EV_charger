/* ============================================================================
 * MERGED APPLICATION SOURCE CODE
 * ============================================================================ */
#include "main.h"

#include <math.h>
uint32_t socEvalTimer = 0;
volatile uint8_t currentSOC = 0;
float TARGET_VOLTAGE = 67.20f;
float TARGET_CURRENT = 6.2f;  
/* SOC to Vset Percentage Mapping Table (Index 0 = 0%, Index 1 = 5% ... Index 20 = 100%) */
const float SOC_VSET_PCT[21] = {
    0.650f, // 0%
    0.803f, // 5%
    0.826f, // 10%
    0.849f, // 15%
    0.872f, // 20%
    0.895f, // 25%
    0.900f, // 30%
    0.905f, // 35%
    0.910f, // 40%
    0.915f, // 45%
    0.920f, // 50%
    0.925f, // 55%
    0.930f, // 60%
    0.935f, // 65%
    0.940f, // 70%
    0.945f, // 75%
    0.956f, // 80%
    0.967f, // 85%
    0.978f, // 90%
    0.989f, // 95%
    1.000f  // 100%
};
uint32_t totalChargeTimer = 0;
uint32_t cvSafetyTimer = 0;
bool cvSafetyTimerActive = false;
volatile float ACTIVE_TARGET_CURRENT = 0.0f; // The PI loop follows this, NOT TARGET_CURRENT
volatile float ACTIVE_TARGET_VOLTAGE = 0.0f;
volatile float g_actual_v = 0.0f;

volatile float g_actual_i = 0.0f;



#define NTC_R25                 10000.0f    // 10k NTC

#define NTC_PULLDOWN_RESISTOR   10000.0f    // 10k Pull-down

#define NTC_BETA                3950.0f     // Standard NTC Beta

#define NTC_T25_KELVIN          298.15f     // 25°C in Kelvin

#define CURRENT_CAL_FACTOR 1.000f  /* MUST BE RESTORED */

#define VOLTAGE_CAL_FACTOR 0.882f



typedef struct {

    float Kp;

    float Ki;

    float integral_sum;

    float out_max;

    float out_min;

} PI_Controller;


/* Initialize integral sums to max (1199) to force TL494 to zero output on startup */
PI_Controller cv_pi = { .Kp = 20.0f, .Ki = 0.5f, .integral_sum = 1199.0f, .out_max = 1199.0f, .out_min = 0.0f };
PI_Controller cc_pi = { .Kp = 10.0f, .Ki = 1.0f, .integral_sum = 1199.0f, .out_max = 1199.0f, .out_min = 0.0f };



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



/* ============================================================================

 * EXPLANATION: NTC Thermistor Module

 *

 * This module calculates real-world temperatures from the raw ADC readings

 * of the NTC (Negative Temperature Coefficient) thermistors attached to the

 * MOSFETs and the Transformer.

 *

 * It uses the Steinhart-Hart equation (or Beta parameter equation) along with

 * a predefined resistor divider network setup to convert raw ADC voltage into

 * a Resistance, and finally into a Temperature in Celsius.

 * ============================================================================ */



void NTC_Init(void) { /* ADC Init handled globally */ }

uint16_t NTC_ReadMosfetADC(void) { return ADC_ReadChannel(NTC_MOSFET_CHANNEL); }

uint16_t NTC_ReadTransformerADC(void) { return ADC_ReadChannel(NTC_TRANSFORMER_CHANNEL); }

float NTC_ReadMosfetVoltage(void) { return ADC_ReadVoltage(NTC_MOSFET_CHANNEL); }

float NTC_ReadTransformerVoltage(void) { return ADC_ReadVoltage(NTC_TRANSFORMER_CHANNEL); }



float NTC_ADCToResistance(uint16_t adc)

{

    /* 1. Open circuit protection (Prevent divide-by-zero) */

    if(adc == 0)

        return 1000000.0f;



    /* 2. Short circuit protection */

    if(adc >= 4095)

        return 1.0f;



    /* 3. Pure ratiometric resistance calculation (Matches old ntc.c exactly) */

    return NTC_PULLDOWN_RESISTOR * (4095.0f - (float)adc) / (float)adc;

}



float NTC_GetTemperatureFromADC(uint16_t adc)

{

    float resistance = NTC_ADCToResistance(adc);

    if(resistance <= 0.0f) return 0.0f;

    float rRatio = resistance / NTC_R25;

    float invT = (1.0f / NTC_T25_KELVIN) + (1.0f / NTC_BETA) * logf(rRatio);

    float kelvin = 1.0f / invT;

    return kelvin - 273.15f;

}



float NTC_GetMosfetResistance(void) { return NTC_ADCToResistance(NTC_ReadMosfetADC()); }

float NTC_GetTransformerResistance(void) { return NTC_ADCToResistance(NTC_ReadTransformerADC()); }

float NTC_GetMosfetTemperature(void) { return NTC_GetTemperatureFromADC(NTC_ReadMosfetADC()); }

float NTC_GetTransformerTemperature(void) { return NTC_GetTemperatureFromADC(NTC_ReadTransformerADC()); }



float NTC_GetMaximumTemperature(void)

{

    float t1 = NTC_GetMosfetTemperature();

    float t2 = NTC_GetTransformerTemperature();

    return (t1 > t2) ? t1 : t2;

}





/* ============================================================================

 * EXPLANATION: Output Enable/Disable Control Module

 *

 * This manages the main power output stage of the charger. It controls a GPIO

 * pin that enables or disables the power switching circuitry.

 *

 * It also contains a timer task mechanism that can enforce a timed output

 * (e.g., active for 60 seconds and then automatically turning off), which is

 * useful for safety timeouts or initialization phases.

 * ============================================================================ */









typedef enum {

    UI_STATE_NORMAL,

    UI_STATE_VSET,

    UI_STATE_CSET,

    UI_STATE_SAVE_DISPLAY_V,

    UI_STATE_SAVE_DISPLAY_C

} UI_State_t;



const float VSET_ARRAY[] = {54.6f, 54.75f, 58.4f, 58.8f, 67.2f, 67.35f, 69.35f, 71.4f, 73.0f, 83.95f, 84.0f, 87.6f};
const float CSET_ARRAY[] = {6.2f, 7.2f, 8.2f, 9.2f, 10.2f};
#define VSET_COUNT (sizeof(VSET_ARRAY) / sizeof(VSET_ARRAY[0]))
#define CSET_COUNT (sizeof(CSET_ARRAY) / sizeof(CSET_ARRAY[0]))

UI_State_t current_ui_state = UI_STATE_NORMAL;
uint8_t vset_index = 4; // Defaults to 67.2V
uint8_t cset_index = 0; // Defaults to 6.2A

uint32_t button_press_start = 0;
uint32_t last_activity_time = 0;
uint32_t save_display_timer = 0;
bool button_held = false;

/* Display Data Maps */
const uint8_t DISP_HITP[4] = {0x76, 0x06, 0x78, 0x73}; // H I T P
const uint8_t DISP_CHTO[4] = {0x39, 0x76, 0x78, 0x3F}; // C H T O
const uint8_t DISP_BTNG[4] = {0x7C, 0x78, 0x54, 0x3D}; // b t n G
const uint8_t DISP_DPDC[4] = {0x5E, 0x73, 0x5E, 0x39}; // d P d C
const uint8_t DISP_BTRM[4] = {0x7C, 0x78, 0x50, 0x54}; // b t r n (M)
const uint8_t DISP_100P[4] = {0x06, 0x3F, 0x3F, 0x73}; // 1 0 0 P
const uint8_t DISP_SCPT[4] = {0x6D, 0x39, 0x73, 0x78}; // S C P t
const uint8_t DISP_POFF[4] = {0x73, 0x40, 0x3F, 0x71}; // P - O F

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


volatile ChargerState_t gState = STATE_INIT;





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





/* ============================================================================

 * EXPLANATION: TM1637 7-Segment Display Module

 *

 * This module bit-bangs the protocol to communicate with a TM1637 7-segment

 * display driver. It is used as the primary User Interface (UI).

 *

 * It contains functions to format and display specific information such as

 * Charge Time (minutes and seconds), Voltage, Current, or Error Fault Codes

 * so the user can visibly monitor the charger's status.

 * ============================================================================ */



static const uint8_t TM1637_DigitMap[] = {

    0x3F, 0x06, 0x5B, 0x4F,

    0x66, 0x6D, 0x7D, 0x07,

    0x7F, 0x6F, 0x77, 0x7C,

    0x39, 0x5E, 0x79, 0x71

};

static uint8_t TM1637_Brightness = 0x07;



static void TM1637_Delay(void)

{

    for(volatile int i=0; i<100; i++){ __NOP(); }

}



static void TM1637_Start(void)

{

    HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);

    TM1637_Delay();

    HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_RESET);

    TM1637_Delay();

    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);

}



static void TM1637_Stop(void)

{

    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_RESET);

    TM1637_Delay();

    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);

    TM1637_Delay();

    HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_SET);

}



static uint8_t TM1637_WriteByte(uint8_t data)

{

    uint8_t ack;

    for(int i=0; i<8; i++)

    {

        HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);

        TM1637_Delay();

        if(data & 0x01)

            HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_SET);

        else

            HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_RESET);

        TM1637_Delay();

        HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);

        TM1637_Delay();

        data >>= 1;

    }

    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);

   

    // Configure DIO as input for ACK

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = TM1637_DIO_PIN;

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(TM1637_DIO_PORT, &GPIO_InitStruct);

   

    TM1637_Delay();

    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);

    TM1637_Delay();

   

    ack = HAL_GPIO_ReadPin(TM1637_DIO_PORT, TM1637_DIO_PIN);

   

    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);

   

    // Reconfigure DIO as output

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    HAL_GPIO_Init(TM1637_DIO_PORT, &GPIO_InitStruct);

   

    return ack;

}



void TM1637_Init(void)

{

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = TM1637_CLK_PIN | TM1637_DIO_PIN;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;

    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   

    HAL_GPIO_WritePin(GPIOB, TM1637_CLK_PIN | TM1637_DIO_PIN, GPIO_PIN_SET);

    TM1637_SetBrightness(0x07);

    TM1637_Clear();

}



void TM1637_SetBrightness(uint8_t level) { TM1637_Brightness = level & 0x07; }



void TM1637_Clear(void)

{

    uint8_t data[4] = {0, 0, 0, 0};

    TM1637_DisplayRaw(data);

}



void TM1637_DisplayRaw(uint8_t data[4])
{
    /* ANTI-FLICKER CACHE: Only write to the I2C bus if the digits changed */
    static uint8_t last_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    if (data[0] == last_data[0] && data[1] == last_data[1] && 
        data[2] == last_data[2] && data[3] == last_data[3]) {
        return; 
    }
    last_data[0] = data[0]; last_data[1] = data[1]; 
    last_data[2] = data[2]; last_data[3] = data[3];

    TM1637_Start();
    TM1637_WriteByte(TM1637_CMD_SETDATA);
    TM1637_Stop();
    
    TM1637_Start();
    TM1637_WriteByte(TM1637_CMD_ADDRESS);
    for(int i=0; i<4; i++)
        TM1637_WriteByte(data[i]);
    TM1637_Stop();
    
    TM1637_Start();
    TM1637_WriteByte(TM1637_CMD_DISPLAY | 0x08 | TM1637_Brightness);
    TM1637_Stop();
}



void TM1637_DisplayNumber(uint16_t number)

{

    uint8_t data[4];

    data[0] = TM1637_DigitMap[(number / 1000) % 10];

    data[1] = TM1637_DigitMap[(number / 100) % 10];

    data[2] = TM1637_DigitMap[(number / 10) % 10];

    data[3] = TM1637_DigitMap[number % 10];

    TM1637_DisplayRaw(data);

}



void TM1637_DisplayVoltage(uint16_t value) { TM1637_DisplayNumber(value); }

void TM1637_DisplayCurrent(uint16_t value) { TM1637_DisplayNumber(value); }



void TM1637_DisplayTime(uint8_t minutes, uint8_t seconds, uint8_t showColon)

{

    uint8_t data[4];

    data[0] = TM1637_DigitMap[(minutes / 10) % 10];

    data[1] = TM1637_DigitMap[minutes % 10];

    data[2] = TM1637_DigitMap[(seconds / 10) % 10];

    data[3] = TM1637_DigitMap[seconds % 10];

   

    if (showColon) data[1] |= 0x80;

    TM1637_DisplayRaw(data);

}



void TM1637_DisplayFault(void)

{

    uint8_t data[4];

    data[0] = 0x71; // F

    data[1] = 0x77; // A

    data[2] = 0x3E; // U (approximated)

    data[3] = 0x38; // L

    TM1637_DisplayRaw(data);

}



/* ============================================================================

 * EXPLANATION: Main Application Logic and State Machine

 *

 * This is the central hub of the firmware. It brings all the modules together.

 *

 * Key components:

 * 1. High-Speed CC/CV Loop: A fast control loop that regulates Output Current (CC)

 *    and Output Voltage (CV). It continuously compares the ADC feedback to target

 *    thresholds and manipulates the PWM/DTC output pin to increase or decrease power.

 *

 * 2. 100ms State Machine:

 *    - STATE_INIT: Startup initialization.

 *    - STATE_WAIT_BATTERY: Provides a probing voltage, waits for a battery to be connected.

 *    - STATE_CHARGING: Actively regulating power to charge the battery. Enforces

 *      thermal safety limits and monitors for full charge/disconnect.

 *    - STATE_IDLE: Charger is asleep. It monitors the output for voltage spikes

 *      or plunges (dV/dt) that indicate a battery was just plugged in, which

 *      wakes it back up.

 *    - STATE_THERMAL_FAULT: Charger is disabled to protect hardware from overheating.

 * ============================================================================ */



typedef enum {
    MODE_NORMAL = 0,
    MODE_DPDC,
    MODE_BTRM
} ChargeMode_t;

volatile ChargeMode_t gChargeMode = MODE_NORMAL;

volatile uint8_t gOutputActive = 0;



 void SystemClock_Config(void);

void PWM_Hardware_Init(void);


static void Output_Enable(void)

{

    gOutputActive = 1;

}



static void Output_Disable(void)

{

    gOutputActive = 0;

    /* The SysTick ISR will handle writing 1199 to CCR4 within 1 millisecond */

}

/* ============================================================================

 * VIRTUAL EEPROM MODULE (FLASH EMULATION)

 * ============================================================================ */



#define FLASH_EEPROM_PAGE_ADDR 0x08005800

#define EEPROM_MAGIC_NUMBER    0xAABBCCDD



void EEPROM_Save(uint8_t v_idx, uint8_t c_idx)

{

    uint32_t PageError = 0;

    FLASH_EraseInitTypeDef EraseInitStruct;

    uint32_t page_buffer[32];

   

    page_buffer[0] = EEPROM_MAGIC_NUMBER;

    page_buffer[1] = (uint32_t)v_idx;

    page_buffer[2] = (uint32_t)c_idx;

   

    for(int i = 3; i < 32; i++) {

        page_buffer[i] = 0xFFFFFFFF;

    }



    HAL_FLASH_Unlock();

   

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGEERASE; /* FIXED FOR PY32 */

    EraseInitStruct.PageAddress = FLASH_EEPROM_PAGE_ADDR;

    EraseInitStruct.NbPages = 1;

   

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK)

    {

        /* FIXED FOR PY32: Passing pointer directly */

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, FLASH_EEPROM_PAGE_ADDR, page_buffer);

    }

   

    HAL_FLASH_Lock();

}



void EEPROM_Load(void)

{

    uint32_t magic = *(__IO uint32_t*)FLASH_EEPROM_PAGE_ADDR;



    if (magic == EEPROM_MAGIC_NUMBER)

    {

        vset_index = (uint8_t)(*(__IO uint32_t*)(FLASH_EEPROM_PAGE_ADDR + 4));

        cset_index = (uint8_t)(*(__IO uint32_t*)(FLASH_EEPROM_PAGE_ADDR + 8));

       

        if (vset_index >= VSET_COUNT) vset_index = 4;

        if (cset_index >= CSET_COUNT) cset_index = 0;

    }

    else

    {

        vset_index = 4; // Default 58.4V

        cset_index = 0; // Default 6.2A

        EEPROM_Save(vset_index, cset_index);

    }

   

    TARGET_VOLTAGE = VSET_ARRAY[vset_index];

    TARGET_CURRENT = CSET_ARRAY[cset_index];

}

void UI_Task(void)

{

    // Read the active-low button state (SW_V connected to PB1)

    bool button_active = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET);

    uint32_t current_time = HAL_GetTick();



    if (button_active) {

        if (button_press_start == 0) {

            button_press_start = current_time;

        } else if ((current_time - button_press_start) >= 2000 && !button_held) {

            button_held = true;

            last_activity_time = current_time;

           

            if (current_ui_state == UI_STATE_NORMAL) {

                current_ui_state = UI_STATE_VSET;

            } else if (current_ui_state == UI_STATE_VSET) {

                current_ui_state = UI_STATE_CSET;

            }

        }

    } else {

        if (button_press_start != 0) {

            uint32_t press_duration = current_time - button_press_start;

            if (press_duration < 2000 && press_duration > 50) { // 50ms debounce

                last_activity_time = current_time;

                if (current_ui_state == UI_STATE_VSET) {

                    vset_index = (vset_index + 1) % VSET_COUNT;

                } else if (current_ui_state == UI_STATE_CSET) {

                    cset_index = (cset_index + 1) % CSET_COUNT;

                }

            }

            button_press_start = 0;

            button_held = false;

        }

    }



    switch (current_ui_state) {

        case UI_STATE_VSET:

            if ((current_time - last_activity_time) >= 8000) {

                current_ui_state = UI_STATE_SAVE_DISPLAY_V;

                save_display_timer = current_time;

            } else {

                TM1637_DisplayNumber((uint16_t)(VSET_ARRAY[vset_index] * 10));

            }

            break;



        case UI_STATE_CSET:

            if ((current_time - last_activity_time) >= 8000) {

                current_ui_state = UI_STATE_SAVE_DISPLAY_V;

                save_display_timer = current_time;

            } else {

                TM1637_DisplayNumber((uint16_t)(CSET_ARRAY[cset_index] * 10));

            }

            break;



        case UI_STATE_SAVE_DISPLAY_V:

            TM1637_DisplayNumber((uint16_t)(VSET_ARRAY[vset_index] * 10));

            if ((current_time - save_display_timer) >= 2000) {

                current_ui_state = UI_STATE_SAVE_DISPLAY_C;

                save_display_timer = current_time;

            }

            break;



        case UI_STATE_SAVE_DISPLAY_C:

            TM1637_DisplayNumber((uint16_t)(CSET_ARRAY[cset_index] * 10));

            if ((current_time - save_display_timer) >= 2000) {

                // Apply the new targets to the PI Loop

                TARGET_VOLTAGE = VSET_ARRAY[vset_index];

                TARGET_CURRENT = CSET_ARRAY[cset_index];

                /* COMMIT TO FLASH MEMORY */

                EEPROM_Save(vset_index, cset_index);

                current_ui_state = UI_STATE_NORMAL;

               

                // Note: Flash Save routine must be called here once you implement EEPROM emulation

            }

            break;

           

        case UI_STATE_NORMAL:

        default:

            break;

    }

}

void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
   
    /* Initialize PB3 for UI Button */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Initialize PA3 for PC817 UV/OV Protection Optocoupler */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* External 1k pull-up exists */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void SOC_Evaluation_Task(void)
{
    /* 1. Do not evaluate if already fully charged or locked */
    if (currentSOC >= 100 || gState == STATE_CHARGE_COMPLETE) {
        return; 
    }

    /* 2. Determine the next +5% target threshold */
    uint8_t next_soc = currentSOC + 5;
    
    /* Hard cap the voltage-based increment at 95% */
    /* 100% is strictly reserved for the CV termination condition */
    if (next_soc > 95) {
        next_soc = 95;
    }

    uint8_t target_index = next_soc / 5;
    float required_threshold_v = TARGET_VOLTAGE * SOC_VSET_PCT[target_index];

    /* 3. Evaluate continuous voltage threshold over 60s */
    if (g_actual_v >= required_threshold_v) {
        if ((HAL_GetTick() - socEvalTimer) >= 60000) {
            /* Passed 60s qualification. Apply the exact 5% increment. */
            currentSOC = next_soc;
            socEvalTimer = HAL_GetTick(); 
        }
    } else {
        /* Voltage dropped below the threshold, reset the 60s qualification timer */
        socEvalTimer = HAL_GetTick();
    }
}

void SOC_Calculate_Initial(void)
{
    float current_v_pct = g_actual_v / TARGET_VOLTAGE;
    currentSOC = 0;
    
    /* Iterate backwards from 95% to find the highest passing threshold */
    for (int i = 19; i >= 0; i--) {
        if (current_v_pct >= SOC_VSET_PCT[i]) {
            currentSOC = i * 5;
            break;
        }
    }
    socEvalTimer = HAL_GetTick(); // Prime the timer for the next increment
}

int main(void)

{

    uint32_t lastUpdate = 0;

    uint32_t waitTimer =  0;



    float previousVoltage = 0.0f;
/* ------------------------------------------------------------------------
     * IMMEDIATE HARDWARE CLAMP
     * Force PA1 HIGH instantly to keep TL494 OFF during the boot delay.
     * ------------------------------------------------------------------------ */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
   /* 1. HARDWARE POWER-RAMP DELAY */
    /* Force the CPU to blindly stall while the Mains SMPS slowly ramps to 5V */
    for(volatile uint32_t wait = 0; wait < 1500000; wait++) { __NOP(); }

    HAL_Init();

    SystemClock_Config();

    SystemCoreClockUpdate();
/* PWM_Hardware_Init will safely override the GPIO setting to Alternate Function */
    PWM_Hardware_Init();

    GPIO_Init();

    ADC_Driver_Init();

    NTC_Init();

    Thermal_Init();

    TM1637_Init();

    Fan_Init();

    Battery_Detect_Init();

    EEPROM_Load(); //  saved last Vset and Cset

    TM1637_Clear();

   

    gState = STATE_INIT;

    
    /* PRIME FILTERS: Prevent PI loop from winding up due to initial 0V variables */
    uint32_t last_pi_time = 0;
    bool filter_initialized = false;

  while(1)
    {

       /* 1. High-Speed ADC Polling Task (With Calibration Applied) */
        float raw_i = (ADC_ReadVoltage(ADC_CH_C_SENSE) / 0.23f) * CURRENT_CAL_FACTOR; 
        raw_i -= 1.0f; /* FIX: Cancel the 1.0A op-amp zero-load baseline offset */
        if (raw_i < 0.0f) raw_i = 0.0f;
        
        float raw_v = (ADC_ReadVoltage(ADC_CH_CV_SENSE) * 17.34f) * VOLTAGE_CAL_FACTOR;
        /* 
         * Digital Low-Pass Filter: 
         * Used for State Machine (100P termination) and UI Display to ignore noise.
         */
        if (!filter_initialized) {
            g_actual_i = raw_i;
            g_actual_v = raw_v;
            filter_initialized = true;
        } else {
            g_actual_i = (g_actual_i * 0.90f) + (raw_i * 0.10f);
            g_actual_v = (g_actual_v * 0.95f) + (raw_v * 0.05f);
        }
       /* 2. PI Control Loop (Runs strictly every 1ms in Main Context) */
        if ((HAL_GetTick() - last_pi_time) >= 1)
        {
            last_pi_time = HAL_GetTick();
            static float active_pwm = 1199.0f; /* DECLARED ONCE HERE */

            if (gOutputActive && gState != STATE_MAINS_FAULT)
            {
                /* SCPT: 150% CSET Threshold */
                if (g_actual_i > (TARGET_CURRENT * 1.5f)) { 
                    active_pwm = 1199.0f;
                    TIM1->CCR4 = 1199; 
                    cv_pi.integral_sum = 1199.0f; 
                    cc_pi.integral_sum = 1199.0f;
                    ACTIVE_TARGET_CURRENT = 0.5f; 
                    gState = STATE_SCPT; /* Trigger Fault State */
                }
               /* DEAD-ZONE SEARCH (Bounded to prevent hardware runaway) */
                else if (g_actual_i < 0.1f && ACTIVE_TARGET_CURRENT > 0.1f && g_actual_v < (ACTIVE_TARGET_VOLTAGE - 0.5f)) {
                    active_pwm -= 0.2f; 
                    if (active_pwm < 0.0f) active_pwm = 0.0f;
                    
                    TIM1->CCR4 = (uint32_t)active_pwm;
                    cc_pi.integral_sum = active_pwm;
                    cv_pi.integral_sum = active_pwm;
                }
                /* STANDARD PI REGULATION */
                else 
                {
                    float cv_pwm = Calculate_PI(&cv_pi, ACTIVE_TARGET_VOLTAGE, g_actual_v);
                    float cc_pwm = Calculate_PI(&cc_pi, ACTIVE_TARGET_CURRENT, g_actual_i);
                   
                    active_pwm = (cv_pwm > cc_pwm) ? cv_pwm : cc_pwm;
                    
                    if (active_pwm > 1199.0f) active_pwm = 1199.0f;
                    if (active_pwm < 0.0f) active_pwm = 0.0f;
                    
                    TIM1->CCR4 = (uint32_t)active_pwm;
                    
                    float aw_val = active_pwm;
                    if (aw_val > 1199.0f) aw_val = 1199.0f; 
                    
                    if (cv_pwm > cc_pwm) {
                        cc_pi.integral_sum = aw_val;
                    } else {
                        cv_pi.integral_sum = aw_val;
                    }
                }
            }
            else
            {
                /* SYSTEM OFF */
                active_pwm = 1199.0f;
                TIM1->CCR4 = 1199;
                cv_pi.integral_sum = 1150.0f; 
                cc_pi.integral_sum = 1150.0f;
            }
        }
        /* 3. 100ms State Machine Update Loop */
        if((HAL_GetTick() - lastUpdate) >= 100)
        {
            lastUpdate = HAL_GetTick();

            /* UI_TASK IS NOW INSIDE THE 100ms LOOP! It will no longer choke the 1ms PI Loop. */
            UI_Task();

        

           switch(gState)
            {
             case STATE_MAINS_FAULT:
                    Output_Disable();
                    Fan_Off();
                    ACTIVE_TARGET_CURRENT = 0.0f;
                    
                    /* Auto-Recovery: Wait for AC to stabilize (PA3 LOW) for 10 seconds */
                    static uint16_t ac_recovery_counter = 0;
                    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
                        ac_recovery_counter++;
                        if (ac_recovery_counter >= 100) { // 100 ticks * 100ms = 10 seconds
                            ac_recovery_counter = 0;
                            gState = STATE_INIT; // Reboot the system
                        }
                    } else {
                        ac_recovery_counter = 0;
                    }
                    break;

                case STATE_INIT:
                    gState = STATE_WAIT_BATTERY;
                    waitTimer = HAL_GetTick();
                    ACTIVE_TARGET_CURRENT = 0.5f; // Safe probing current prevents the 8.5A windup surge 
                    break;
case STATE_WAIT_BATTERY:
                {
                    uint32_t elapsed_wake_time = HAL_GetTick() - waitTimer;
                    bool battery_detected = false;

                    /* FAN RULE: On for 10 seconds & stop */
                    if (elapsed_wake_time < 10000) Fan_On(); else Fan_Off();
                    
                    /* YIELD DISPLAY TO UI */
                    if (current_ui_state == UI_STATE_NORMAL) {
                        if (elapsed_wake_time < 2000) TM1637_DisplayNumber((uint16_t)(TARGET_VOLTAGE * 10));
                        else if (elapsed_wake_time < 4000) TM1637_DisplayNumber((uint16_t)(TARGET_CURRENT * 10));
                        else TM1637_DisplayNumber((uint16_t)(TARGET_VOLTAGE * 10)); /* Forced static Vset */
                    }
                    
                    /* ---------------------------------------------------------
                     * TWO-STAGE BATTERY DETECTION
                     * --------------------------------------------------------- */
                    if (elapsed_wake_time < 2000) {
                        /* STAGE 1 (0-2s): Passive Detection (Output OFF) */
                        Output_Disable();
                        ACTIVE_TARGET_VOLTAGE = 0.0f;
                        ACTIVE_TARGET_CURRENT = 0.0f;
                        
                        if (g_actual_v > 15.0f) {
                            battery_detected = true; /* Standard battery found */
                        }
                    } 
                    else {
                        /* STAGE 2 (2-60s): VSET Wake Mode (Output ON) */
                        Output_Enable(); 
                        ACTIVE_TARGET_VOLTAGE = TARGET_VOLTAGE;
                        ACTIVE_TARGET_CURRENT = TARGET_CURRENT; 

                        /* ALLOW CAPACITORS TO CHARGE BEFORE CHECKING CURRENT */
                        /* Give the hardware 3 seconds to stabilize at the target voltage */
                        if (elapsed_wake_time > 5000) { 
                            if (g_actual_i > 0.2f) { // 0.2A threshold
                                battery_detected = true; /* Sleeping BMS woke up and is taking charge */
                            }
                        }
                    }
                    
                    /* State Transitions */
                    if (battery_detected) {
                        float current_v_pct = g_actual_v / TARGET_VOLTAGE;
                        if (current_v_pct >= 0.65f) gChargeMode = MODE_NORMAL;
                        else if (current_v_pct >= 0.40f) gChargeMode = MODE_DPDC;
                        else gChargeMode = MODE_BTRM;
                        
                        /* FIX: Defer SOC calculation until voltage settles */
                        currentSOC = 255;

                     
                        
                        cv_pi.integral_sum = 1150.0f; 
                        cc_pi.integral_sum = 1150.0f;
                        ACTIVE_TARGET_CURRENT = 1.0f; /* Jumpstart for dead-zone search */
                        ACTIVE_TARGET_VOLTAGE = g_actual_v; 
                        
                        gState = STATE_CHARGING;
                        totalChargeTimer = HAL_GetTick(); 
                    }
                    else if (elapsed_wake_time >= 60000) {
                        /* No battery detected after 60 seconds. Exit to IDLE. */
                        gState = STATE_IDLE; 
                    }
                    
                    previousVoltage = g_actual_v;
                    break;
                }

               case STATE_CHARGING:
                    /* FIX: Wait 2 seconds for capacitors and filters to drop to true battery voltage */
                    if (currentSOC == 255) {
                        if ((HAL_GetTick() - totalChargeTimer) >= 2000) {
                            SOC_Calculate_Initial();
                        }
                    } else {
                        SOC_Evaluation_Task(); 
                    }
                    
                    if ((HAL_GetTick() - totalChargeTimer) >= 25200000) {
                        gState = STATE_CHTO; /* Route to CHTO */
                        Output_Disable();
                        break;
                    }

                    if (g_actual_v >= (TARGET_VOLTAGE * SOC_VSET_PCT[19])) { 
                        if (!cvSafetyTimerActive) {
                            cvSafetyTimer = HAL_GetTick();
                            cvSafetyTimerActive = true;
                        } else if ((HAL_GetTick() - cvSafetyTimer) >= 3600000) {
                            gState = STATE_CHTO; /* Route to CHTO */
                            Output_Disable();
                            break;
                        }
                    } else {
                        cvSafetyTimerActive = false; 
                    }
                    
                    Output_Enable();
                    Fan_On();

                    if (Thermal_IsFault()) {
                        gState = STATE_THERMAL_FAULT;
                        Output_Disable();
                        break;
                    }

                    if (ACTIVE_TARGET_VOLTAGE < TARGET_VOLTAGE) {
                        ACTIVE_TARGET_VOLTAGE += 0.5f;
                        if (ACTIVE_TARGET_VOLTAGE > TARGET_VOLTAGE) ACTIVE_TARGET_VOLTAGE = TARGET_VOLTAGE;
                    } else if (ACTIVE_TARGET_VOLTAGE > TARGET_VOLTAGE) {
                        ACTIVE_TARGET_VOLTAGE -= 0.5f;
                        if (ACTIVE_TARGET_VOLTAGE < TARGET_VOLTAGE) ACTIVE_TARGET_VOLTAGE = TARGET_VOLTAGE;
                    }

                    float v_65 = TARGET_VOLTAGE * 0.65f;
                    float v_40 = TARGET_VOLTAGE * 0.40f;
                    float v_20 = TARGET_VOLTAGE * 0.20f;
                    float safe_current_limit = 0.0f;
                    
                    if (g_actual_v < v_20) {
                        gState = STATE_BTNG; /* Route to BTNG */
                        Output_Disable();
                        break;
                    } else if (g_actual_v < v_40) {
                        safe_current_limit = 0.5f; 
                    } else if (g_actual_v < v_65) {
                        safe_current_limit = 1.0f; 
                    } else {
                        safe_current_limit = TARGET_CURRENT; 
                    }
                    
                    safe_current_limit *= Thermal_GetCurrentScale();

                    if (ACTIVE_TARGET_CURRENT < safe_current_limit) {
                        ACTIVE_TARGET_CURRENT += 0.03f;
                        if (ACTIVE_TARGET_CURRENT > safe_current_limit) ACTIVE_TARGET_CURRENT = safe_current_limit;
                    } else if (ACTIVE_TARGET_CURRENT > safe_current_limit) {
                        ACTIVE_TARGET_CURRENT = safe_current_limit; 
                    }
                    
                    /* REQUIRED CV TERMINATION LOGIC */
                    static uint32_t cv_termination_timer = 0;
                    float termination_current = TARGET_CURRENT * 0.4f; /* 0.4 * CSET */
                    
                    if (g_actual_v >= (TARGET_VOLTAGE - 0.2f) && g_actual_i <= termination_current) {
                        if (cv_termination_timer == 0) cv_termination_timer = HAL_GetTick();
                        else if ((HAL_GetTick() - cv_termination_timer) >= 10000) { 
                            gState = STATE_CHARGE_COMPLETE;
                            cv_termination_timer = 0; 
                        }
                    } else {
                        cv_termination_timer = 0; 
                    }
                    break;

                case STATE_CHARGE_COMPLETE:
                    /* HARD LOCK: AC restart required to exit this state */
                    Output_Disable();
                    Fan_Off();
                    ACTIVE_TARGET_CURRENT = 0.0f;
                    break;

                case STATE_FAULT_LOCK:
                    Output_Disable();
                    Fan_Off();
                    ACTIVE_TARGET_CURRENT = 0.0f;
                    
                    break;

               case STATE_IDLE:
                    Output_Disable();
                    Fan_Off();
                    ACTIVE_TARGET_CURRENT = 0.0f;
                    ACTIVE_TARGET_VOLTAGE = 0.0f; 

                    float raw_dV = g_actual_v - previousVoltage;
                    
                    /* If voltage jumps by more than 2V (e.g. from 0V to 60V on plug-in) */
                    if ((raw_dV > 2.0f) && g_actual_v > 15.0f)
                    {
                        gState = STATE_WAIT_BATTERY;
                        waitTimer = HAL_GetTick();
                    }
                    
                    previousVoltage = g_actual_v;
                    break;
                    
                case STATE_THERMAL_FAULT:
                    Output_Disable();
                    Fan_Off();
                    if (!Thermal_IsFault())
                    {
                        gState = STATE_INIT;
                    }
                    break;
            }

Thermal_Task();

            if (gState == STATE_THERMAL_FAULT)
            {
                TM1637_DisplayFault();
            }
            else if (current_ui_state == UI_STATE_NORMAL && gState != STATE_WAIT_BATTERY)
            {
                uint8_t toggle_10s = (HAL_GetTick() / 10000) % 2;

                if (toggle_10s == 0 || gState == STATE_INIT) {
                    /* Display static Vset target */
                    TM1637_DisplayNumber((uint16_t)(TARGET_VOLTAGE * 10)); 
                } 
                else {
                    /* Display Context/Fault on Odd 10s cycles */
                    switch(gState) {
                        case STATE_CHARGING:
                            if (gChargeMode == MODE_DPDC) TM1637_DisplayRaw((uint8_t*)DISP_DPDC);
                            else if (gChargeMode == MODE_BTRM) TM1637_DisplayRaw((uint8_t*)DISP_BTRM);
                            else {
                                uint8_t soc_data[4] = {0x6D, 0x00, TM1637_DigitMap[(currentSOC / 10) % 10], TM1637_DigitMap[currentSOC % 10]};
                                TM1637_DisplayRaw(soc_data);
                            }
                            break;
                        case STATE_CHARGE_COMPLETE: TM1637_DisplayRaw((uint8_t*)DISP_100P); break;
                        case STATE_IDLE: TM1637_DisplayRaw((uint8_t*)DISP_BTNG); break; 
                        case STATE_BTNG: TM1637_DisplayRaw((uint8_t*)DISP_BTNG); break; 
                        case STATE_CHTO: TM1637_DisplayRaw((uint8_t*)DISP_CHTO); break;
                        case STATE_SCPT: TM1637_DisplayRaw((uint8_t*)DISP_SCPT); break;
                        case STATE_THERMAL_FAULT: TM1637_DisplayRaw((uint8_t*)DISP_HITP); break;
                        case STATE_MAINS_FAULT: TM1637_DisplayRaw((uint8_t*)DISP_POFF); break;
                        default: TM1637_DisplayFault(); break;
                    }
                }
            }
        }
    }
}

TIM_HandleTypeDef htim1;

           



void PWM_Hardware_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 1. Configure Timer Timebase (24MHz / 1 / 1200 = 20kHz) */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 1199;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    /* 2. Configure Channel 4 for PWM Mode 1 */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1199; /* Initialize to max value (minimum power out) */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

    /* 3. Configure Break and Dead-Time (Enables MOE Bit) */
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

    /* 4. Start the PWM output internally FIRST */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* 5. FINALLY Map the Pin to the Active Timer (Handoff without glitching) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}



 void SystemClock_Config(void)

{

    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};



    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;

    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;

    RCC_OscInitStruct.LSIState = RCC_LSI_OFF;

    //RCC_OscInitStruct.LSEState = RCC_LSE_OFF;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)

    {

        while(1);

    }



    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;



    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)

    {

        while(1);

    }

}

// ============================================================================
// SYSTEM TIMER INTERRUPT HANDLER
// ============================================================================

extern volatile uint8_t gOutputActive;

void SysTick_Handler(void)
{
    HAL_IncTick();

    /* 1. AC Mains UV/OV Protection Override (Ultra-Fast) */
  //  static uint8_t mains_fault_counter = 0;
    
    //if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) 
   // { 
     //   mains_fault_counter++;
       // if (mains_fault_counter >= 50) { // 50ms debounce
            
          /* Instantly sever output power via direct register write */
         //   TIM1->CCR4 = 1199;
           // gOutputActive = 0;
            
            /* Hijack state machine */
            //if (gState != STATE_MAINS_FAULT) {
              //  gState = STATE_MAINS_FAULT;
            //}
        //}
    //} 
    //else 
    //{
      //  mains_fault_counter = 0; 
    //}
}