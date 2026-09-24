/* ============================================================================
 * MERGED APPLICATION SOURCE CODE 
 * ============================================================================ */
#include "main.h"
#include "soc_display_exact_table.c"
#include <math.h>

float TARGET_VOLTAGE = 67.20f;
float TARGET_CURRENT = 6.2f;  

const float SOC_VSET_PCT[21] = {
    0.650f, 0.803f, 0.826f, 0.849f, 0.872f, 0.895f, 0.900f, 0.905f, 0.910f, 0.915f, 
    0.920f, 0.925f, 0.930f, 0.935f, 0.940f, 0.945f, 0.956f, 0.967f, 0.978f, 0.989f, 1.000f  
};

uint32_t totalChargeTimer = 0;
uint32_t cvSafetyTimer = 0;
bool cvSafetyTimerActive = false;

volatile float ACTIVE_TARGET_CURRENT = 0.0f; 
volatile float ACTIVE_TARGET_VOLTAGE = 0.0f;
volatile float g_actual_v = 0.0f;
volatile float g_actual_i = 0.0f;

volatile uint8_t currentSOC = 0; /* Retained solely as a Phase 1 Timer Flag */

#define NTC_R25                 10000.0f
#define NTC_PULLDOWN_RESISTOR   10000.0f
#define NTC_BETA                3950.0f 
#define NTC_T25_KELVIN          298.15f 
#define CURRENT_CAL_FACTOR 1.000f  
#define VOLTAGE_CAL_FACTOR 0.882f

typedef struct {
    float Kp;
    float Ki;
    float integral_sum;
    float out_max;
    float out_min;
} PI_Controller;

PI_Controller cv_pi = { .Kp = 20.0f, .Ki = 0.5f, .integral_sum = 1199.0f, .out_max = 1199.0f, .out_min = 0.0f };
PI_Controller cc_pi = { .Kp = 2.0f, .Ki = 0.05f, .integral_sum = 1199.0f, .out_max = 1199.0f, .out_min = 0.0f };

float Calculate_PI(PI_Controller *pi, float setpoint, float actual)
{
    float error = actual - setpoint;
    
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

static ADC_HandleTypeDef hadc;

void ADC_Driver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

    for(volatile int i=0; i<100; i++) { __NOP(); }

    HAL_ADC_Start(&hadc);

    if (HAL_ADC_PollForConversion(&hadc, 100) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc);
        return 0;
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

void Battery_Detect_Init(void) { }

bool Battery_IsDetected(void)
{
    float raw_voltage = ADC_ReadVoltage(ADC_CH_CV_SENSE);
    float output_voltage = raw_voltage * 17.34f;
    
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

void Fan_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = FAN_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    HAL_GPIO_Init(FAN_GPIO_PORT, &GPIO_InitStruct);
    Fan_Off();
}

void Fan_On(void) { HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_GPIO_PIN, GPIO_PIN_SET); }
void Fan_Off(void) { HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_GPIO_PIN, GPIO_PIN_RESET); }
void Fan_Toggle(void) { HAL_GPIO_TogglePin(FAN_GPIO_PORT, FAN_GPIO_PIN); }

void NTC_Init(void) { }

uint16_t NTC_ReadMosfetADC(void) { return ADC_ReadChannel(NTC_MOSFET_CHANNEL); }
uint16_t NTC_ReadTransformerADC(void) { return ADC_ReadChannel(NTC_TRANSFORMER_CHANNEL); }
float NTC_ReadMosfetVoltage(void) { return ADC_ReadVoltage(NTC_MOSFET_CHANNEL); }
float NTC_ReadTransformerVoltage(void) { return ADC_ReadVoltage(NTC_TRANSFORMER_CHANNEL); }

float NTC_ADCToResistance(uint16_t adc)
{
    if(adc == 0) return 1000000.0f;
    if(adc >= 4095) return 1.0f;
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
uint8_t vset_index = 4;
uint8_t cset_index = 0;

uint32_t button_press_start = 0;
uint32_t last_activity_time = 0;
uint32_t save_display_timer = 0;
bool button_held = false;

const uint8_t DISP_HITP[4] = {0x76, 0x06, 0x78, 0x73}; 
const uint8_t DISP_CHTO[4] = {0x39, 0x76, 0x78, 0x3F}; 
const uint8_t DISP_BTNG[4] = {0x7C, 0x78, 0x54, 0x3D}; 
const uint8_t DISP_DPDC[4] = {0x5E, 0x73, 0x5E, 0x39}; 
const uint8_t DISP_BTRM[4] = {0x7C, 0x78, 0x50, 0x54}; 
const uint8_t DISP_100P[4] = {0x06, 0x3F, 0x3F, 0x73}; 
const uint8_t DISP_SCPT[4] = {0x6D, 0x39, 0x73, 0x78}; 
const uint8_t DISP_POFF[4] = {0x73, 0x40, 0x3F, 0x71}; 

typedef enum {
    STATE_INIT = 0,
    STATE_WAIT_BATTERY,
    STATE_CHARGING,
    STATE_CHARGE_COMPLETE,
    STATE_THERMAL_FAULT,
    STATE_IDLE,
    STATE_FAULT_LOCK,
    STATE_MAINS_FAULT,
    STATE_BTNG,        
    STATE_CHTO,        
    STATE_SCPT         
} ChargerState_t;

volatile ChargerState_t gState = STATE_INIT;

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
            if (maxT >= THERM_SHUTDOWN_TEMP) { thermData.state = THERM_STATE_FAULT; }
            else if (maxT >= THERM_DERATE_START_TEMP) { thermData.state = THERM_STATE_DERATING; }
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

static const uint8_t TM1637_DigitMap[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71
};

static uint8_t TM1637_Brightness = 0x07;

static void TM1637_Delay(void) { for(volatile int i=0; i<100; i++){ __NOP(); } }

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
        if(data & 0x01) HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_SET);
        else HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_RESET);
        TM1637_Delay();
        HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);
        TM1637_Delay();
        data >>= 1;
    }
    HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);
    
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
    for(int i=0; i<4; i++) TM1637_WriteByte(data[i]);
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

void TM1637_DisplayFault(void)
{
    uint8_t data[4] = {0x71, 0x77, 0x3E, 0x38}; // FAUL
    TM1637_DisplayRaw(data);
}

typedef enum {
    MODE_NORMAL = 0,
    MODE_DPDC,
    MODE_BTRM
} ChargeMode_t;

volatile ChargeMode_t gChargeMode = MODE_NORMAL;
volatile uint8_t gOutputActive = 0;

void SystemClock_Config(void);
void PWM_Hardware_Init(void);

static void Output_Enable(void) { gOutputActive = 1; }
static void Output_Disable(void) { gOutputActive = 0; }

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
    
    for(int i = 3; i < 32; i++) { page_buffer[i] = 0xFFFFFFFF; }

    HAL_FLASH_Unlock();
    
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGEERASE; 
    EraseInitStruct.PageAddress = FLASH_EEPROM_PAGE_ADDR;
    EraseInitStruct.NbPages = 1;
    
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, FLASH_EEPROM_PAGE_ADDR, page_buffer);
    }
    
    HAL_FLASH_Lock();
}

void EEPROM_Load(void)
{
    uint32_t magic = *(__IO uint32_t*)FLASH_EEPROM_PAGE_ADDR;

    if (magic == EEPROM_MAGIC_NUMBER) {
        vset_index = (uint8_t)(*(__IO uint32_t*)(FLASH_EEPROM_PAGE_ADDR + 4));
        cset_index = (uint8_t)(*(__IO uint32_t*)(FLASH_EEPROM_PAGE_ADDR + 8));
        
        if (vset_index >= VSET_COUNT) vset_index = 4;
        if (cset_index >= CSET_COUNT) cset_index = 0;
    } else {
        vset_index = 4; 
        cset_index = 0; 
        EEPROM_Save(vset_index, cset_index);
    }
    
    TARGET_VOLTAGE = VSET_ARRAY[vset_index];
    TARGET_CURRENT = CSET_ARRAY[cset_index];
}

void UI_Task(void)
{
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
            if (press_duration < 2000 && press_duration > 50) { 
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
                TARGET_VOLTAGE = VSET_ARRAY[vset_index];
                TARGET_CURRENT = CSET_ARRAY[cset_index];
                EEPROM_Save(vset_index, cset_index);
                current_ui_state = UI_STATE_NORMAL;
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
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

int main(void)
{
    uint32_t lastUpdate = 0;
    uint32_t waitTimer =  0;
    float previousVoltage = 0.0f;

    /* ------------------------------------------------------------------------
     * IMMEDIATE HARDWARE CLAMP
     * ------------------------------------------------------------------------ */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    /* HARDWARE POWER-RAMP DELAY */
    for(volatile uint32_t wait = 0; wait < 1500000; wait++) { __NOP(); }

    HAL_Init();
    SystemClock_Config();
    SystemCoreClockUpdate();
    
    PWM_Hardware_Init();
    GPIO_Init();
    ADC_Driver_Init();
    NTC_Init();
    Thermal_Init();
    TM1637_Init();
    Fan_Init();
    Battery_Detect_Init();
    EEPROM_Load(); 
    TM1637_Clear();
    
    gState = STATE_INIT;
    
    uint32_t last_pi_time = 0;
    bool filter_initialized = false;

    while(1)
    {
        /* 1. High-Speed ADC Polling Task */
        float raw_i = (ADC_ReadVoltage(ADC_CH_C_SENSE) / 0.23f) * CURRENT_CAL_FACTOR; 
        raw_i -= 1.0f; /* Cancel op-amp zero-load bias */
        if (raw_i < 0.0f) raw_i = 0.0f;
        
        float raw_v = (ADC_ReadVoltage(ADC_CH_CV_SENSE) * 17.34f) * VOLTAGE_CAL_FACTOR;
        
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
            static float active_pwm = 1199.0f; 

            if (gOutputActive && gState != STATE_MAINS_FAULT)
            {
                /* SCPT: 150% CSET Threshold (With 50ms Inrush Debounce) */
                static uint8_t scpt_debounce = 0;
                
                if (g_actual_i > (TARGET_CURRENT * 1.5f)) { 
                    scpt_debounce++;
                    if (scpt_debounce >= 50) { 
                        active_pwm = 1199.0f;
                        TIM1->CCR4 = 1199; 
                        cv_pi.integral_sum = 1199.0f; 
                        cc_pi.integral_sum = 1199.0f;
                        ACTIVE_TARGET_CURRENT = 0.0f; 
                        gState = STATE_SCPT; 
                        scpt_debounce = 0;
                    }
                } 
                else 
                {
                    scpt_debounce = 0;
                    
                    /* PURE PI REGULATION ONLY */
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

            static float disp_v_filtered = 0.0f;
            if (disp_v_filtered == 0.0f) disp_v_filtered = g_actual_v;
            disp_v_filtered = (disp_v_filtered * 0.8f) + (g_actual_v * 0.2f);

            UI_Task();

            switch(gState)
            {
                case STATE_MAINS_FAULT:
                    Output_Disable();
                    Fan_Off();
                    ACTIVE_TARGET_CURRENT = 0.0f;
                    
                    static uint16_t ac_recovery_counter = 0;
                    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
                        ac_recovery_counter++;
                        if (ac_recovery_counter >= 100) { 
                            ac_recovery_counter = 0;
                            gState = STATE_INIT; 
                        }
                    } else {
                        ac_recovery_counter = 0;
                    }
                    break;

                case STATE_INIT:
                    gState = STATE_WAIT_BATTERY;
                    waitTimer = HAL_GetTick();
                    ACTIVE_TARGET_CURRENT = 0.5f; 
                    break;

                case STATE_WAIT_BATTERY:
                {
                    uint32_t elapsed_wake_time = HAL_GetTick() - waitTimer;
                    bool battery_detected = false;

                    if (elapsed_wake_time < 10000) Fan_On(); else Fan_Off();
                    
                    if (elapsed_wake_time < 2000) {
                        /* STAGE 1 (0-2s): Passive Detection */
                        Output_Disable();
                        ACTIVE_TARGET_VOLTAGE = 0.0f;
                        ACTIVE_TARGET_CURRENT = 0.0f;
                        
                        if (g_actual_v > 15.0f) {
                            battery_detected = true; 
                        }
                    } 
                    else {
                        /* STAGE 2 (2-60s): Smooth Voltage Ramp */
                        Output_Enable(); 
                        
                        if (ACTIVE_TARGET_VOLTAGE < TARGET_VOLTAGE) {
                            ACTIVE_TARGET_VOLTAGE += 0.5f; 
                            if (ACTIVE_TARGET_VOLTAGE > TARGET_VOLTAGE) ACTIVE_TARGET_VOLTAGE = TARGET_VOLTAGE;
                        }
                        ACTIVE_TARGET_CURRENT = 0.5f; 

                        if (elapsed_wake_time > 5000) { 
                            if (g_actual_i > 0.2f) { 
                                battery_detected = true; 
                            }
                        }
                    }
                    
                    if (battery_detected) {
                        float current_v_pct = g_actual_v / TARGET_VOLTAGE;
                        if (current_v_pct >= 0.65f) gChargeMode = MODE_NORMAL;
                        else if (current_v_pct >= 0.40f) gChargeMode = MODE_DPDC;
                        else gChargeMode = MODE_BTRM;
                        
                        SOC_DisplayInit(g_actual_v, TARGET_VOLTAGE);
                        currentSOC = 255; /* Phase 1 timer flag */
                        
                        cv_pi.integral_sum = 1150.0f; 
                        cc_pi.integral_sum = 1150.0f;
                        ACTIVE_TARGET_CURRENT = 0.5f; 
                        ACTIVE_TARGET_VOLTAGE = g_actual_v; 
                        
                        gState = STATE_CHARGING;
                        totalChargeTimer = HAL_GetTick(); 
                    }
                    else if (elapsed_wake_time >= 60000) {
                        gState = STATE_IDLE; 
                    }
                    
                    previousVoltage = g_actual_v;
                    break;
                }

                case STATE_CHARGING:
                    if ((HAL_GetTick() - totalChargeTimer) >= 25200000) {
                        gState = STATE_CHTO; 
                        Output_Disable();
                        break;
                    }

                    if (Thermal_IsFault()) {
                        gState = STATE_THERMAL_FAULT;
                        Output_Disable();
                        break;
                    }

                    Output_Enable();
                    Fan_On();

                    /* PHASE 1: HARDWARE STABILIZATION */
                    if (currentSOC == 255) {
                        ACTIVE_TARGET_CURRENT = 1.5f; 
                        ACTIVE_TARGET_VOLTAGE = TARGET_VOLTAGE; 
                        
                        if ((HAL_GetTick() - totalChargeTimer) >= 3000) {
                            currentSOC = 0; /* Clear flag to enter Phase 2 */
                        }
                    } 
                    /* PHASE 2: ACTIVE CHARGING REGULATION */
                    else {
                        SOC_DisplayUpdateTask(g_actual_v, TARGET_VOLTAGE, 60000); 

                        if (g_actual_v >= (TARGET_VOLTAGE * SOC_VSET_PCT[19])) { 
                            if (!cvSafetyTimerActive) {
                                cvSafetyTimer = HAL_GetTick();
                                cvSafetyTimerActive = true;
                            } else if ((HAL_GetTick() - cvSafetyTimer) >= 3600000) {
                                gState = STATE_CHTO; 
                                Output_Disable();
                                break;
                            }
                        } else {
                            cvSafetyTimerActive = false; 
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
                            gState = STATE_BTNG; 
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
                    }
                    
                    static uint32_t cv_termination_timer = 0;
                    float termination_current = TARGET_CURRENT * 0.4f; 
                    
                    if (g_actual_v >= (TARGET_VOLTAGE - 0.2f) && g_actual_i <= termination_current) {
                        if (cv_termination_timer == 0) cv_termination_timer = HAL_GetTick();
                        else if ((HAL_GetTick() - cv_termination_timer) >= 10000) { 
                            SOC_DisplaySetComplete();
                            gState = STATE_CHARGE_COMPLETE;
                            cv_termination_timer = 0; 
                        }
                    } else {
                        cv_termination_timer = 0; 
                    }
                    break;

                case STATE_CHARGE_COMPLETE:
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
                    if ((raw_dV > 2.0f) && g_actual_v > 15.0f) {
                        gState = STATE_WAIT_BATTERY;
                        waitTimer = HAL_GetTick();
                    }
                    previousVoltage = g_actual_v;
                    break;
                    
                case STATE_THERMAL_FAULT:
                    Output_Disable();
                    Fan_Off();
                    if (!Thermal_IsFault()) {
                        gState = STATE_INIT;
                    }
                    break;
            }

            Thermal_Task();

            /* DYNAMIC DISPLAY UPDATE */
            if (gState == STATE_THERMAL_FAULT) {
                TM1637_DisplayFault();
            }
            else if (current_ui_state == UI_STATE_NORMAL && gState != STATE_WAIT_BATTERY) {
                uint32_t cycle_time = HAL_GetTick() % 10000;

                if (gState == STATE_INIT) {
                    TM1637_DisplayNumber((uint16_t)(disp_v_filtered * 10)); 
                } 
                else if (gState == STATE_CHARGING) {
                    if (cycle_time < 4000) {
                        TM1637_DisplayNumber((uint16_t)(disp_v_filtered * 10));
                    } 
                    else if (cycle_time < 6000) {
                        if (gChargeMode == MODE_DPDC) TM1637_DisplayRaw((uint8_t*)DISP_DPDC);
                        else if (gChargeMode == MODE_BTRM) TM1637_DisplayRaw((uint8_t*)DISP_BTRM);
                        else if (currentSOC == 255) TM1637_DisplayNumber((uint16_t)(disp_v_filtered * 10));
                        else {
                            uint8_t display_soc = SOC_DisplayGetPercentage();
                            uint8_t soc_data[4] = {0x6D, 0x39, TM1637_DigitMap[(display_soc / 10) % 10], TM1637_DigitMap[display_soc % 10]};
                            TM1637_DisplayRaw(soc_data);
                        }
                    } 
                    else if (cycle_time < 8000) {
                        TM1637_DisplayNumber((uint16_t)(TARGET_VOLTAGE * 10));
                    } 
                    else {
                        TM1637_DisplayNumber((uint16_t)(TARGET_CURRENT * 10));
                    }
                }
                else {
                    switch(gState) {
                        case STATE_CHARGE_COMPLETE: TM1637_DisplayRaw((uint8_t*)DISP_100P); break;
                        case STATE_IDLE: TM1637_DisplayRaw((uint8_t*)DISP_BTNG); break; 
                        case STATE_BTNG: TM1637_DisplayRaw((uint8_t*)DISP_BTNG); break; 
                        case STATE_CHTO: TM1637_DisplayRaw((uint8_t*)DISP_CHTO); break;
                        case STATE_SCPT: TM1637_DisplayRaw((uint8_t*)DISP_SCPT); break;
                        case STATE_MAINS_FAULT: TM1637_DisplayRaw((uint8_t*)DISP_POFF); break;
                        default: TM1637_DisplayFault(); break;
                    }
                }
            }
            else if (current_ui_state == UI_STATE_NORMAL && gState == STATE_WAIT_BATTERY) {
                TM1637_DisplayNumber((uint16_t)(disp_v_filtered * 10));
            }
        }
    }
}

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

extern volatile uint8_t gOutputActive;

void SysTick_Handler(void)
{
    HAL_IncTick();
}
