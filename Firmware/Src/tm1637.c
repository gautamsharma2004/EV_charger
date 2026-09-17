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



#include "tm1637.h"

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

