/**
 *****************************************************************************
 * @file           : io.c
 * @brief          : this file contains the definitions of gpio control
 *                   functions
 *****************************************************************************/

#include "io.h"
#include "main.h"

typedef struct IO_PORT
{
    GPIO_TypeDef *port;
    uint16_t pin;
} IO_PORT_t;

static const IO_PORT_t InputPins[] =
    {
        {SENSOR_0_GPIO_Port, SENSOR_0_Pin},
        {SENSOR_1_GPIO_Port, SENSOR_1_Pin},
        {SENSOR_2_GPIO_Port, SENSOR_2_Pin},
        {SENSOR_3_GPIO_Port, SENSOR_3_Pin},

};

static const IO_PORT_t OutPins[] =
    {
			{EN0_GPIO_Port,EN0_Pin},
			{EN1_GPIO_Port,EN1_Pin},
};

#define OUT_PIN_COUNT (sizeof(OutPins) / sizeof(IO_PORT_t))
#define IN_PIN_COUNT (sizeof(InputPins) / sizeof(IO_PORT_t))

/**
 * @brief Set the level of output pins
 * @param pinNum, the pin designator defined in io.h
 * @param level, the GPIO level to be set
 * @retval None
 */
void SetPinState(uint8_t pinNum, GPIO_PinState level)
{
    if (pinNum < OUT_PIN_COUNT)
        HAL_GPIO_WritePin(OutPins[pinNum].port, OutPins[pinNum].pin, level);
}

/**
 * @brief toggle the pin state
 * @param pinNum, the pib number
 * @retval None
 */
void TogglePin(uint8_t pinNum)
{
    if (pinNum < OUT_PIN_COUNT)
        HAL_GPIO_TogglePin(OutPins[pinNum].port, OutPins[pinNum].pin);
}

/**
 * @brief Read the pin state
 * @param Pin number
 * @retval None
 */
GPIO_PinState ReadPinState(uint8_t i)
{
    if (i < IN_PIN_COUNT)
    {
        return HAL_GPIO_ReadPin(InputPins[i].port, InputPins[i].pin);
    }

    else
    {
        while (1)
            ;
    }
}

/******************************END OF FILE*************************************/
