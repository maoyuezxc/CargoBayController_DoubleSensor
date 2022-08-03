/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : io.h
 * @brief          : Header for io.c file.
 *                   This file contains the common defines of the gpio.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Pix Moving
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IO_H
#define __IO_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum IO_INPUT_PIN
{
  PinSensor0 = 0, // PB2
  PinSensor1,     // PB3
  PinSensor2,     // PB4
  PinSensor3,     // PB5

} IO_INPUT_PIN_t;

typedef enum IO_OUTPUT_PIN
{
	PinDriverEn0 = 0,
	PinDriverEn1,

} IO_OUTPUT_PIN_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void SetPinState(uint8_t pinNum, GPIO_PinState level);
void TogglePin(uint8_t pinNum);
GPIO_PinState ReadPinState(uint8_t pinNum);
/* Private defines -----------------------------------------------------------*/

#endif /* __IO_H */

/******************************END OF FILE*************************************/
