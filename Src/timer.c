/* Includes -----------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "motor.h"
#include "detector.h"
#include "door_control.h"

/* External variables---------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/**
 * @brief TIMER overflow callback funtion
 * @param htim Pointer to TIMER handle which caused the interrupt.
 * @return none
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    EncoderCounterOverflow(0);
  }
  else if (htim->Instance == TIM3)
  {
    EncoderCounterOverflow(1);
  }
  else if (htim->Instance == TIM4)
  { // 20ms�ж�һ��
    UpdateEncoderCount();
    for (int i = 0; i < DOOR_QUANTITY; ++i)
      PID_Control(i);
    Detector();
  }
}
