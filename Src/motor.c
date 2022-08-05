/**
 * 电机控制，电机编码器每圈17个脉冲，X4以后68个脉冲
 */

/* Includes -----------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include "stm32f1xx_hal.h"
#include "pid.h"
#include "door_control.h"
#include "motor.h"
#include "detector.h"
#include "can.h"
#include "OverCurrentProtection.h"

/* PID 参数------------------------------------------------------------------*/
#define P 0.2
#define I 0
#define D 0.05

/* --------------- Private functions ---------------------------------------*/
static void SetPWM(uint16_t motorNo, float pwm_val);

/* External variables---------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* Private variables --------------------------------------------------------*/
// 测出来的角速度
static float angularSpeed[DOOR_QUANTITY];
// 编码器读数
static int32_t encoderCount[DOOR_QUANTITY];
static int32_t lastCount[DOOR_QUANTITY]; // 编码器上次读取值
static float outputPwm[DOOR_QUANTITY];          // PWM输出值
// 上次角度
static float prevAngle[DOOR_QUANTITY];
// 当前角度
float currentAngle[DOOR_QUANTITY];

// PID 参数
static PID motorPID[DOOR_QUANTITY];

/**
 * @brief 电机PID控制
 * 从编码器差值（由本次编码器读数和上次读数差值得到）计算当前电机角速度，调用PID函数进行计算。
 * 得到计算结果，对PWM输出值进行增量式修正。将结果通过调用SetPWM()函数传递给定时器。
 * @param None
 * @retval None
 */
void PID_Control(uint16_t motorNo)
{
    float pidResult = PIDCalc(&motorPID[motorNo], angularSpeed[motorNo]);
    outputPwm[motorNo] += pidResult;
    if (outputPwm[motorNo] > 1)
        outputPwm[motorNo] = 1;
    else if (outputPwm[motorNo] < -1)
        outputPwm[motorNo] = -1;
    SetPWM(motorNo, outputPwm[motorNo]);
    UpdateOverCurrentProtection(motorNo, outputPwm[motorNo]);
}

/**
 * @brief 设置目标角速度
 * @param motorNo 电机选择
 * @param angularSpeed 目标角速度
 * @retval None
 */
void SetAngularSpeed(uint16_t motorNo, float angularSpeed)
{
    SetPIDObject(&motorPID[motorNo], angularSpeed);
}

/**
 * @brief	: 设置轮子PWM值
 * @param 	motorNo	选择电机
 * @param	pwm_val	小于等于1的PWM占空比
 * @param   direction 方向，FORWARD前进，BACKWARD后退
 * @retval	None
 */
static void SetPWM(uint16_t motorNo, float pwm_val)
{
    if (pwm_val > 1.0f)
        pwm_val = 1.0f;
    else if (pwm_val < -1.0f)
        pwm_val = -1.0f;

    uint16_t pwm;
    if (motorNo == LEFT)
    {
        if (pwm_val >= 0)
        {
            pwm = pwm_val * MAX_PWM_VAL;
            TIM1->CCR1 = 0;
            TIM1->CCR2 = pwm;
        }
        else
        {
            pwm = -pwm_val * MAX_PWM_VAL;
            TIM1->CCR1 = pwm;
            TIM1->CCR2 = 0;
        }
    }
    else if (motorNo == RIGHT)
    {
        if (pwm_val >= 0)
        {
            pwm = pwm_val * MAX_PWM_VAL;
            TIM1->CCR3 = 0;
            TIM1->CCR4 = pwm;
        }
        else
        {
            pwm = -pwm_val * MAX_PWM_VAL;
            TIM1->CCR3 = pwm;
            TIM1->CCR4 = 0;
        }
    }
}

/**
 * @brief: 设置比较值为0，停止电机转动, 同时把PID的目标值设置为0
 * @param: 门的编号
 * @retval: None
 */
void SetAngularSpeedToZero(uint16_t motorNo)
{
    SetAngularSpeed(motorNo, 0);
    if (motorNo == LEFT)
    {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
    }
    else if (motorNo == RIGHT)
    {
        TIM1->CCR3 = 0;
        TIM1->CCR4 = 0;
    }
		
    outputPwm[motorNo] = 0;
}

/**
 * @brief 控制电机的外设初始化，PID参数初始化
 * @return none
 */
void MotorInit(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    for (int i = 0; i < DOOR_QUANTITY; ++i)
        PIDInit(&motorPID[i], P, I, D);
}

/**
 * @brief Read encoder count
 * Read the encoder and subtract the last saved value from
 * the current value to get the difference. The difference means
 * speed of the motor.
 * @return none
 */
void UpdateEncoderCount(void)
{
    int32_t leftCount = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t rightCount = __HAL_TIM_GET_COUNTER(&htim3);

    encoderCount[0] -= lastCount[0];
    encoderCount[0] += leftCount;
    encoderCount[1] -= lastCount[1];
    encoderCount[1] += rightCount;
    lastCount[0] = leftCount;
    lastCount[1] = rightCount;

    currentAngle[0] = (2 * PI / (EDGE_PER_CIRCLE)) * (float)encoderCount[0];
    currentAngle[1] = (2 * PI / (EDGE_PER_CIRCLE)) * (float)encoderCount[1];
    angularSpeed[0] = (currentAngle[0] - prevAngle[0]) / TIME_PERIOD;
    angularSpeed[1] = (currentAngle[1] - prevAngle[1]) / TIME_PERIOD;
    prevAngle[0] = currentAngle[0];
    prevAngle[1] = currentAngle[1];
}

float GetMotorPosition(uint16_t motorNo)
{
    return currentAngle[motorNo];
}

void SetMotorPosition(uint16_t motorNo, float pos)
{
    encoderCount[motorNo] = pos;
}

void EncoderCounterOverflow(uint16_t encoderNo)
{
    int32_t count;
    if (encoderNo == 0)
        count = __HAL_TIM_GET_COUNTER(&htim2);
    else if (encoderNo == 1)
        count = __HAL_TIM_GET_COUNTER(&htim3);
    if (count > 0x7FFF)
    { // 大于一半值说明从0x0到0xFFFF翻转，否则说明从0xFFFF到0x0
        encoderCount[encoderNo] -= 65536;
    }
    else
    {
        encoderCount[encoderNo] += 65536;
    }
}

/**
 * @brief Clear all encoder count and angular data.
 * @param encodeNo Encoder Number
 * @retval none
 */
void ClearEncoderAndAngle(uint16_t encoderNo)
{
    encoderCount[encoderNo] = 0;
    currentAngle[encoderNo] = 0;
    prevAngle[encoderNo] = 0;
    angularSpeed[encoderNo] = 0;
}
