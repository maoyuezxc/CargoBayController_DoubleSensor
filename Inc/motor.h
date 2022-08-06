#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>

// PI
#define PI 3.14159265358979323846

// 轮子每转一圈的AB相边沿数量之和
// 减速比515，编码器每圈17个脉冲，X4，齿轮减速比1.5:1
#define EDGE_PER_CIRCLE (515.0f * 11.0f * 4.0f * 36.0f / 30.0f) //原来52530, 515.0f * 11.0f * 4.0f* 36.0f / 30.0f
//#define EDGE_PER_CIRCLE (515.0f * 11.0f * 4.0f * 18.0f /12.0f)
// TIM4定时器每一次跳动的时间周期
#define TIME_PERIOD 0.02

// PWM最大值
#define MAX_PWM_VAL 9000 - 1

// 左右定义
#define LEFT 0x00
#define RIGHT 0x01

// 前后定义
#define FORWARD 0x00
#define BACKWARD 0x01

void MotorInit(void);
void MotorFunction(void);
void SetAngularSpeed(uint16_t, float);
void SetSpeed(uint16_t, float);
void MotorSetGoalPosition(float goalAngle);
float GetMotorPosition(uint16_t side);
void SetMotorPosition(uint16_t side, float pos);
void EncoderCounterOverflow(uint16_t encoderNo);
void UpdateEncoderCount(void);
void PID_Control(uint16_t motorNo);
void ClearEncoderAndAngle(uint16_t encoderNo);
void SetAngularSpeedToZero(uint16_t motorNo);

#endif // _MOTOR_H
