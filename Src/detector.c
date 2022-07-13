#include "main.h"
#include "motor.h"
#include "detector.h"
#include "door_control.h"

// 到位检测
bool doorClosed[DOOR_QUANTITY];
// 消抖晃计数器
static uint32_t closeAntiJitterCount[DOOR_QUANTITY];
static uint32_t openAntiJitterCount[DOOR_QUANTITY];
static const uint32_t JITTER_NUMBER = 10;

void Detector(void)
{
	if (HAL_GPIO_ReadPin(SENSOR_0_GPIO_Port, SENSOR_0_Pin))
	{ // 检测到门关好信号，假如是第一次检测到，复位编码器的值为零
		openAntiJitterCount[0] = 0;
		if (++closeAntiJitterCount[0] > JITTER_NUMBER)
		{
			// if(!doorClosed[0]) ClearEncoderAndAngle(0);
			closeAntiJitterCount[0] = 0;
			doorClosed[0] = true;
		}
	}
	else
	{
		closeAntiJitterCount[0] = 0;
		if (++openAntiJitterCount[0] > JITTER_NUMBER)
		{
			openAntiJitterCount[0] = 0;
			doorClosed[0] = false;
		}
	}
	if (HAL_GPIO_ReadPin(SENSOR_1_GPIO_Port, SENSOR_1_Pin))
	{ // 检测到门关好信号，假如是第一次检测到，复位编码器的值为零
		openAntiJitterCount[1] = 0;
		if (++closeAntiJitterCount[1] > JITTER_NUMBER)
		{
			// if(!doorClosed[1]) ClearEncoderAndAngle(1);
			closeAntiJitterCount[1] = 0;
			doorClosed[1] = true;
		}
	}
	else
	{
		closeAntiJitterCount[1] = 0;
		if (++openAntiJitterCount[1] > JITTER_NUMBER)
		{
			openAntiJitterCount[1] = 0;
			doorClosed[1] = false;
		}
	}
}

bool IsDoorClosed(uint16_t doorNo)
{
	return doorClosed[doorNo];
}

void InitDetector(void)
{
	for (int i = 0; i < DOOR_QUANTITY; ++i)
	{
		doorClosed[i] = false;
		closeAntiJitterCount[i] = 0;
		openAntiJitterCount[i] = 0;
	}
}
