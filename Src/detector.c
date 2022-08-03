#include "main.h"
#include "motor.h"
#include "detector.h"
#include "door_control.h"
#include "io.h"
#define SENSOR_QUANTITY (4U)

// 到位检测
static uint8_t doorClosed[DOOR_QUANTITY];
static bool sensorDetected[4]; // 0 and 1 for front door, 2 and 3 for rear door;
static uint8_t SensorClosedAntiJitterCount[4];
static uint8_t SensorOpenAntiJitterCount[4];

// 消抖晃计数器
static uint32_t closeAntiJitterCount[DOOR_QUANTITY];
static uint32_t openAntiJitterCount[DOOR_QUANTITY];
static const uint32_t JITTER_NUMBER = 10;

void Detector(void)
{
	uint8_t i;
	for (i = PinSensor0; i <= PinSensor3; i++)
	{
		if (ReadPinState(i))
		{
			SensorOpenAntiJitterCount[i] = 0;
			if (++SensorClosedAntiJitterCount[i] > JITTER_NUMBER)
			{
				SensorClosedAntiJitterCount[i] = 0;
				sensorDetected[i] = true;
			}
		}
		else
		{
			SensorClosedAntiJitterCount[i] = 0;
			if (++SensorOpenAntiJitterCount[i] > JITTER_NUMBER)
			{
				SensorOpenAntiJitterCount[i] = 0;
				sensorDetected[i] = false;
			}
		}
	}

	for (i = 0; i < 2; i++)
	{
		if (sensorDetected[2 * i] && sensorDetected[2 * i + 1])
		{
			doorClosed[i] = DETECT_DOOR_CLOSED;
		}
		else
		{
			if (sensorDetected[2 * i] || sensorDetected[2 * i + 1])
				doorClosed[i] = DETECT_DOOR_HALF_CLOSED;
			else
				doorClosed[i] = DETECT_DOOR_OPEN;
		}
	}
}

uint8_t DoorClosedDetected(uint16_t doorNo)
{
	return doorClosed[doorNo];
}

void InitDetector(void)
{
	uint8_t i;
	for (i = 0; i < DOOR_QUANTITY; ++i)
	{
		doorClosed[i] = DETECT_DOOR_OPEN;
	}

	for (i = 0; i < SENSOR_QUANTITY; ++i)
	{
		sensorDetected[i] = false;
		SensorClosedAntiJitterCount[i] = 0;
		SensorOpenAntiJitterCount[i] = 0;
	}
}
