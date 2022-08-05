#include "main.h"
#include "door_control.h"
#include "motor.h"
#include "OverCurrentProtection.h"

// 电机平均象征电流0 - 1.0
static float averageCurrent[DOOR_QUANTITY];
// 电机电流保护门限
static const float CLOSE_LID_PROTECTION_CURRENT = -0.44f;
static const float OPEN_LID_PROTECTION_CURRENT = 0.50f;
// Over-current status
bool overCurrent[DOOR_QUANTITY];

void UpdateOverCurrentProtection(uint16_t motorNo, float current)
{
	averageCurrent[motorNo] = averageCurrent[motorNo] * 0.9f + current * 0.1f;
	if (averageCurrent[motorNo] > OPEN_LID_PROTECTION_CURRENT || averageCurrent[motorNo] < CLOSE_LID_PROTECTION_CURRENT)
	{
		// SetAngularSpeed(motorNo, 0);
		// SetAngularSpeedToZero(motorNo);
		// SetLidStatus(motorNo, LID_STATUS_FAULT);
		overCurrent[motorNo] = true;
	}
}

/**
 * @brief Get the Over Current Status
 *
 * @param motorNo motor's ID.
 * @return true There's an over-current situation happens.
 * @return false No over-current happens.
 */
bool GetOverCurrentStatus(uint16_t motorNo)
{
	if (motorNo >= DOOR_QUANTITY)
		return false;
	return overCurrent[motorNo];
}

void ClearOverCurrentStatus(uint16_t motorNo)
{
	if (motorNo >= DOOR_QUANTITY)
		return;
	overCurrent[motorNo] = false;
}
