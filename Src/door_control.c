#include "main.h"
#include "can.h"
#include "motor.h"
#include <string.h>
#include "detector.h"
#include "door_control.h"
#include "OverCurrentProtection.h"

/* ------------------ Speed parameters ----------------------------------*/
#define ANGULAR_SPEED 0.27f
#define ANGULAR_ACCELARATION 0.02f
#define DOOR_HALF_CLOSED_DELAY (1000)

/* ------------------- Angular parameters ------------------------------- */
#define ANGULAR_DEAD_ZONE 0.02f
#define OPEN_ANGLE (PI * 90.0f / 180.0f)    // 95 degrees，90 degree - wolf on 2022.07.11
#define CLOSE_ANGLE (PI * -100.0f / 180.0f) // -105 degrees, 100 degree - wolf on 2022.07.11
const float TIGHT_ANGLE[DOOR_QUANTITY] = {PI * -12.0f / 180.0f, PI * -12.0f / 180.0f};
// original value: {PI * -12.0f / 180.0f, PI * -16.0f / 180.0f};

#define REPORT_STATUS_TIME 200
#define WAIT_SENSOR_TIME 300

typedef struct
{
    uint8_t cargo1Status; // 0 - opened 1 - opening 2 - closing 3 -closed 4 - fault
    uint8_t cargo2Status; // 0 - opened 1 - opening 2 - closing 3 -closed 4 - fault
} CargoLidStatus_t;

/* ----------------- Static variables ------------ */
// 门状态
static uint32_t doorState[DOOR_QUANTITY];
// 目标角度
static float goalAngle[DOOR_QUANTITY];
// 系统Tick
static uint32_t lastTick;

static void ReportLidStatus(void);

static uint32_t HalfClosedDelay;

/** Initialize door control, parameters ...
 * @param none
 * @retval none
 */
void InitDoorControl(void)
{
    for (int i = 0; i < DOOR_QUANTITY; ++i)
    {
        doorState[i] = LID_STATUS_READY;
        goalAngle[i] = 0;
    }
}

/**
 * @brief Open the door
 * @param leftOrRight LEFT-Front door RIGHT-Rear door
 * @retval None
 */
void DoorOpen(uint16_t doorNo)
{
    if (DoorClosedDetected(doorNo) != DETECT_DOOR_OPEN)
    {
        doorState[doorNo] = LID_STATUS_OPENING;
        goalAngle[doorNo] = OPEN_ANGLE;
        ClearOverCurrentStatus(doorNo);
        SetAngularSpeed(doorNo, ANGULAR_SPEED);
    }
}

/**
 * @brief Close the door
 * @param leftOrRight LEFT-Front door RIGHT-Rear door
 * @retval None
 */
void DoorClose(uint16_t doorNo)
{
    if (!(LID_STATUS_CLOSED == doorState[doorNo] || LID_STATUS_CLOSING == doorState[doorNo]))
    {
        doorState[doorNo] = LID_STATUS_CLOSING;
        goalAngle[doorNo] = CLOSE_ANGLE;
        ClearOverCurrentStatus(doorNo);
        SetAngularSpeed(doorNo, -ANGULAR_SPEED);
    }
}

void DoorReset(uint16_t doorNo)
{
    if (LID_STATUS_FAULT == doorState[doorNo])
    {
        doorState[doorNo] = LID_STATUS_READY;
        ClearOverCurrentStatus(doorNo);
    }
}

uint16_t ll(uint16_t DoorNo)
{
    return doorState[DoorNo];
}

void DoorControlFunction(void)
{
    for (int i = 0; i < DOOR_QUANTITY; ++i)
    {
        switch (doorState[i])
        {
        case LID_STATUS_READY:
            if (HAL_GetTick() < WAIT_SENSOR_TIME)
                break;
            if (DETECT_DOOR_CLOSED == DoorClosedDetected(i))
            {
                ClearEncoderAndAngle(i);
                goalAngle[i] = 0;
                SetAngularSpeed(i, 0);
                doorState[i] = LID_STATUS_CLOSED;
            }
            else
            {
                doorState[i] = LID_STATUS_CLOSING;
                goalAngle[i] = CLOSE_ANGLE;
                SetAngularSpeed(i, -ANGULAR_SPEED);
            }
            break;

        case LID_STATUS_CLOSING:
            if (DoorClosedDetected(i) == DETECT_DOOR_HALF_CLOSED)
            {
                ClearEncoderAndAngle(i);
                goalAngle[i] = TIGHT_ANGLE[i];
                SetAngularSpeed(i, -ANGULAR_SPEED); //-ANGULAR_SPEED
                // doorState[i] = LID_STATUS_CLOSED;
                doorState[i] = LID_STATUS_TIGHTING;
                HalfClosedDelay = HAL_GetTick();
            }
            else if (GetOverCurrentStatus(i))
                doorState[i] = LID_STATUS_FAULT;
            break;

        case LID_STATUS_TIGHTING:
            if (DoorClosedDetected(i) == DETECT_DOOR_CLOSED)
            {
                SetAngularSpeedToZero(i);
                doorState[i] = LID_STATUS_CLOSED;
                ClearOverCurrentStatus(i);
            }
            else if (GetOverCurrentStatus(i) || HAL_GetTick() - HalfClosedDelay > DOOR_HALF_CLOSED_DELAY)
            {
                SetAngularSpeedToZero(i);
                doorState[i] = LID_STATUS_FAULT;
                ClearOverCurrentStatus(i);
            }
            break;

        case LID_STATUS_OPENING:
            if (GetOverCurrentStatus(i))
                doorState[i] = LID_STATUS_FAULT;
            if (GetMotorPosition(i) >= OPEN_ANGLE)
            {
                SetAngularSpeedToZero(i);
                doorState[i] = LID_STATUS_OPENED;
            }
            break;

        case LID_STATUS_FAULT:
            SetAngularSpeedToZero(i);
            break;

        default:
            break;
        }

        //        float diffAngle = goalAngle[i] - GetMotorPosition(i);
        //        if (diffAngle < ANGULAR_DEAD_ZONE && diffAngle > -ANGULAR_DEAD_ZONE)
        //        {
        //            // SetAngularSpeed(i, 0);
        //            SetAngularSpeedToZero(i);
        //            if (LID_STATUS_OPENING == doorState[i])
        //                doorState[i] = LID_STATUS_OPENED;
        //
        //        }
    }

    uint32_t timeInterval = HAL_GetTick() - lastTick;
    if (timeInterval >= REPORT_STATUS_TIME)
    {
        ReportLidStatus();
        lastTick += timeInterval;
    }
}

/**
 * @brief Inform the door control module command received.
 * Provide an interface for CAN module to inform the door control module
 * when there're new commands received. The function will analysis the
 * message and control the lid as the command.
 * @param lidId 1 - Front lid 2 - Rear Lid
 * @param action 0 - close 1 - open 2 - reset
 * @retval none
 */
void InformDoorControl(uint16_t lidId, uint16_t action)
{
    --lidId;
    if (lidId >= DOOR_QUANTITY)
        return;
    switch (action)
    {
    case LID_CLOSE:
        DoorClose(lidId);
        break;

    case LID_OPEN:
        DoorOpen(lidId);
        break;

    case LID_RESET:
        DoorReset(lidId);
        break;

    default:
        break;
    }
}

void ReportLidStatus(void)
{
    CargoLidStatus_t cargoLidStatus;
    cargoLidStatus.cargo1Status = doorState[0];
    cargoLidStatus.cargo2Status = doorState[1];
    CanSendMessageToRcu((uint8_t *)&cargoLidStatus, sizeof(cargoLidStatus));
}

void SetLidStatus(uint16_t lidNo, uint16_t status)
{
    doorState[lidNo] = status;
}
