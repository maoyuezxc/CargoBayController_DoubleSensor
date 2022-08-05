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
#define OPEN_ANGLE (PI * 90.0f / 180.0f) // 95 degrees，90 degree - wolf on 2022.07.11
#define OPEN_ANGLE_FOR_CLOSE_JAM (PI * 25.0f / 180.0f)
#define CLOSE_ANGLE (PI * -100.0f / 180.0f) // -105 degrees, 100 degree - wolf on 2022.07.11
static const float TIGHT_ANGLE[DOOR_QUANTITY] = {PI * -12.0f / 180.0f, PI * -18.0f / 180.0f};

// original value: {PI * -12.0f / 180.0f, PI * -16.0f / 180.0f};

#define REPORT_STATUS_TIME 200
#define WAIT_SENSOR_TIME 300
#define CLOSE_ANGLE_TH (PI * 70.0f / 180.0f)
#define OPEN_ANGLE_TH (PI * 20.0f / 180.0f)
#define FRONT_OPEN_OFFSET (PI * 3.0f / 180.0f)

typedef struct
{
    uint8_t cargo1Status; // 0 - opened 1 - opening 2 - closing 3 -closed 4 - fault
    uint8_t cargo2Status; // 0 - opened 1 - opening 2 - closing 3 -closed 4 - fault
} CargoLidStatus_t;

/* ----------------- Static variables ------------ */
// 门状态
LidStatusTypeDef doorState[DOOR_QUANTITY];

// 目标角度
static float goalAngle[DOOR_QUANTITY];
// 系统Tick
static uint32_t lastTick;
bool IsPowerOnJam[DOOR_QUANTITY] = {true, true};

/* --------------Static Functions---------------- */
static void TightDoor(uint16_t doorNo);
static void ReportLidStatus(void);

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

    if (doorState[FRONT] == LID_STATUS_FAULT && IsPowerOnJam[FRONT])
    {
        doorState[FRONT] = LID_STATUS_OPENING;
        ClearEncoderAndAngle(FRONT);
        goalAngle[FRONT] = OPEN_ANGLE_FOR_CLOSE_JAM;
        SetAngularSpeed(FRONT, ANGULAR_SPEED);
        return;
    }

    else if (doorState[REAR] == LID_STATUS_FAULT && IsPowerOnJam[REAR])
    {
        doorState[REAR] = LID_STATUS_OPENING;
        ClearEncoderAndAngle(REAR);
        goalAngle[REAR] = OPEN_ANGLE_FOR_CLOSE_JAM;
        SetAngularSpeed(REAR, ANGULAR_SPEED);
        return;
    }
    // the original state:
    if (doorState[doorNo] == LID_STATUS_CLOSED ||
        doorState[doorNo] == LID_STATUS_FAULT)
    {
        doorState[doorNo] = LID_STATUS_OPENING;
        if (doorNo == FRONT)
            goalAngle[doorNo] = OPEN_ANGLE + FRONT_OPEN_OFFSET;
        else if (doorNo == REAR)
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
    if (LID_STATUS_OPENED == doorState[doorNo] ||
        LID_STATUS_READY == doorState[doorNo])
    {
        doorState[doorNo] = LID_STATUS_CLOSING;
        goalAngle[doorNo] = CLOSE_ANGLE;
        ClearOverCurrentStatus(doorNo);
        SetAngularSpeed(doorNo, -ANGULAR_SPEED);
    }
}

void DoorReset(uint16_t doorNo)
{
    if (LID_STATUS_FAULT == doorState[REAR])
    {
        doorState[REAR] = LID_STATUS_READY;
        ClearOverCurrentStatus(REAR);
    }

    if (LID_STATUS_FAULT == doorState[FRONT])
    {
        doorState[FRONT] = LID_STATUS_READY;
        ClearOverCurrentStatus(FRONT);
    }
}

void TightDoor(uint16_t i)
{
    ClearEncoderAndAngle(i);
    goalAngle[i] = TIGHT_ANGLE[i];
    SetAngularSpeed(i, -ANGULAR_SPEED);
    doorState[i] = LID_STATUS_TIGHTING;
}

LidStatusTypeDef GetDoorStatus(uint16_t DoorNo)
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
                doorState[i] = LID_STATUS_CLOSED;
            }
            else
            {
                if (i == REAR)
                {
                    if (doorState[FRONT] == LID_STATUS_CLOSED)
                        DoorClose(i);
                }
                else if (i == FRONT)
                {
                    DoorClose(i);
                }
            }
            break;

        case LID_STATUS_CLOSING:
            if (GetOverCurrentStatus(i))
            {
                if (IsPowerOnJam[i])
                    doorState[i] = LID_STATUS_FAULT;
                else
                {
                    doorState[FRONT] = LID_STATUS_FAULT;
                    doorState[REAR] = LID_STATUS_FAULT;
                }

                ClearOverCurrentStatus(i);
            }
            else
            {
                if (DoorClosedDetected(i) == DETECT_DOOR_CLOSED)
                {
                    TightDoor(i);
                }

                else
                {
                    if (FRONT == i)
                        if (GetMotorPosition(i) < CLOSE_ANGLE_TH &&
                            doorState[REAR] == LID_STATUS_OPENED)
                            DoorClose(REAR);
                }
            }
            break;

        case LID_STATUS_TIGHTING:
            if (GetOverCurrentStatus(i))
            {
                doorState[i] = LID_STATUS_FAULT;
                ClearOverCurrentStatus(i);
            }
            else if (GetMotorPosition(i) < goalAngle[i])
            {
                doorState[i] = LID_STATUS_CLOSED;
            }
            break;

        case LID_STATUS_OPENING:
            if (GetOverCurrentStatus(i))
            {
                doorState[REAR] = LID_STATUS_FAULT;
                doorState[FRONT] = LID_STATUS_FAULT;
                ClearOverCurrentStatus(i);
            }

            else
            {
                if (GetMotorPosition(i) >= goalAngle[i])
                {
                    // SetAngularSpeedToZero(i);
                    doorState[i] = LID_STATUS_OPENED;
                    if (IsPowerOnJam[i])
                    {
                        doorState[FRONT] = LID_STATUS_FAULT;
                        doorState[REAR] = LID_STATUS_FAULT;
                    }
                }

                else
                {
                    if (REAR == i)
                    {

                        if (GetMotorPosition(i) >= OPEN_ANGLE_TH &&
                            (doorState[FRONT] == LID_STATUS_CLOSED ||
                             doorState[FRONT] == LID_STATUS_FAULT))
                            DoorOpen(FRONT);
                    }
                }
            }

            break;

        case LID_STATUS_CLOSED:
            IsPowerOnJam[i] = false;
        case LID_STATUS_FAULT:
        case LID_STATUS_OPENED:
            SetAngularSpeedToZero(i);
            break;

        default:
            break;
        }
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
    //    --lidId;
    //   if (lidId >= DOOR_QUANTITY)
    //       return;
    switch (action)
    {
    case LID_CLOSE:
        DoorClose(FRONT);
        break;

    case LID_OPEN:
        DoorOpen(REAR);
        break;

    case LID_RESET:
        DoorReset(FRONT);
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
