#pragma once

#define DOOR_QUANTITY 2
#define FRONT_DOOR 0
#define REAR_DOOR

/* ----------------- Lid actions and status --------------------------- */
enum LID_ACTION
{
    LID_CLOSE = 0,
    LID_OPEN,
    LID_RESET
};

enum LID_STATUS
{
    LID_STATUS_OPENED = 0,
    LID_STATUS_OPENING,
    LID_STATUS_CLOSING,
    LID_STATUS_CLOSED,
    LID_STATUS_FAULT,
    LID_STATUS_READY,
    LID_STATUS_TIGHTING
};

void DoorControlFunction(void);
void InitDoorControl(void);
void InformDoorControl(uint16_t lidId, uint16_t action);
void SetLidStatus(uint16_t lidNo, uint16_t status);
uint16_t GetDoorStatus(uint16_t DoorNo);
