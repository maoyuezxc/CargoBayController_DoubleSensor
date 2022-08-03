#pragma once
enum DetectorState
{
    DETECT_DOOR_OPEN = 0,
    DETECT_DOOR_HALF_CLOSED,
    DETECT_DOOR_CLOSED,
};

void Detector(void);
uint8_t DoorClosedDetected(uint16_t doorNo);
void InitDetector(void);
