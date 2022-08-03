#pragma once

void UpdateOverCurrentProtection(uint16_t motorNo, float current);
bool GetOverCurrentStatus(uint16_t motorNo);
void ClearOverCurrentStatus(uint16_t motorNo);
