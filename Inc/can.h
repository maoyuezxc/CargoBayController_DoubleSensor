#pragma once

#include <stdint.h>
#include <stdbool.h>

void InitCan(void);
void CanFunction(void);
void CanSendMessageToRcu(uint8_t *message, uint32_t size);
