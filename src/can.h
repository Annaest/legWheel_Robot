#ifndef __CAN_H__
#define __CAN_H__

#include "config.h"


void CAN_RecvCallback(uint32_t id, uint8_t *data);
void CAN_RecvTask(void *arg);
void CAN_Init();
void CAN_SendFrame(uint32_t id, uint8_t *data);

#endif