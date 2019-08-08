#ifndef __DSP_CAN_OUTPUT_H_
#define __DSP_CAN_OUTPUT_H_
#include "stm32f1xx_hal.h"
#include "can.h"
static CanRxMsgTypeDef RxMessage;      //接收消息
static CanTxMsgTypeDef TxMessage;			//发送消息
void CAN_Send(uint8_t *pData);
#endif
