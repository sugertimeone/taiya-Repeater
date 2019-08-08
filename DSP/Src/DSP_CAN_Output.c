#include "DSP_CAN_Output.h"
void CAN_Send(uint8_t *pData)
{
	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;
	hcan.pTxMsg->DLC= 8;
	hcan.pTxMsg->IDE = CAN_ID_EXT;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;
	hcan.pTxMsg->ExtId = 0x18fe0018;
	for(uint8_t i=0;i<16;i++)
		hcan.pTxMsg->Data[i]=pData[i];
	HAL_CAN_Transmit(&hcan,1);
}
