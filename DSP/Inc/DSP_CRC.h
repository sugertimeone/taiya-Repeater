#ifndef __DSP_CRC_H_
#define __DSP_CRC_H_
#include "main.h"
#include "stm32f1xx_hal.h"

unsigned int CRC_OutData(uint8_t *data,char size);
unsigned int cal_crc(unsigned char *ptr, unsigned char len);
#endif
