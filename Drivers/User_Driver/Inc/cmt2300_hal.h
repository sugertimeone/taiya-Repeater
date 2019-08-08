/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, CMOSTEK SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * Copyright (C) CMOSTEK SZ.
 */

/*!
 * @file    cmt2300_hal.h
 * @brief   CMT2300 hardware abstraction layer
 *
 * @version 1.1
 * @date    Feb 08 2017
 * @author  CMOSTEK R@D
 */
 
#ifndef __CMT2300_HAL_H
#define __CMT2300_HAL_H

#include "sys.h"
#include "stm32f1xx.h"
#include "systick.h"
#include "typedefs.h"
//#include "gpio_defs.h"
//#include "common.h"
//#include "system.h"

#ifdef __cplusplus 
extern "C" { 
#endif

/* ************************************************************************
*  The following need to be modified by user
*  ************************************************************************ */
#define Cmt2300_SetGpio1In()           SET_GPIO_IN(CMT_GPIO1_GPIO)
#define Cmt2300_SetGpio2In()           SET_GPIO_IN(CMT_GPIO2_GPIO)
#define Cmt2300_SetGpio3In()           SET_GPIO_IN(CMT_GPIO3_GPIO)
#define Cmt2300_ReadGpio1()            READ_GPIO_PIN(CMT_GPIO1_GPIO)
#define Cmt2300_ReadGpio2()            READ_GPIO_PIN(CMT_GPIO2_GPIO)
#define Cmt2300_ReadGpio3()            READ_GPIO_PIN(CMT_GPIO3_GPIO)
//#define Cmt2300_DelayMs(ms)            system_delay_ms(ms)
//#define Cmt2300_DelayUs(us)            system_delay_us(us)
#define Cmt2300_DelayMs(ms)            delay_ms(ms)
#define Cmt2300_DelayUs(us)            delay_us(us)
#define Cmt2300_GetTickCount()         g_nSysTickCount
/* ************************************************************************ */

void Cmt2300_InitGpio(void);

u8 Cmt2300_ReadReg(u8 addr);
void Cmt2300_WriteReg(u8 addr, u8 dat);

void Cmt2300_ReadFifo(u8 buf[], u16 len);
void Cmt2300_WriteFifo(const u8 buf[], u16 len);

#ifdef __cplusplus 
} 
#endif

#endif
