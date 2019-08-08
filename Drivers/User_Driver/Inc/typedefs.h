#ifndef _TYPEDEFS_H_
#define _TYPEDEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include "stm32f10x.h"
#include "stm32f1xx.h"
	
#define xdata
#define data
#define code const

#define NOP() __NOP()

#ifndef u8
#define u8  unsigned char
#endif

#ifndef u16
#define u16 unsigned short
#endif

#ifndef u32
#define u32 unsigned long
#endif

#ifndef U8
#define U8  unsigned char
#endif

#ifndef S8
#define S8  char
#endif

#ifndef U16
#define U16 unsigned short
#endif

#ifndef U32
#define U32 unsigned long
#endif

#ifndef BOOL
#define BOOL u8
#endif

#ifndef TRUE
#define TRUE (BOOL)SET
#endif

#ifndef FALSE
#define FALSE (BOOL)RESET
#endif

#define INFINITE 0xFFFFFFFF



#define ENABLE_ANTENNA_SWITCH
#ifdef __cplusplus
}
#endif



#endif
