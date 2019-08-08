#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"  

//////////////////////////////////////////////////////////////////////////////////////////////////////
//�û������Լ�����Ҫ����
#define STM32_FLASH_SIZE 	512 	 		//��ѡSTM32��FLASH������С(��λΪK)
#define STM32_FLASH_WREN 	1         //ʹ��FLASHд��(0��������;1��ʹ��)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ

#define FLASH_ADDR_SET   0X0800C000 	//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000) 
#define SIZE_U8 1											//V1.02.4�޶� VehicleType u8���� 1���ֽ�
#define SIZE_U16 2										//VehicleType u16����2���ֽ�					
#define ADDR_PRESSURE_OFFSET 0
#define ADDR_TEMPERATURE_OFFSET 6*2 
#define	ADDR_SET_PRESS_FRONT_HIGH_OFFSET 12*2
#define	ADDR_SET_PRESS_FRONT_LOW_OFFSET 13*2
#define	ADDR_SET_PRESS_REAR_HIGH_OFFSET 14*2
#define	ADDR_SET_PRESS_REAR_LOW_OFFSET 15*2
#define ADDR_SET_T_OFFSET 16*2
#define ADDR_ID_OFFSET 17*2	
#define ADDR_TYPE_OFFSET 29*2	

//FLASH������ֵ
//#define FLASH_KEY1               0X45670123
//#define FLASH_KEY2               0XCDEF89AB
void STMFLASH_Unlock(void);					  //FLASH����
void STMFLASH_Lock(void);					  //FLASH����
u8 STMFLASH_GetStatus(void);				  //���״̬
u8 STMFLASH_WaitDone(u16 time);				  //�ȴ���������
u8 STMFLASH_ErasePage(u32 paddr);			  //����ҳ
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);//д�����
u16 STMFLASH_ReadHalfWord(u32 faddr);		  //��������  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//ָ����ַ��ʼд��ָ�����ȵ�����
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);											//ָ����ַ��ʼ��ȡָ����������
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);			//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   			//��ָ����ַ��ʼ����ָ�����ȵ�����

//����д��
void Test_Write(u32 WriteAddr,u16 WriteData);								   
#endif

















