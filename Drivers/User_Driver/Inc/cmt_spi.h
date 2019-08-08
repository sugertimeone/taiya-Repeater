#ifndef __CMT_SPI_H
#define __CMT_SPI_H	 
#include "sys.h" 
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//LED�˿ڶ���
//void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
//void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#define CSB_H         HAL_GPIO_WritePin(CSB_GPIO_Port,CSB_Pin,GPIO_PIN_SET)  
#define CSB_L         HAL_GPIO_WritePin(CSB_GPIO_Port,CSB_Pin,GPIO_PIN_RESET) 


#define FCSB_H         HAL_GPIO_WritePin(FCSB_GPIO_Port,FCSB_Pin,GPIO_PIN_SET)   
#define FCSB_L         HAL_GPIO_WritePin(FCSB_GPIO_Port,FCSB_Pin,GPIO_PIN_RESET) 


#define SCK_H         HAL_GPIO_WritePin(SCLK_GPIO_Port,SCLK_Pin,GPIO_PIN_SET)  
#define SCK_L         HAL_GPIO_WritePin(SCLK_GPIO_Port,SCLK_Pin,GPIO_PIN_RESET) 


#define SDIO_H            HAL_GPIO_WritePin(SDIO_GPIO_Port,SDIO_Pin,GPIO_PIN_SET)  
#define SDIO_L            HAL_GPIO_WritePin(SDIO_GPIO_Port,SDIO_Pin,GPIO_PIN_RESET)
#define cmt_spi_sdio_read()   HAL_GPIO_ReadPin(SDIO_GPIO_Port,SDIO_Pin)




#define SET_SDIO_IN      SET_GPIO_PIN_IN(SDIO_GPIO_Port,SDIO_Pin)
#define SET_SDIO_OUT     SET_GPIO_PIN_OUT(SDIO_GPIO_Port,SDIO_Pin)

#define RF_PACKET_SIZE 32            /* Define the payload size here */
void SET_GPIO_PIN_OUT_IN(GPIO_TypeDef*  GPIOx,uint16_t GPIO_PIN,uint32_t GPIO_TYPE);
void GPIO_Config(void);//��ʼ��		
void CMT_SPI_Init();

void cmt_spi3_delay(void);
void cmt_spi3_delay_us(void);
void cmt_spi3_send(u8 data8);
u8 cmt_spi3_recv(void);

void cmt_spi3_write(u8 addr, u8 dat);
void cmt_spi3_read(u8 addr, u8* p_dat);

void cmt_spi3_write_fifo(const u8* p_buf, u16 len);
void cmt_spi3_read_fifo(u8* p_buf, u16 len);

//void system_delay_us(u32 n);
//void system_delay_ms(u32 nms);
#endif

















