#include <stm32f1xx.h>
#include "cmt_spi.h" 
#include "systick.h"
#include "cmt2300.h"
//#include "cmt2300_params.h"


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
void SET_GPIO_PIN_IN(GPIO_TypeDef*  GPIOx,uint16_t GPIO_PIN)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  GPIO_InitStructure.Pin = GPIO_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;//����
	GPIO_InitStructure.Pull=GPIO_PULLDOWN;        //����
  GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;    	 	//����
	
  HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);//��ʼ��
}
void SET_GPIO_PIN_OUT(GPIO_TypeDef*  GPIOx,uint16_t GPIO_PIN)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  GPIO_InitStructure.Pin = GPIO_PIN;
  GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;  	//�������;
	GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;    	 	//����
  HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);//��ʼ��GPIO
}
void CMT_SPI_Init()
{
	//GPIO_Config();
	CSB_H;
	FCSB_H;
	SCK_L;
	SET_SDIO_OUT;
	SDIO_H;
}

void cmt_spi3_delay(void)
{
    u32 n =7;
    while(n--);
}

void cmt_spi3_delay_us(void)
{
    u16 n = 8;
    while(n--);
}
void cmt_spi3_send(u8 data8)
{
    u8 i;
    for(i=0; i<8; i++)
    {
        SCK_L;

        /* Send byte on the rising edge of SCL */
        if(data8 & 0x80)
				{SDIO_H;}
        else            
				{ SDIO_L;}

        cmt_spi3_delay();

        data8 <<= 1;
        SCK_H;
        cmt_spi3_delay();
    }
}


u8 cmt_spi3_recv(void)
{
    u8 i;
    u8 data8 = 0xFF;

    for(i=0; i<8; i++)
    {
        SCK_L;
        cmt_spi3_delay();
        data8 <<= 1;

        SCK_H;

        /* Read byte on the rising edge of SCL */
        if(cmt_spi_sdio_read())
            data8 |= 0x01;
        else
            data8 &= ~0x01;

        cmt_spi3_delay();
    }

    return data8;
}
//д�Ĵ���
void cmt_spi3_write(u8 addr, u8 dat)
{
	  SDIO_H;
	  SET_SDIO_OUT;  //���
	  SCK_L;
	  FCSB_H;
   	CSB_L;
	  
	  //> 0.5 SCL cycle 
    cmt_spi3_delay();
    cmt_spi3_delay();
	
    // r/w = 0 
    cmt_spi3_send(addr&0x7F);

    cmt_spi3_send(dat);	 
    

    SCK_L;

    /* > 0.5 SCL cycle */
    cmt_spi3_delay();
    cmt_spi3_delay();

    CSB_H;
    
	  SDIO_H;
	  SET_SDIO_IN;  //����
	  FCSB_H;    
}
//���Ĵ���
void cmt_spi3_read(u8 addr, u8* p_dat)
{
	  SDIO_H;
	  SET_SDIO_OUT;  //���
	  SCK_L;
	  FCSB_H;
   	CSB_L;
	  
	  //> 0.5 SCL cycle 
    cmt_spi3_delay();
    cmt_spi3_delay();
	
    // r/w = 0 
    cmt_spi3_send(addr|0x80);
	
	  SET_SDIO_IN;  //����
	
	  *p_dat = cmt_spi3_recv();
	
	  SCK_L;

    /* > 0.5 SCL cycle */
    cmt_spi3_delay();
    cmt_spi3_delay();

    CSB_H;
    
	  SDIO_H;
	  //�SSET_SDIO_IN;  //���
	  FCSB_H;  


}
//д����
void cmt_spi3_write_fifo(const u8* p_buf, u16 len)
{
	  u16 i;
	  SDIO_H;
	  FCSB_H;
   	CSB_H;
	  SCK_L;
	  SET_SDIO_OUT;  //���
	   for(i=0; i<len; i++)
    {
        FCSB_L;

        /* > 1 SCL cycle */
        cmt_spi3_delay();
        cmt_spi3_delay();
			  cmt_spi3_delay();

        cmt_spi3_send(p_buf[i]);

        SCK_L;

        /* > 2 us */
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();

        FCSB_H;

        /* > 4 us */
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
    }

    SET_SDIO_IN;
    
    FCSB_H;
	 
}
//������
void cmt_spi3_read_fifo(u8* p_buf, u16 len)
{
   	u16 i;
	  SDIO_H;
	  FCSB_H;
   	CSB_H;
	  SCK_L;
	  SET_SDIO_IN;  //���
	   for(i=0; i<len; i++)
    {
        FCSB_L;

        /* > 1 SCL cycle */
        cmt_spi3_delay();
        cmt_spi3_delay();
			  cmt_spi3_delay();

        p_buf[i] = cmt_spi3_recv();

        SCK_L;

        /* > 2 us */
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();

        FCSB_H;

        /* > 4 us */
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
    }

    SET_SDIO_IN;
    FCSB_H;
}
//void system_delay_us(u32 n)
//{
//    delay_us(n);
//}

//void system_delay_ms(u32 nms)
//{
//    delay_ms(nms);
//}  
