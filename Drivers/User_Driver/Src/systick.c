#include"systick.h"
void delay_us(u32 nus)
{
	 u32 temp;
	 SysTick->LOAD = 9*nus;
	 SysTick->VAL=0X00;       //��ռ�����
	 SysTick->CTRL=0X01;      //ʹ�ܣ�����0���޶����������ⲿʱ��Դ
	 do
	 {
	  temp=SysTick->CTRL;     //��ȡ��ǰ������ֵ
	 }while((temp&0x01)&&(!(temp&(1<<16))));       //�ȴ�ʱ�䵽��
	 
	 SysTick->CTRL=0x00;      //�رռ�����
	 SysTick->VAL =0X00;      //��ռ�����
}
/**
  * ��    �ƣ�delay_ms(u32 mus)
  * ��    �ܣ�������ʱ����
  * ��ڲ�����u32 mus
  * ���ڲ�������
  * ˵    ����
  * ���÷�������
  **/
void delay_ms(u16 nms)
{
	for(uint16_t i=0;i<nms;i++)
	delay_us(1000);
}
//void delay_ms(u16 nms)
//{
//	 u32 temp;
//	 SysTick->LOAD = 9000*nms;
//	 SysTick->VAL=0X00;
//	 SysTick->CTRL=0X01;
//	 do
//	 {
//	  temp=SysTick->CTRL;
//	 }while((temp&0x01)&&(!(temp&(1<<16))));
//	 SysTick->CTRL=0x00;
//	 SysTick->VAL =0X00; 
//}
