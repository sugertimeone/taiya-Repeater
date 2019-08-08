#include"systick.h"
void delay_us(u32 nus)
{
	 u32 temp;
	 SysTick->LOAD = 9*nus;
	 SysTick->VAL=0X00;       //清空计数器
	 SysTick->CTRL=0X01;      //使能，减到0是无动作，采用外部时钟源
	 do
	 {
	  temp=SysTick->CTRL;     //读取当前倒计数值
	 }while((temp&0x01)&&(!(temp&(1<<16))));       //等待时间到达
	 
	 SysTick->CTRL=0x00;      //关闭计数器
	 SysTick->VAL =0X00;      //清空计数器
}
/**
  * 名    称：delay_ms(u32 mus)
  * 功    能：毫秒延时函数
  * 入口参数：u32 mus
  * 出口参数：无
  * 说    明：
  * 调用方法：无
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
