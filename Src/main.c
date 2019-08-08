
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "can.h"
/* USER CODE BEGIN Includes */
#include "stmflash.h"
#include"systick.h"
#include "cmt_spi.h"
#include "cmt2300.h"
#include "radio.h"
#include "DSP_CRC.h"

#include "DSP_CAN_Output.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//#define VEHICLE_TYPE_BUS            1
//#define TWO_AXIS_SINGLE_WHEEL       4//双轴单轮
//uint16_t ID[4]={25237,25228,25246,25234};
//#define THREE_AXIS_SINGLE_WHEEL     6//三轴单轮
//uint16_t ID[6]={25237,25228,25246,25234,25224,25242};
//#define TWO_AXIS_TWO_WHEEL          8//双轴双轮
//uint16_t ID[8]={25237,25228,25246,25234,25224,25242};
//#define THREE_AXIS_TWO_WHEEL          12//三轴双轮

uint16_t ID[12]={25237,25228,25246,25234,25224,50697};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void OnMaster(void);
void OnSlave(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t SP_receive_end_flag;  //接收到数据的标志
uint8_t g_rxBuffer[RF_PACKET_SIZE];  //用来存接受到传感器的数据
uint8_t g_txBuffer[RF_PACKET_SIZE];   //发送的数据，通过CMT2300A
u16 vehicle_type=1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
	uint8_t abc;
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t ID_read_num=0;
	uint8_t send[16]={0};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/
 
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
	MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  RF_Init();
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Stop_IT(&htim4); 
	delay_ms(1000);         //延时一段时间等待显示器初始化完成
	//STMFLASH_Write(FLASH_ADDR_SET+ADDR_TYPE_OFFSET, &vehicle_type, SIZE_U8*6);
	STMFLASH_Read(FLASH_ADDR_SET+ADDR_TYPE_OFFSET, &vehicle_type, SIZE_U8);
	#ifdef VEHICLE_TYPE_BUS   //普通客车
	if(vehicle_type!=1)
	{
	vehicle_type=1;
	STMFLASH_Write(FLASH_ADDR_SET+ADDR_TYPE_OFFSET, &vehicle_type, SIZE_U8);      //写入车的类型
	}
	#endif
	#ifdef TWO_AXIS_SINGLE_WHEEL   //双轴单轮
	if(vehicle_type!=4)
	{
	vehicle_type=4;
	STMFLASH_Write(FLASH_ADDR_SET+ADDR_TYPE_OFFSET, &vehicle_type, SIZE_U8);      //写入车的类型
	}  
	#endif
	#ifdef THREE_AXIS_SINGLE_WHEEL  //三轴单轮
	if(vehicle_type!=6)
	{
	vehicle_type=6;
	STMFLASH_Write(FLASH_ADDR_SET+ADDR_TYPE_OFFSET, &vehicle_type, SIZE_U8);      //写入车的类型
	}
//	STMFLASH_Read(FLASH_ADDR_SET+ADDR_ID_OFFSET, &ID_read_num, 1);
//	if(ID_read_num==0xFFFF)
//	{
//		STMFLASH_Write(FLASH_ADDR_SET+ADDR_ID_OFFSET, ID, SIZE_U8*6);
//	}
//	else
//	{
//		STMFLASH_Read(FLASH_ADDR_SET+ADDR_ID_OFFSET, ID, SIZE_U8*6);
//	}
	#endif
	#ifdef TWO_AXIS_TWO_WHEEL  //双轴双轮
	if(vehicle_type!=8)
	{
	vehicle_type=8;
	STMFLASH_Write(FLASH_ADDR_SET+ADDR_TYPE_OFFSET, &vehicle_type, SIZE_U8);      //写入车的类型
	}
	#endif
	#ifdef THREE_AXIS_TWO_WHEEL   //三轴双轮
	if(vehicle_type!=12)
	{
	vehicle_type=12;
	STMFLASH_Write(FLASH_ADDR_SET+ADDR_TYPE_OFFSET, &vehicle_type, SIZE_U8);      //写入车的类型
	}
	#endif
	STMFLASH_Read(FLASH_ADDR_SET+ADDR_ID_OFFSET, &ID_read_num, 1);	
	if(vehicle_type==1)
	{
		if(ID_read_num==0xFFFF)
		{
			STMFLASH_Write(FLASH_ADDR_SET+ADDR_ID_OFFSET, ID, SIZE_U8*6);
		}
		else
		{
			STMFLASH_Read(FLASH_ADDR_SET+ADDR_ID_OFFSET, ID, SIZE_U8*6);
		}
	}
	else
	{
		if(ID_read_num==0xFFFF)
		{
			STMFLASH_Write(FLASH_ADDR_SET+ADDR_ID_OFFSET, ID, SIZE_U8*vehicle_type);
		}
		else
		{
			STMFLASH_Read(FLASH_ADDR_SET+ADDR_ID_OFFSET, ID, SIZE_U8*vehicle_type);
		}
	}
	//告诉显示器挂车的类型
	send[14]=vehicle_type;
	send[15]=0x00;
	send[0]=0x01;
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
//	delay_ms(500);
//	delay_ms(500);
//	delay_ms(500);	
//	delay_ms(500);
	CAN_Send((uint8_t*)send);
	HAL_UART_Transmit(&huart2,(uint8_t*)send,16,1000);	//发送数据给显示器
	while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);	//等待发送结束
	delay_ms(3);
	CAN_Send((uint8_t*)send);
	HAL_UART_Transmit(&huart2,(uint8_t*)send,16,1000);	//发送数据给显示器
	while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);	//等待发送结束
	while(1)
  {
		  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//	delay_ms(500);
//	delay_ms(500);
//			delay_ms(500);
//	delay_ms(500);
//		RF_StartTx(g_txBuffer, RF_PACKET_SIZE, 0);  
//		g_txBuffer[2]+=1;
//		if(g_txBuffer[2]==255)
//		g_txBuffer[2]=1;	
//		//		g_txBuffer[0]	=0x00;
//		g_txBuffer[1] =0x00;
//		g_txBuffer[2] =0x11;
//		g_txBuffer[3] =0x11;
//		g_txBuffer[4] =0x00;
//		g_txBuffer[5] =0x50;
//		g_txBuffer[6] =0x1C;
//		g_txBuffer[7] =0x00;
//		g_txBuffer[8] =0x00;
//		g_txBuffer[9] =0x00;
//		g_txBuffer[10]=0x00;
//		g_txBuffer[11]=0x50;
//		g_txBuffer[12]=0x1C;
//		g_txBuffer[13]=0x00;
//		g_txBuffer[14]=0x00;
//		g_txBuffer[15]=CRC_OutData(g_txBuffer,15);
//if(g_rxBuffer[15]==CRC_OutData(g_rxBuffer,15)&&g_rxBuffer[3]!=0)
//			RF_StartTx(g_rxBuffer, RF_PACKET_SIZE, 0); 
//				CAN_Send((uint8_t*)g_rxBuffer);
	 if(SP_receive_end_flag==1)  //接收到数据
	 {
		  if((g_rxBuffer[0]==0xfe)&&(g_rxBuffer[1]==0x68))
			{
				 g_txBuffer[0]=g_rxBuffer[0];
				 g_txBuffer[1]=g_rxBuffer[1];
				 for(uint8_t i=0;i<6;i++)
				 {
					 g_txBuffer[2+2*i]=ID[i]>>8;
					 g_txBuffer[3+2*i]=ID[i]&0xff;
				 }
				 g_txBuffer[14]=vehicle_type;
				 g_txBuffer[15]=1;
				 RF_StartTx(g_txBuffer, RF_PACKET_SIZE, 0);
				 delay_ms(500);
				 for(uint8_t i=0;i<6;i++)
				 {
					 g_txBuffer[2+2*i]=ID[6+i]>>8;
					 g_txBuffer[3+2*i]=ID[6+i]&0xff;
				 }	
				 g_txBuffer[15]=2;
				 RF_StartTx(g_txBuffer, RF_PACKET_SIZE, 0);
				 //delay_ms(200);				 
			}
		 	if(g_rxBuffer[15]==0xfe)	
			{
				  RF_StartTx(g_rxBuffer, RF_PACKET_SIZE, 0);
			}	
			else
			{
				
				if(g_rxBuffer[15]==CRC_OutData(g_rxBuffer,15))
				{
					RF_StartTx(g_rxBuffer, RF_PACKET_SIZE, 0); 
					delay_ms(200);
				}
				CAN_Send((uint8_t*)g_rxBuffer);
					if(g_rxBuffer[14]==0x00)  //接收到传感器的数据
					{
						u16 dat=0;
						dat=g_rxBuffer[2]<<8|g_rxBuffer[3];  //	ID值
						if(dat!=0)
						{
						 if((g_rxBuffer[4]==g_rxBuffer[10])&&(g_rxBuffer[5]==g_rxBuffer[11])&&(g_rxBuffer[6]==g_rxBuffer[12])&&(g_rxBuffer[7]==g_rxBuffer[13]))
						 { 
								g_rxBuffer[14]=vehicle_type;
								if(vehicle_type==1)
								{
									CAN_Send((uint8_t*)g_rxBuffer);
									HAL_UART_Transmit(&huart2,(uint8_t*)g_rxBuffer,16,1000);	//发送数据给显示器
									while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);	//等待发送结束
								}
								else
								{
									uint8_t i=0;
									for(i=0;i<g_rxBuffer[14];i++)
									{
										if(dat==ID[i])
										{
											g_rxBuffer[15]=i+1;//轮胎的编号
											CAN_Send((uint8_t*)g_rxBuffer);
											HAL_UART_Transmit(&huart2,(uint8_t*)g_rxBuffer,16,1000);	//发送数据给显示器
											while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);	//等待发送结束
											break;
										}	
									} 
								 }
						  }  
						}
					}
					else if((g_rxBuffer[14]==0x40)||(g_rxBuffer[14]==0x60)||(g_rxBuffer[14]==0x80)||(g_rxBuffer[14]==0xc0)||(g_rxBuffer[14]==0x10))  //更新挂车ID的标志
					{
						 u8 i=0;
						 #if (defined VEHICLE_TYPE_BUS||defined TWO_AXIS_SINGLE_WHEEL||defined THREE_AXIS_SINGLE_WHEEL||defined TWO_AXIS_TWO_WHEEL||defined THREE_AXIS_TWO_WHEEL)
						 #else
						 vehicle_type=g_rxBuffer[14]/16;
						 STMFLASH_Write(FLASH_ADDR_SET+ADDR_TYPE_OFFSET,&vehicle_type, SIZE_U8);	
						 send[14]=vehicle_type;
//						 HAL_UART_Transmit(&huart2,(uint8_t*)send,16,1000);	//发送数据给显示器
//						 while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);	//等待发送结束
//						 delay_ms(10);
						 #endif
						if(vehicle_type==1)  //双轴单轮
						 {		
								CAN_Send((uint8_t*)g_rxBuffer);
									HAL_UART_Transmit(&huart2,(uint8_t*)g_rxBuffer,16,1000);	//发送数据给显示器
									while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);	//等待发送结束
					   }
						 if(vehicle_type==4)  //双轴单轮
						 {
							 for(i=0;i<2;i++)
							 {
									if(((g_rxBuffer[12]>>i)&0x01)==1)
									{
										ID[(g_rxBuffer[12]/16-1)*2+i]=g_rxBuffer[2*i]<<8|g_rxBuffer[2*i+1];
									}
							 }
					   }
						 if(vehicle_type==6)  //双轴单轮
						 {
							 for(i=0;i<2;i++)
							 {
									if(((g_rxBuffer[12]>>i)&0x01)==1)
									{
										ID[(g_rxBuffer[12]/16-1)*2+i]=g_rxBuffer[2*i]<<8|g_rxBuffer[2*i+1];
									}
							 }
						 }
						 if(vehicle_type==8)  //双轴单轮
						 {
							 for(i=0;i<4;i++)
							 {
									if(((g_rxBuffer[12]>>i)&0x01)==1)
									{
										ID[(g_rxBuffer[12]/16-1)*4+i]=g_rxBuffer[2*i]<<8|g_rxBuffer[2*i+1];
									}
							 }
						 }							 
						 if(vehicle_type==12)  //双轴单轮
						 {
							 for(i=0;i<4;i++)
							 {
									if(((g_rxBuffer[12]>>i)&0x01)==1)
									{
										ID[(g_rxBuffer[12]/16-1)*4+i]=g_rxBuffer[2*i]<<8|g_rxBuffer[2*i+1];
									}
							 }				 
						 }		
             STMFLASH_Write(FLASH_ADDR_SET+ADDR_ID_OFFSET, ID, SIZE_U8*vehicle_type);							 
					}
					else if((g_rxBuffer[14]&0x0f)==0x01)  //挂车更新ID值和温度值
					{
							CAN_Send((uint8_t*)g_rxBuffer);
							HAL_UART_Transmit(&huart2,(uint8_t*)g_rxBuffer,16,1000);	//发送数据给显示器
							while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);	//等待发送结束
					}	
					else if((g_rxBuffer[14]&0xf0)==0x10)
					{
							CAN_Send((uint8_t*)g_rxBuffer);
							HAL_UART_Transmit(&huart2,(uint8_t*)g_rxBuffer,16,1000);	//发送数据给显示器
							while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);	//等待发送结束
					}					
			}
			SP_receive_end_flag=0;	
		}
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void OnMaster(void)
{
    char str[32];
    
    switch(RF_Process())
    {
    case RF_IDLE:
    {
       //RF_StartTx(g_txBuffer, RF_PACKET_SIZE, 1000);
			 
        break;
    }
    
    case RF_TX_DONE:
    {

       // RF_Init();
        //RF_StartRx(g_rxBuffer, RF_PACKET_SIZE, 1000);
        break;
    }
    
    case RF_RX_DONE:
    {

        break;
    }
    
    case RF_RX_TIMEOUT:
    {

        
        break;
    }
    
    case RF_ERROR:
    {

        
        break;
    }
    
    default:
        break;
    }
}

/* Manages the slave operation */
void OnSlave(void)
{
    char str[32];
    
    switch(RF_Process())
    {
    case RF_IDLE:
    {
        RF_StartRx(g_rxBuffer, RF_PACKET_SIZE, INFINITE);
        break;
    }
    
    case RF_RX_DONE:
    {


       // Cmt2300_DelayMs(10);
        //SP_receive_end_flag=1;
        //RF_StartTx(g_txBuffer, RF_PACKET_SIZE, 1000);
        
        break;
    }
    
    case RF_TX_DONE:
    {

        
        break;
    }
    
    case RF_ERROR:
    {

        
        break;
    }
    
    default:
        break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
