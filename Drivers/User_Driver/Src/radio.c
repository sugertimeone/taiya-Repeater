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
 * @file    radio.c
 * @brief   Generic radio handlers
 *
 * @version 1.1
 * @date    Feb 08 2017
 * @author  CMOSTEK R@D
 */
 
#include "radio.h"
#include "cmt2300_params.h"
#include "cmt_spi.h" 

#include <string.h>
static EnumRFStatus g_nNextRFState = RF_STATE_IDLE;
static u8* g_pRxBuffer = NULL;
static u8* g_pTxBuffer = NULL;
static u16 g_nRxLength = 0;
static u16 g_nTxLength = 0;

static u32 g_nRxTimeout = INFINITE;
static u32 g_nTxTimeout = INFINITE;
static u32 g_nRxTimeCount = 0;
static u32 g_nTxTimeCount = 0;

static u8 g_nInterrutFlags = 0;

u8 a,b,c,d=0;
u8 SEND_END_FLAG=0;
//void RF_Init(void)
//{
//    Cmt2300_InitGpio();
//	  Cmt2300_Init();
//    
//    /* Config registers */
//    Cmt2300_ConfigRegBank(CMT2300_CMT_BANK_ADDR       , g_cmt2300CmtBank       , CMT2300_CMT_BANK_SIZE       );
//    Cmt2300_ConfigRegBank(CMT2300_SYSTEM_BANK_ADDR    , g_cmt2300SystemBank    , CMT2300_SYSTEM_BANK_SIZE    );
//    Cmt2300_ConfigRegBank(CMT2300_FREQUENCY_BANK_ADDR , g_cmt2300FrequencyBank , CMT2300_FREQUENCY_BANK_SIZE );
//    Cmt2300_ConfigRegBank(CMT2300_DATA_RATE_BANK_ADDR , g_cmt2300DataRateBank  , CMT2300_DATA_RATE_BANK_SIZE );
//    Cmt2300_ConfigRegBank(CMT2300_BASEBAND_BANK_ADDR  , g_cmt2300BasebandBank  , CMT2300_BASEBAND_BANK_SIZE  );
//    Cmt2300_ConfigRegBank(CMT2300_TX_BANK_ADDR        , g_cmt2300TxBank        , CMT2300_TX_BANK_SIZE        );

//	RF_Config();
//}
void RF_Init()
{
	CMT_SPI_Init();
	Cmt2300_Init();//CMT2300A��ʼ��
	  //���üĴ���������RFPDK�������
    Cmt2300_ConfigRegBank(CMT2300_CMT_BANK_ADDR       , g_cmt2300CmtBank       , CMT2300_CMT_BANK_SIZE       );
    Cmt2300_ConfigRegBank(CMT2300_SYSTEM_BANK_ADDR    , g_cmt2300SystemBank    , CMT2300_SYSTEM_BANK_SIZE    );
    Cmt2300_ConfigRegBank(CMT2300_FREQUENCY_BANK_ADDR , g_cmt2300FrequencyBank , CMT2300_FREQUENCY_BANK_SIZE );
    Cmt2300_ConfigRegBank(CMT2300_DATA_RATE_BANK_ADDR , g_cmt2300DataRateBank  , CMT2300_DATA_RATE_BANK_SIZE );
    Cmt2300_ConfigRegBank(CMT2300_BASEBAND_BANK_ADDR  , g_cmt2300BasebandBank  , CMT2300_BASEBAND_BANK_SIZE  );
    Cmt2300_ConfigRegBank(CMT2300_TX_BANK_ADDR        , g_cmt2300TxBank        , CMT2300_TX_BANK_SIZE        );
	  RF_Config();
}
//void RF_R_Init()
//{
//	  Cmt2300_GoStby();
//	  Cmt2300_ClearInterruptFlags();    //����жϱ�־
//	  //���üĴ���������RFPDK�������
//    Cmt2300_ConfigRegBank(CMT2300_CMT_BANK_ADDR       , g_cmt2300CmtBank       , CMT2300_CMT_BANK_SIZE       );
//    Cmt2300_ConfigRegBank(CMT2300_SYSTEM_BANK_ADDR    , g_cmt2300SystemBank    , CMT2300_SYSTEM_BANK_SIZE    );
//    Cmt2300_ConfigRegBank(CMT2300_FREQUENCY_BANK_ADDR , g_cmt2300FrequencyBank , CMT2300_FREQUENCY_BANK_SIZE );
//    Cmt2300_ConfigRegBank(CMT2300_DATA_RATE_BANK_ADDR , g_cmt2300DataRateBank  , CMT2300_DATA_RATE_BANK_SIZE );
//    Cmt2300_ConfigRegBank(CMT2300_BASEBAND_BANK_ADDR  , g_cmt2300BasebandBank  , CMT2300_BASEBAND_BANK_SIZE  );
//    Cmt2300_ConfigRegBank(CMT2300_TX_BANK_ADDR        , g_cmt2300TxBank        , CMT2300_TX_BANK_SIZE        );
//	  Cmt2300_GoSleep();//������Ч
//}
//void RF_S_Init()
//{
//	 //Cmt2300_Init();//CMT2300A��ʼ��
//	  Cmt2300_GoStby();
//	  Cmt2300_ClearInterruptFlags();    //����жϱ�־
//	  //���üĴ���������RFPDK�������
//    Cmt2300_ConfigRegBank(CMT2300_CMT_BANK_ADDR       , g_cmt2300CmtBank       , CMT2300_CMT_BANK_SIZE       );
//    Cmt2300_ConfigRegBank(CMT2300_SYSTEM_BANK_ADDR    , g_cmt2300SystemBank    , CMT2300_SYSTEM_BANK_SIZE    );
//    Cmt2300_ConfigRegBank(CMT2300_FREQUENCY_BANK_ADDR , g_cmt2300FrequencyBank , CMT2300_FREQUENCY_BANK_SIZE );
//    Cmt2300_ConfigRegBank(CMT2300_DATA_RATE_BANK_ADDR , g_cmt2300DataRateBank1  , CMT2300_DATA_RATE_BANK_SIZE );
//    Cmt2300_ConfigRegBank(CMT2300_BASEBAND_BANK_ADDR  , g_cmt2300BasebandBank1  , CMT2300_BASEBAND_BANK_SIZE  );
//    Cmt2300_ConfigRegBank(CMT2300_TX_BANK_ADDR        , g_cmt2300TxBank        , CMT2300_TX_BANK_SIZE        );
//	  Cmt2300_GoSleep();//������Ч
//}
void RF_Config(void)
{
#ifdef ENABLE_ANTENNA_SWITCH
    //��������߿��أ�Ĭ��GPIO1��GPIO2λ���״̬���û������ڶ�������
    Cmt2300_EnableAntennaSwitch(0);
    
#else
    /* Config GPIOs */
    Cmt2300_ConfigGpio(
        CMT2300_GPIO1_SEL_INT1 | /* INT1 > GPIO1 */
        CMT2300_GPIO2_SEL_INT2 | /* INT2 > GPIO2 */
        CMT2300_GPIO3_SEL_DOUT
        );
    
    /* Config interrupt */
    Cmt2300_ConfigInterrupt(
        CMT2300_INT_SEL_TX_DONE, /* Config INT1 */
        CMT2300_INT_SEL_PKT_OK   /* Config INT2 */
        );
#endif

    /* Enable interrupt */
        Cmt2300_EnableInterrupt(
        CMT2300_MASK_TX_DONE_EN  |    //��������ж�ʹ��
        CMT2300_MASK_PREAM_OK_EN |    //Preamble���ɹ��ж�ʹ��
        CMT2300_MASK_SYNC_OK_EN  |    //ͬ���ּ��ɹ��ж�ʹ��
        CMT2300_MASK_NODE_OK_EN  |    //Node ID���ɹ��ж�ʹ��
        CMT2300_MASK_CRC_OK_EN   |    //CRC���ɹ��ж�ʹ��
        CMT2300_MASK_PKT_DONE_EN     //Packet��������ж�ʹ��
        );
        
    Cmt2300_EnableLfosc(FALSE);//If you need use sleep timer, you should enable LFOSC.
    
    /* Use a single 64-byte FIFO for either Tx or Rx */
    //Cmt2300_EnableFifoMerge(TRUE);
    
    //Cmt2300_SetFifoThreshold(16);
    
    /* Go to sleep for configuration to take effect */
    Cmt2300_GoSleep();//������Ч
}

void RF_SetStatus(EnumRFStatus nStatus)
{
    g_nNextRFState = nStatus;
}

EnumRFStatus RF_GetStatus(void)
{
    return g_nNextRFState;
}

u8 RF_GetInterruptFlags(void)
{
    return g_nInterrutFlags;
}

void RF_StartRx(u8 buf[], u16 len, u32 timeout)
{
    g_pRxBuffer = buf;
    g_nRxLength = len;
    g_nRxTimeout = timeout;
    
    //memset(g_pRxBuffer, 0, g_nRxLength);
    
    g_nNextRFState = RF_STATE_RX_START;
}

void RF_StartTx(u8 buf[], u16 len, u32 timeout)
{
    g_pTxBuffer = buf;
    g_nTxLength = len;
    g_nTxTimeout = timeout;
    
    g_nNextRFState = RF_STATE_TX_START;
}
//״̬�ж�
EnumRFResult RF_Process(void)
{
    EnumRFResult nRes = RF_BUSY;
    
    switch(g_nNextRFState) 
    {
    case RF_STATE_IDLE:    //����״̬
    {
        nRes = RF_IDLE;
        break;
    }
    
    case RF_STATE_RX_START:       ////���տ�ʼ״̬
    {
        Cmt2300_GoStby();
        Cmt2300_ClearInterruptFlags();
        
        /* Must clear FIFO after enable SPI to read or write the FIFO */
        Cmt2300_EnableReadFifo();    //ʹ�ܶ�FIFO
        Cmt2300_ClearFifo();         //���FIFO
       
        if(FALSE==Cmt2300_GoRx())
            g_nNextRFState = RF_STATE_ERROR;   //����
        else
            g_nNextRFState = RF_STATE_RX_WAIT;  //���յȴ�
        
        //g_nRxTimeCount = Cmt2300_GetTickCount();
        
        break;
    }
        
    case RF_STATE_RX_WAIT:     //���յȴ�
    {
#ifdef ENABLE_ANTENNA_SWITCH
			  a=1;
			  c=Cmt2300_ReadReg(CMT2300_CUS_INT_FLAG);
        if(CMT2300_MASK_PKT_OK_FLG & Cmt2300_ReadReg(CMT2300_CUS_INT_FLAG))  /* ��������ɱ�־*/
#else
        //if(Cmt2300_ReadGpio2())  /* Read INT2, PKT_OK */
#endif
        {
					  b=1;c=Cmt2300_ReadReg(CMT2300_CUS_INT_FLAG);
            g_nNextRFState = RF_STATE_RX_DONE;
        }
        
        //if( (INFINITE != g_nRxTimeout) && ((g_nSysTickCount-g_nRxTimeCount) > g_nRxTimeout) )
           // g_nNextRFState = RF_STATE_RX_TIMEOUT;
        
        break;
    }
        
    case RF_STATE_RX_DONE:  //�������
    {
        Cmt2300_GoStby();

        /* The length need be smaller than 32 */
        Cmt2300_ReadFifo(g_pRxBuffer, g_nRxLength);

        g_nInterrutFlags = Cmt2300_ClearInterruptFlags();
            
        Cmt2300_GoSleep();
        
        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_RX_DONE;
			  SP_receive_end_flag=1;//������ɱ�־
        break;
    }
    
    case RF_STATE_RX_TIMEOUT:  //����ʱ�䳬ʱ
    {
        Cmt2300_GoSleep();
        
        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_RX_TIMEOUT;
        break;
    }
    
    case RF_STATE_TX_START:      //���տ�ʼ״̬
    {
        Cmt2300_GoStby();
        Cmt2300_ClearInterruptFlags();  //�����־λ
        
        /* Must clear FIFO after enable SPI to read or write the FIFO */
        Cmt2300_EnableWriteFifo();    // дFIFOʹ��
        Cmt2300_ClearFifo();          //���FIFO
        
        /* The length need be smaller than 32 */
        Cmt2300_WriteFifo(g_pTxBuffer, g_nTxLength);
        
        if( 0==(CMT2300_MASK_TX_FIFO_NMTY_FLG & Cmt2300_ReadReg(CMT2300_CUS_FIFO_FLAG)) )  //δ�����ݳ����ж�
            g_nNextRFState = RF_STATE_ERROR;

        if(FALSE==Cmt2300_GoTx())
            g_nNextRFState = RF_STATE_ERROR;
        else
            g_nNextRFState = RF_STATE_TX_WAIT;
        
       // g_nTxTimeCount = Cmt2300_GetTickCount();
        
        break;
    }
        
    case RF_STATE_TX_WAIT:    //����
    {
#ifdef ENABLE_ANTENNA_SWITCH
        if(CMT2300_MASK_TX_DONE_FLG & Cmt2300_ReadReg(CMT2300_CUS_INT_CLR1))  /* ������ɱ�־ */
#else
        //if(Cmt2300_ReadGpio1())  /* Read INT1, TX_DONE */
#endif
        {
            g_nNextRFState = RF_STATE_TX_DONE;
        }
        
       // if( (INFINITE != g_nTxTimeout) && ((g_nSysTickCount-g_nTxTimeCount) > g_nTxTimeout) )
           // g_nNextRFState = RF_STATE_TX_TIMEOUT;
            
        break;
    }
            
    case RF_STATE_TX_DONE://�������
    {
        Cmt2300_ClearInterruptFlags();
        Cmt2300_GoSleep();
        
        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_TX_DONE;
			   SEND_END_FLAG=1;
			  //RF_R_Init();
        break;
    }
    
    case RF_STATE_TX_TIMEOUT://���ͳ�ʱ
    {
        Cmt2300_GoSleep();
        
        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_TX_TIMEOUT;
        break;
    }
    
    case RF_STATE_ERROR://����״̬����������
    {
        Cmt2300_SoftReset();
        Cmt2300_DelayMs(20);
        
        Cmt2300_GoStby();
        RF_Config();
        
        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_ERROR;
        break;
    }
    
    default:
        break;
    }
    
    return nRes;
}
