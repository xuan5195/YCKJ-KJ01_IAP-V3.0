/*******************************************************************************
** 文件名: 		mian.c
** 版本：  		1.0
** 工作环境: 	RealView MDK-ARM 4.14
** 作者: 		wuguoyana
** 生成日期: 	2011-04-28
** 功能:		USART初始化和RCC设置，然后从common.c中执行主菜单
** 相关文件:	stm32f10x.h
** 修改日志：	2011-04-29   创建文档
*******************************************************************************/
/* 包含头文件 *****************************************************************/
#include "common.h"
#include "bsp.h"


/* 变量 ----------------------------------------------------------------------*/
uint8_t g_RxMessage[8]={0};	//CAN接收数据
uint8_t g_RxMessFlag=0;		//CAN接收数据 标志
extern uint8_t Physical_ADD[4];//物理地址
extern uint8_t FM1702_Key[7];
extern uint8_t WaterCost,CostNum;	//WaterCost=水费 最小扣款金额  //脉冲数
extern uint8_t g_IAP_Flag;	//在线升级标志


extern pFunction Jump_To_Application;
extern uint32_t JumpAddress;

/* 函数声明 ------------------------------------------------------------------*/
void Delay(__IO uint32_t nCount);
static void IAP_Init(void);
void USART_Configuration(void);
/* 函数功能 ------------------------------------------------------------------*/

/*******************************************************************************
  * @函数名称	main
  * @函数说明   主函数
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
int main(void)
{
	uint8_t ProgramCount=0;
    FLASH_Unlock();//Flash 解锁
	BspTm1639_Config();
	BspTm1639_Show(0x03,0x00);
    IAP_Init();
	SerialPutString("\r\n\r\nYCKJ-KJ01_IAP V0.1...\r\nStarting Up...");
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,25,CAN_Mode_Normal);//CAN初始化正常模式,波特率90Kbps    
	Read_Flash_Dat();	//读取Flash数据
	printf("g_IAP_Flag:0x%02X;\r\n",g_IAP_Flag);
//	g_IAP_Flag = 0xAA;
    while (1)
    {
		if (g_IAP_Flag  == 0xAA)
		{//执行IAP驱动程序更新Flash程序
			SerialPutString("\r\n======================================================================");
			SerialPutString("\r\n=     In-Application Programming Application  (Version 1.2.4)        =");
			SerialPutString("\r\n======================================================================");
			SerialPutString("\r\n\r\n");
			Main_Menu ();
		}		
		else//否则执行用户程序
		{
			//判断用户是否已经下载程序，因为正常情况下此地址是栈地址。
			//若没有这一句的话，即使没有下载程序也会进入而导致跑飞。
			if (((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
			{
				SerialPutString("Execute user Program\r\n\n");
				//跳转至用户代码
				JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
				Jump_To_Application = (pFunction) JumpAddress;

				//初始化用户程序的堆栈指针
				__set_MSP(*(__IO uint32_t*) ApplicationAddress);
				Jump_To_Application();
			}
			else
			{
				SerialPutString("no user Program\r\n\n");
				Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);
				Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);
				Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);
				if(ProgramCount>200)	g_IAP_Flag = 0xAA;	//转入IAP程序接收模式
				else 					ProgramCount++;
			}
		}
    }
}


/*******************************************************************************
  * @函数名称	GPIO_Configuration
  * @函数说明   配置使用USART1的相关IO管脚
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void GPIO_Configuration(void)
{
}

/*******************************************************************************
  * @函数名称	IAP_Init
  * @函数说明   配置使用IAP
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void IAP_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* USART1 配置 ------------------------------------------------------------
         USART1 配置如下:
          - 波特率      = 115200 baud
          - 字长        = 8 Bits
          - 一个停止位
          - 无校验
          - 无硬件流控制
          - 接受和发送使能
    --------------------------------------------------------------------------*/
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    // 配置 USART1 Tx (PA.09) 作为功能引脚并上拉输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //配置 USART1 Tx (PA.10) 作为功能引脚并是浮空输入模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    USART_Init(USART1, &USART_InitStructure);
    // 使能 USART1
    USART_Cmd(USART1, ENABLE);
}

/*******************************************************************************
  * @函数名称	Delay
  * @函数说明   插入一段延时时间
  * @输入参数   nCount: 指定延时时间长度
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void Delay(__IO uint32_t nCount)
{
    for (; nCount != 0; nCount--);
}

#ifdef  USE_FULL_ASSERT

/*******************************************************************************
  * @函数名称	assert_failed
  * @函数说明   报告在检查参数发生错误时的源文件名和错误行数
  * @输入参数   file: 源文件名
  				line: 错误所在行数
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* 用户可以增加自己的代码用于报告错误的文件名和所在行数,
       例如：printf("错误参数值: 文件名 %s 在 %d行\r\n", file, line) */

    //死循环
    while (1)
    {
    }
}
#endif

/*******************************文件结束***************************************/
