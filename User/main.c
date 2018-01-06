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
#include "bsp_tm1639.h"		
#include "bsp_can.h"		


/* 变量 ----------------------------------------------------------------------*/
extern pFunction Jump_To_Application;
extern uint32_t JumpAddress;
extern uint8_t g_RxMessage[8];	//CAN接收数据
extern uint8_t g_RxMessFlag;	//CAN接收数据 标志

/* 函数声明 ------------------------------------------------------------------*/
void Delay(__IO uint32_t nCount);


void RCC_Configuration(void)
{
//    RCC_DeInit();
//    RCC_HSEConfig(RCC_HSE_OFF);
//    RCC_HSICmd(ENABLE);
    RCC_HSEConfig(RCC_HSE_ON);
}
/*******************************************************************************
  * @函数名称	main
  * @函数说明   主函数
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
int main(void)
{   
	RCC_Configuration();
    FLASH_Unlock();		//Flash 解锁
	BspTm1639_Config();	//TM1639初始化
	BspTm1639_Show(0x02,0x00);
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_7tq,5,CAN_Mode_Normal);//CAN初始化正常模式,波特率450Kbps    
	printf("Starting Up...\r\n");

    while (1)
    {
		if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)  == 0x00)
		{
			//假如按键按下
			//执行IAP驱动程序更新Flash程序

			SerialPutString("\r\n======================================================================");
			SerialPutString("\r\n=              (C) COPYRIGHT 2011 Lierda                             =");
			SerialPutString("\r\n=                                                                    =");
			SerialPutString("\r\n=     In-Application Programming Application  (Version 1.0.0)        =");
			SerialPutString("\r\n=                                                                    =");
			SerialPutString("\r\n=                                   By wuguoyan                      =");
			SerialPutString("\r\n======================================================================");
			SerialPutString("\r\n\r\n");
			Main_Menu ();
		}
		//否则执行用户程序
		else
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
				Delay(0xFFFF);
			}
		}
    }
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
