#include "stm32f10x.h"
#include <stdio.h>
#include "common.h"

#include "bsp_can.h"
#include "bsp_crc8.h"
#include "bsp_tm1639.h"	
#include "bsp.h"
#include "bsp_crc16.h"

CanRxMsg CAN1_RxMessage;
//CBL_CMD_LIST CMD_List;
volatile uint8_t CAN1_CanRxMsgFlag=0;//接收到CAN数据后的标志
extern uint8_t g_RxMessage[8];	//CAN接收数据
extern uint8_t g_RxMessFlag;	//CAN接收数据 标志

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:1~3; CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//tbs2:时间段2的时间单元.范围:1~8;
//tbs1:时间段1的时间单元.范围:1~16;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//注意以上参数任何一个都不能设为0,否则会乱.
//波特率=Fpclk1/((tsjw+tbs1+tbs2)*brp);
//mode:0,普通模式;1,回环模式;
//Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN_Normal_Init(1,8,7,5,1);
//则波特率为:36M/((1+8+7)*5)=450Kbps
//返回值:0,初始化OK;	其他,初始化失败;


u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	u16 std_id = 0x500,std_id1 = std_id1;
//	u32 ext_id = 0x1800F001;
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
 	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
    GPIO_Init(GPIOA, &GPIO_InitStructure);		//初始化IO
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化IO
	  
 	//CAN单元设置
 	CAN_InitStructure.CAN_TTCM=DISABLE;	//禁止时间触发通信模式
 	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件对CAN_MCR寄存器的INRQ位进行置1随后清0后，一旦硬件检测到128次11位连续的隐性位，就退出离线状态。
  	CAN_InitStructure.CAN_AWUM=DISABLE;	//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)//
  	CAN_InitStructure.CAN_NART=ENABLE;	//ENABLE:CAN报文只被发送1次，不管发送的结果如何（成功、出错或仲裁丢失）
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//在接收溢出时FIFO未被锁定，当接收FIFO的报文未被读出，下一个收到的报文会覆盖原有的报文
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//发送FIFO优先级由报文的标识符来决定
  	CAN_InitStructure.CAN_Mode= mode;	//模式设置： mode:0,普通模式;1,回环模式; //
  	//设置波特率
  	CAN_InitStructure.CAN_SJW=tsjw;				//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;        //分频系数(Fdiv)为brp+1	//
  	CAN_Init(CAN1, &CAN_InitStructure);         // 初始化CAN1 

 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //设置过滤器组0，范围为0~13  
 	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //CAN_FilterMode_IdMask:屏蔽模式  CAN_FilterMode_IdList:列表模式
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //设置过滤器位宽为32位  
		CAN_FilterInitStructure.CAN_FilterIdHigh=std_id<<5;  		//设置标识符寄存器高字节  
		CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;		//设置标识符寄存器低字节	
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xA000; 		//设置屏蔽寄存器高字节  只关心std_id这三位为（101）
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0|CAN_ID_STD;	//设置屏蔽寄存器低字节  
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
 	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0

  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
	
#if CAN_RX0_INT_ENABLE	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif

	return 0;
}   
 
#if CAN_RX0_INT_ENABLE	//使能RX0中断
	    
void USB_LP_CAN1_RX0_IRQHandler(void)	//中断服务函数		
{
	CAN_Receive(CAN1,CAN_FIFO0, &CAN1_RxMessage);
	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
	CAN1_CanRxMsgFlag = 1;
}
#endif

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为16)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;	其他,失败;
u8 Can_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	if((msg[0]==0xB3)||(msg[0]==0xC2))	
		TxMessage.StdId=0x200;			// 标准标识符为0
	else				
		TxMessage.StdId=0x200|msg[1];	// 标准标识符为0
	TxMessage.ExtId=0x1800F001;			// 设置扩展标示符（29位）
	TxMessage.IDE=0;		// 不使用扩展标识符
	TxMessage.RTR=0;		// 消息类型：CAN_RTR_Data为数据帧;CAN_RTR_Remote为远程帧
	TxMessage.DLC=len;		// 发送两帧信息
	for(i=0;i<8;i++)
	TxMessage.Data[i]=msg[i];	// 帧信息          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	return 0;		

}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;	 其他,接收的数据长度;
u8 Can_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);				//读取数据	
    for(i=0;i<8;i++)    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

void Package_Send(u8 _mode,u8 *Package_Dat)
{
	uint8_t Package_SendBuf[8]={0x00};//发送缓冲区
	if (_mode==0x00)
    {
		Package_SendBuf[0] = 0xB3;
		Package_SendBuf[1] = Package_Dat[1];	//物理地址
		Package_SendBuf[2] = Package_Dat[2];
		Package_SendBuf[3] = Package_Dat[3];
		Package_SendBuf[4] = Package_Dat[4];
		Package_SendBuf[5] = Package_Dat[5];
		Package_SendBuf[6] = Package_Dat[6];
		Package_SendBuf[7] = Package_Dat[7];
		Can_Send_Msg((u8 *)Package_SendBuf,8);//发送8个字节
    }
}
// 发送一帧CAN数据
// CANx CAN通道号
//TxMessage CAN消息指针

uint8_t CAN_WriteData(CanTxMsg *TxMessage)
{
	uint8_t TransmitMailbox;   
	uint32_t	TimeOut=0;
	TransmitMailbox = CAN_Transmit(CAN1,TxMessage);
	while(CAN_TransmitStatus(CAN1,TransmitMailbox)!=CAN_TxStatus_Ok)
	{
		TimeOut++;
		if(TimeOut > 10000000)
		{
			return 1;
		}
	}
	return 0;
}
//执行主机下发的命令
//pRxMessage CAN总线消息
void CAN_BOOT_ExecutiveCommand(CanRxMsg *pRxMessage)
{
	CanTxMsg TxMessage;
	uint8_t ret,i;
	uint8_t can_cmd = (pRxMessage->StdId)&0x0F;		//ID的bit0~bit3位为命令码 0x7FF 低四位为命令码
//	uint16_t can_addr = (pRxMessage->StdId >> 4);	//ID的bit4~bit7位为数据包编号，每1K数据包为一个编号，每1K数据包后二位为为CRC16 共1028byte 
//	uint32_t BaudRate;
//	static uint32_t jump_addr;
	uint16_t crc_data;
	static uint32_t start_addr;
	static uint32_t data_size=0;
	static uint32_t data_index=0;
	__align(4) static uint8_t	data_temp[1028];	//四字体 对齐
	//判断接收的数据地址是否和本节点地址匹配，若不匹配则直接返回，不做任何事情
//	if((can_addr!=CAN_BOOT_GetAddrData())&&(can_addr!=0))	return;
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;	//不使用扩展标识符
	TxMessage.RTR = CAN_RTR_Data;		//消息类型：CAN_RTR_Data为数据帧;CAN_RTR_Remote为远程帧
	TxMessage.DLC = 0;					//数据长度为0；
	switch (can_cmd)
	{
		//CMD_List.EraseFlash，擦除Flash中的数据，起始地址存储在Data[0]到Data[3]中，结束地址存储在Data[4]到Data[7]中
		case CMD_List_EraseFlash:
			SerialPutString("CMD_List_EraseFlash\r\n");
			__set_PRIMASK(1);
			FLASH_Unlock();
			ret = CAN_BOOT_ErasePage((pRxMessage->Data[0]<<24)|(pRxMessage->Data[1]<<16)|(pRxMessage->Data[2]<<8)|(pRxMessage->Data[3]<<0),
									 (pRxMessage->Data[4]<<24)|(pRxMessage->Data[5]<<16)|(pRxMessage->Data[6]<<8)|(pRxMessage->Data[7]<<0));
			FLASH_Lock();	
			__set_PRIMASK(0);
//			if(can_addr != 0x00)
			{
				if(ret==FLASH_COMPLETE)
					TxMessage.StdId = CAN_BOOT_GetAddrData()|CMD_List_EraseFlash;
				else
					TxMessage.StdId = CAN_BOOT_GetAddrData()|CMD_List_CmdFaild;
				TxMessage.DLC = 0;
				CAN_WriteData(&TxMessage);
			}
			break;
		//CMD_List.BlockWriteInfo，设置写Flash数据的相关信息，比如数据起始地址，数据大小
		//数据起始地址存储在Data[0]到Data[3]中，数据大小存储在Data[4]到Data[7]中，该函数必须在Bootloader程序中实现，APP程序可以不用实现
		case CMD_List_BlockWriteInfo:
			SerialPutString("CMD_List_BlockWriteInfo\r\n");
			start_addr = (pRxMessage->Data[0]<<24)|(pRxMessage->Data[1]<<16)|(pRxMessage->Data[2]<<8)|(pRxMessage->Data[3]<<0);
			data_size = (pRxMessage->Data[4]<<24)|(pRxMessage->Data[5]<<16)|(pRxMessage->Data[6]<<8)|(pRxMessage->Data[7]<<0);
			data_index = 0;
//			if(can_addr != 0x00)
			{
				TxMessage.StdId = CAN_BOOT_GetAddrData()|CMD_List_BlockWriteInfo;	
				TxMessage.DLC = 0;
				CAN_WriteData(&TxMessage);
			}
			break;
		//CMD_List.WriteBlockFlash，先将数据存储在本地缓冲区中，然后计算数据的CRC，若校验正确则写数据到Flash中
		//每次执行该数据，数据缓冲区的数据字节数会增加pRxMessage->DLC字节，当数据量达到data_size（包含2字节CRC校验码）字节后
		//对数据进行CRC校验，若数据校验无误，则将数据写入Flash中
		//该函数在Bootloader程序中必须实现，APP程序可以不用实现
		case CMD_List_WriteBlockFlash:
			SerialPutString("CMD_List_WriteBlockFlash\r\n");
			if((data_index<data_size)&&(data_index<1026))
			{
				__set_PRIMASK(1);
				for(i=0;i<pRxMessage->DLC;i++)
				{
					data_temp[data_index++] = pRxMessage->Data[i];
				}
				__set_PRIMASK(0);
			}
			if((data_index>=data_size)||(data_index>=1026))
			{
				crc_data = crc16_ccitt(data_temp,data_size-2);
				if(crc_data==((data_temp[data_size-2]<<8)|(data_temp[data_size-1])))
				{
					__set_PRIMASK(1);
					FLASH_Unlock();
					ret = CAN_BOOT_ProgramDatatoFlash(start_addr,data_temp,data_index-2);
					FLASH_Lock();	
					__set_PRIMASK(0);
//					if(can_addr != 0x00)
					{
						if(ret==FLASH_COMPLETE)
						{
							TxMessage.StdId = CAN_BOOT_GetAddrData()|CMD_List_WriteBlockFlash;	
						}
						else
						{
							TxMessage.StdId = CAN_BOOT_GetAddrData()|CMD_List_CmdFaild;
						}
						TxMessage.DLC = 0;
						CAN_WriteData(&TxMessage);
					}
				}
				else
				{
					TxMessage.StdId = CAN_BOOT_GetAddrData()|CMD_List_CmdFaild;
					TxMessage.DLC = 0;
					CAN_WriteData(&TxMessage);
				}
			}
			break;
		
		case CMD_List_ExcuteApp:  //CMD_List.ExcuteApp，控制程序跳转到指定地址执行
			SerialPutString("CMD_List_ExcuteApp\r\n");
			CAN_JumpToApplication();
			break;
		default:
			break;
	}
}

//擦出指定扇区区间的Flash数据 。
//StartPage 起始扇区地址
//EndPage 结束扇区地址
//扇区擦出状态  
FLASH_Status CAN_BOOT_ErasePage(uint32_t StartPageAddr,uint32_t EndPageAddr)
{
	uint32_t i;
	FLASH_Status FLASHStatus=FLASH_COMPLETE;
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
	
	for(i=StartPageAddr;i<=EndPageAddr;i+=PAGE_SIZE)
	{
		FLASHStatus = FLASH_ErasePage(i);
		if(FLASHStatus!=FLASH_COMPLETE)
		{
			FLASH_Lock();
			return	FLASHStatus;	
		}
	}
	FLASH_Lock();
	return FLASHStatus;
}

//获取节点地址信息
//节点地址
uint16_t CAN_BOOT_GetAddrData(void)
{
	return 0x500;
}

/**
  * @brief  将数据烧写到指定地址的Flash中 。
  * @param  Address Flash起始地址。
  * @param  Data 数据存储区起始地址。
  * @param  DataNum 数据字节数。
  * @retval 数据烧写状态。
  */
FLASH_Status CAN_BOOT_ProgramDatatoFlash(uint32_t StartAddress,uint8_t *pData,uint32_t DataNum) 
{
	FLASH_Status FLASHStatus = FLASH_COMPLETE;

	uint32_t i;

	if(StartAddress<EXE_APP_FLAG)
	{
		return FLASH_ERROR_PG;
	}
	/* Clear All pending flags */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

	for(i=0;i<(DataNum>>2);i++)
	{
		FLASHStatus = FLASH_ProgramWord(StartAddress, *((uint32_t*)pData));
		if (FLASHStatus == FLASH_COMPLETE)
		{
			StartAddress += 4;
			pData += 4;
		}
		else
		{ 
			return FLASHStatus;
		}
	}
	return	FLASHStatus;
}



