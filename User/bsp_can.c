#include "stm32f10x.h"
#include <stdio.h>

#include "bsp_can.h"
#include "bsp_crc8.h"
#include "bsp_tm1639.h"	

//extern uint8_t Physical_ADD[4];//物理地址
//extern uint8_t Logic_ADD;
uint8_t g_RxMessage[8];	//CAN接收数据
uint8_t g_RxMessFlag;	//CAN接收数据 标志
//extern unsigned char FM1702_Key[7];
//extern uint8_t WaterCost,CostNum;	//WaterCost=水费 最小扣款金额  //脉冲数
//extern uint8_t g_MemoryBuf[5][10];	//数据缓存，[0]=0xAA表示有插卡数据，[0]=0xBB表示有拔卡数据，[1-4]卡号；[5-7]金额；[8]卡核验码；[9]通信码
//extern void PutOutMemoryBuf(void);	//清第一个缓存
//extern uint8_t re_RxMessage[16];
//extern unsigned char UID[5];
//extern uint8_t gErrorDat[6];	//异常代码存储
//extern uint8_t Flash_UpdateFlag;	//Flash有数据更新标志，0xAA表示有数据要更新

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
	u16 std_id = 0x200,std_id1 = 0x201;
	u32 ext_id = 0x1800F001;
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
 	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;  //CAN_FilterMode_IdMask:屏蔽模式  CAN_FilterMode_IdList:列表模式
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //设置过滤器位宽为32位  
	if(std_id1!=std_id)
	{
		CAN_FilterInitStructure.CAN_FilterIdHigh=std_id<<5;  		//设置标识符寄存器高字节  
		CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;		//设置标识符寄存器低字节	
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=std_id1<<5; 	//设置屏蔽寄存器高字节  
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0|CAN_ID_STD;	//设置屏蔽寄存器低字节  
	}
	else
	{
		CAN_FilterInitStructure.CAN_FilterIdHigh=std_id<<5;  	//设置标识符寄存器高字节  
		CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;	//设置标识符寄存器低字节	
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((ext_id<<3)>>16) & 0xffff; 		//设置屏蔽寄存器高字节  
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=((ext_id<<3)& 0xffff) | CAN_ID_EXT;	//设置屏蔽寄存器低字节  
	}
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
//中断服务函数			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
//	uint8_t SendCAN_Buf[8]={0};
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
	if(g_RxMessFlag != 0xAA)
	{
		for(i=0;i<8;i++)	
		{
			//printf("%02X",RxMessage.Data[i]);
			g_RxMessage[i] = RxMessage.Data[i];
		}
		g_RxMessFlag = 0xAA;
	}
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
	//TxMessage.StdId=0x200;			// 标准标识符为0
	TxMessage.ExtId=0x1800F001;			// 设置扩展标示符（29位）
	TxMessage.IDE=0;			// 不使用扩展标识符
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
	uint8_t res;
	uint8_t Package_SendBuf[8]={0x00};//发送缓冲区
	switch (_mode)
    {
    	case 0x04:	//轮循回复 插卡数据2
 			Package_SendBuf[0] = 0x04;
			Package_SendBuf[1] = 0x01;	//逻辑地址
			Package_SendBuf[2] = Package_Dat[0];	//金额1
 			Package_SendBuf[3] = Package_Dat[1];	//金额2
			Package_SendBuf[4] = Package_Dat[2];	//金额3
			Package_SendBuf[5] = Package_Dat[3];	//卡校验
			Package_SendBuf[6] = Package_Dat[4];	//通信码
			break;
    	default:
    		break;
    }
	res = Can_Send_Msg(Package_SendBuf,8);//发送8个字节
	if(res)	{printf(" F, ");}	
	else 	{printf(" S, ");}	
}



