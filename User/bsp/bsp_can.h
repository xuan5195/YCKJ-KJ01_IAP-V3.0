#ifndef __BSP_CAN_H
#define __BSP_CAN_H	 

#include "stm32f10x.h"

//CAN接收RX0中断使能
#define CAN_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    

typedef enum
{
	CMD_List_EraseFlash=1,		//擦出APP储存扇区数据
	CMD_List_WriteBlockFlash,	//以多字节形式写数据
	CMD_List_ReadBlockFlash,	//以多字节形式读数据
	CMD_List_BlockWriteInfo,	//设置多字节写数据相关参数（写起始地址，数据量）
	CMD_List_BlockReadInfo,		//设置多字节数据读相关参数（读起始地址，数据量）
	CMD_List_OnlineCheck,		//检测节点是否在线
	CMD_List_CmdSuccess,		//命令执行成功
	CMD_List_CmdFaild,			//命令执行失败
	CMD_List_SetBaudRate,		//设置节点波特率
	CMD_List_ExcuteApp,			//执行应用程序
	CMD_List_Undefine0,			//未定义
	CMD_List_Undefine1,			//未定义
	CMD_List_Undefine2,			//未定义
	CMD_List_Undefine3,			//未定义
}CMD_List;

u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
u8 Can_Send_Msg(u8* msg,u8 len);						//发送数据
u8 Can_Receive_Msg(u8 *buf);							//接收数据
void Package_Send(u8 _mode,u8 *Package_Dat);
void CAN_BOOT_ExecutiveCommand(CanRxMsg *pRxMessage);
FLASH_Status CAN_BOOT_ErasePage(uint32_t StartPageAddr,uint32_t EndPageAddr);
FLASH_Status CAN_BOOT_ProgramDatatoFlash(uint32_t StartAddress,uint8_t *pData,uint32_t DataNum); 
uint16_t CAN_BOOT_GetAddrData(void);
void CAN_IAPCommand(CanRxMsg *pRxMessage);


#endif

















