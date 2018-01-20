#ifndef __BSP_CAN_H
#define __BSP_CAN_H	 

#include "stm32f10x.h"

//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    

typedef enum
{
	CMD_List_EraseFlash=1,		//����APP������������
	CMD_List_WriteBlockFlash,	//�Զ��ֽ���ʽд����
	CMD_List_ReadBlockFlash,	//�Զ��ֽ���ʽ������
	CMD_List_BlockWriteInfo,	//���ö��ֽ�д������ز�����д��ʼ��ַ����������
	CMD_List_BlockReadInfo,		//���ö��ֽ����ݶ���ز���������ʼ��ַ����������
	CMD_List_OnlineCheck,		//���ڵ��Ƿ�����
	CMD_List_CmdSuccess,		//����ִ�гɹ�
	CMD_List_CmdFaild,			//����ִ��ʧ��
	CMD_List_SetBaudRate,		//���ýڵ㲨����
	CMD_List_ExcuteApp,			//ִ��Ӧ�ó���
	CMD_List_Undefine0,			//δ����
	CMD_List_Undefine1,			//δ����
	CMD_List_Undefine2,			//δ����
	CMD_List_Undefine3,			//δ����
}CMD_List;

u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
u8 Can_Send_Msg(u8* msg,u8 len);						//��������
u8 Can_Receive_Msg(u8 *buf);							//��������
void Package_Send(u8 _mode,u8 *Package_Dat);
void CAN_BOOT_ExecutiveCommand(CanRxMsg *pRxMessage);
FLASH_Status CAN_BOOT_ErasePage(uint32_t StartPageAddr,uint32_t EndPageAddr);
FLASH_Status CAN_BOOT_ProgramDatatoFlash(uint32_t StartAddress,uint8_t *pData,uint32_t DataNum); 
uint16_t CAN_BOOT_GetAddrData(void);
void CAN_IAPCommand(CanRxMsg *pRxMessage);


#endif

















