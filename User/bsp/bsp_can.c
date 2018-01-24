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
volatile uint8_t CAN1_CanRxMsgFlag=0;//���յ�CAN���ݺ�ı�־
extern uint8_t g_RxMessage[8];	//CAN��������
extern uint8_t g_RxMessFlag;	//CAN�������� ��־
extern uint8_t g_PrintfFlag;

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:1~3; CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.��Χ:1~8;
//tbs1:ʱ���1��ʱ�䵥Ԫ.��Χ:1~16;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
//ע�����ϲ����κ�һ����������Ϊ0,�������.
//������=Fpclk1/((tsjw+tbs1+tbs2)*brp);
//mode:0,��ͨģʽ;1,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Normal_Init(1,8,7,5,1);
//������Ϊ:36M/((1+8+7)*5)=450Kbps
//������Ϊ:36M/((1+13+2)*18)=125Kbps CAN_Normal_Init(1,13,2,18,1);
//����ֵ:0,��ʼ��OK;	����,��ʼ��ʧ��;


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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);		//��ʼ��IO
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��IO
	  
 	//CAN��Ԫ����
 	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ֹʱ�䴥��ͨ��ģʽ
 	CAN_InitStructure.CAN_ABOM=DISABLE;	//�����CAN_MCR�Ĵ�����INRQλ������1�����0��һ��Ӳ����⵽128��11λ����������λ�����˳�����״̬��
  	CAN_InitStructure.CAN_AWUM=DISABLE;	//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)//
  	CAN_InitStructure.CAN_NART=ENABLE;	//ENABLE:CAN����ֻ������1�Σ����ܷ��͵Ľ����Σ��ɹ���������ٲö�ʧ��
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//�ڽ������ʱFIFOδ��������������FIFO�ı���δ����������һ���յ��ı��ĻḲ��ԭ�еı���
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//����FIFO���ȼ��ɱ��ĵı�ʶ��������
  	CAN_InitStructure.CAN_Mode= mode;	//ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; //
  	//���ò�����
  	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;        //��Ƶϵ��(Fdiv)Ϊbrp+1	//
  	CAN_Init(CAN1, &CAN_InitStructure);         // ��ʼ��CAN1 

 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //���ù�������0����ΧΪ0~13  
 	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //CAN_FilterMode_IdMask:����ģʽ  CAN_FilterMode_IdList:�б�ģʽ
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //���ù�����λ��Ϊ32λ  
		CAN_FilterInitStructure.CAN_FilterIdHigh=std_id<<5;  		//���ñ�ʶ���Ĵ������ֽ�  
		CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;		//���ñ�ʶ���Ĵ������ֽ�	
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xA000; 		//�������μĴ������ֽ�  ֻ����std_id����λΪ��101��
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0|CAN_ID_STD;	//�������μĴ������ֽ�  
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
 	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0

  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
	
#if CAN_RX0_INT_ENABLE	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif

	return 0;
}   
 
#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�

u8 g_CanRecCount=0;//CAN���ݰ����ձ�־��0-15��ÿ�ν���16֡
u8 g_CanPackDat[128+8]={0};	//ǰ128��Ϊ���ݰ������һ��ΪCRC8
void USB_LP_CAN1_RX0_IRQHandler(void)	//�жϷ�����		
{
	CAN_Receive(CAN1,CAN_FIFO0, &CAN1_RxMessage);
//	printf("CanRxMsg:%03X %02X%02X %02X%02X %02X%02X %02X%02X;\r\n",CAN1_RxMessage.StdId,\
	CAN1_RxMessage.Data[0],CAN1_RxMessage.Data[1],CAN1_RxMessage.Data[2],CAN1_RxMessage.Data[3],\
	CAN1_RxMessage.Data[4],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[7]);
//	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
	CAN1_CanRxMsgFlag = 1;
	
}
#endif

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ16)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;	����,ʧ��;
u8 Can_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	if((msg[0]==0xB3)||(msg[0]==0xC2))	
		TxMessage.StdId=0x200;			// ��׼��ʶ��Ϊ0
	else				
		TxMessage.StdId=0x200|msg[1];	// ��׼��ʶ��Ϊ0
	TxMessage.ExtId=0x1800F001;			// ������չ��ʾ����29λ��
	TxMessage.IDE=0;		// ��ʹ����չ��ʶ��
	TxMessage.RTR=0;		// ��Ϣ���ͣ�CAN_RTR_DataΪ����֡;CAN_RTR_RemoteΪԶ��֡
	TxMessage.DLC=len;		// ������֡��Ϣ
	for(i=0;i<8;i++)
	TxMessage.Data[i]=msg[i];	// ֡��Ϣ          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;		

}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;	 ����,���յ����ݳ���;
u8 Can_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);				//��ȡ����	
    for(i=0;i<8;i++)    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

void Package_Send(u8 _mode,u8 *Package_Dat)
{
	uint8_t Package_SendBuf[8]={0x00};//���ͻ�����
	if (_mode==0x00)
    {
		Package_SendBuf[0] = 0xB3;
		Package_SendBuf[1] = Package_Dat[1];	//�����ַ
		Package_SendBuf[2] = Package_Dat[2];
		Package_SendBuf[3] = Package_Dat[3];
		Package_SendBuf[4] = Package_Dat[4];
		Package_SendBuf[5] = Package_Dat[5];
		Package_SendBuf[6] = Package_Dat[6];
		Package_SendBuf[7] = Package_Dat[7];
		Can_Send_Msg((u8 *)Package_SendBuf,8);//����8���ֽ�
    }
}
// ����һ֡CAN����
// CANx CANͨ����
//TxMessage CAN��Ϣָ��

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
//ִ�������·�������
//pRxMessage CAN������Ϣ
void CAN_BOOT_ExecutiveCommand(CanRxMsg *pRxMessage)
{
	CanTxMsg TxMessage;
	uint8_t ret,i;
	uint8_t can_cmd = (pRxMessage->StdId)&0x0F;		//ID��bit0~bit3λΪ������ 0x7FF ����λΪ������
	uint16_t crc_data;
	static uint32_t start_addr;
	static uint32_t data_size=0;
	static uint32_t data_index=0;
	__align(4) static uint8_t	data_temp[1028];	//������ ����
	TxMessage.ExtId = 0;
	TxMessage.IDE = CAN_Id_Standard;	//��ʹ����չ��ʶ��
	TxMessage.RTR = CAN_RTR_Data;		//��Ϣ���ͣ�CAN_RTR_DataΪ����֡;CAN_RTR_RemoteΪԶ��֡
	TxMessage.DLC = 0;					//���ݳ���Ϊ0��
	switch (can_cmd)
	{
		//CMD_List.EraseFlash������Flash�е����ݣ���ʼ��ַ�洢��Data[0]��Data[3]�У�������ַ�洢��Data[4]��Data[7]��
		case CMD_List_EraseFlash:
			SerialPutString("CMD_List_EraseFlash, ");
			__set_PRIMASK(1);
			FLASH_Unlock();
			printf("Start_addr:0x%02X%02X%02X%02X;  And_addr:0x%02X%02X%02X%02X;  ",\
			pRxMessage->Data[0],pRxMessage->Data[1],pRxMessage->Data[2],pRxMessage->Data[3],\
			pRxMessage->Data[4],pRxMessage->Data[5],pRxMessage->Data[6],pRxMessage->Data[7]);
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
			printf("EraseFlash Over;\r\n");
			break;
		//CMD_List.BlockWriteInfo������дFlash���ݵ������Ϣ������������ʼ��ַ�����ݴ�С
		//������ʼ��ַ�洢��Data[0]��Data[3]�У����ݴ�С�洢��Data[4]��Data[7]�У��ú���������Bootloader������ʵ�֣�APP������Բ���ʵ��
		case CMD_List_BlockWriteInfo:
			printf("CMD_List_BlockWriteInfo, ");
			start_addr = (pRxMessage->Data[0]<<24)|(pRxMessage->Data[1]<<16)|(pRxMessage->Data[2]<<8)|(pRxMessage->Data[3]<<0);
			data_size = (pRxMessage->Data[4]<<24)|(pRxMessage->Data[5]<<16)|(pRxMessage->Data[6]<<8)|(pRxMessage->Data[7]<<0);
			printf("Start_addr:0x%02X%02X%02X%02X;  Data_size:%4d;  ",\
			pRxMessage->Data[0],pRxMessage->Data[1],pRxMessage->Data[2],pRxMessage->Data[3],data_size);
			data_index = 0;
//			if(can_addr != 0x00)
			{
				TxMessage.StdId = CAN_BOOT_GetAddrData()|CMD_List_BlockWriteInfo;	
				TxMessage.DLC = 0;
				CAN_WriteData(&TxMessage);
			}
			break;
		//CMD_List.WriteBlockFlash���Ƚ����ݴ洢�ڱ��ػ������У�Ȼ��������ݵ�CRC����У����ȷ��д���ݵ�Flash��
		//ÿ��ִ�и����ݣ����ݻ������������ֽ���������pRxMessage->DLC�ֽڣ����������ﵽdata_size������2�ֽ�CRCУ���룩�ֽں�
		//�����ݽ���CRCУ�飬������У������������д��Flash��
		//�ú�����Bootloader�����б���ʵ�֣�APP������Բ���ʵ��
		case CMD_List_WriteBlockFlash:
			printf("CMD_List_WriteBlockFlash, %4d; >>",data_index);
			if((data_index<data_size)&&(data_index<1026))
			{
				__set_PRIMASK(1);
				for(i=0;i<pRxMessage->DLC;i++)
				{
					data_temp[data_index++] = pRxMessage->Data[i];
					printf(" %02X",pRxMessage->Data[i]);
				}
				printf("\r\n");
				__set_PRIMASK(0);
			}
			if((data_index>=data_size)||(data_index>=1026))
			{
				printf("(data_index>=data_size)||(data_index>=1026);\r\n");
				crc_data = crc16_ccitt(data_temp,data_size-2);
				if(crc_data==((data_temp[data_size-2]<<8)|(data_temp[data_size-1])))
				{
					__set_PRIMASK(1);
					FLASH_Unlock();
					ret = CAN_BOOT_ProgramDatatoFlash(start_addr,data_temp,data_index-2);
					printf("ProgramDatatoFlash.\r\n");
					FLASH_Lock();	
					__set_PRIMASK(0);
//					if(can_addr != 0x00)
					{
						if(ret==FLASH_COMPLETE)
						{
							TxMessage.StdId = CAN_BOOT_GetAddrData()|CMD_List_WriteBlockFlash;
							printf("ProgramDatatoFlash Over.\r\n");							
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
		
		case CMD_List_ExcuteApp:  //CMD_List.ExcuteApp�����Ƴ�����ת��ָ����ִַ��
			SerialPutString("CMD_List_ExcuteApp\r\n");
			CAN_JumpToApplication();
			break;
		default:
			break;
	}
}

//ִ�������·�������
//pRxMessage CAN������Ϣ
void CAN_IAPCommand(CanRxMsg *pRxMessage)
{
	uint8_t i;
	uint8_t crc_data;
	static uint8_t pack_no;
	static uint32_t start_addr;
	static uint32_t data_index=0;
	__align(4) static uint8_t	data_temp[128+8];	//������ ����
	switch ((pRxMessage->StdId)&0x0FF)
	{
		case 0xFB:  //���ڴ�ӡ�ڲ�Flash���ݣ����ڴ�ӡ ����ʹ��
			g_PrintfFlag = 0xAA;	//��ӡ��־
			break;
		case 0xFC:  //���Ƴ�����ת��ָ����ִַ��
			SerialPutString("CMD_List_ExcuteApp\r\n");
			CAN_JumpToApplication();
			break;
		case 0xFD:	//������ӦFLashҳ
			__set_PRIMASK(1);
			FLASH_Unlock();
			CAN_BOOT_ErasePage(0x08005000,0x0800F800);	//����42K Flash
			FLASH_Lock();	
			__set_PRIMASK(0);
			printf("EraseFlash Over;\r\n");
			break;
		case 0xFE:	//���ݰ�ͷ+CRC
			__set_PRIMASK(1);
			data_index = 0;
			pack_no = pRxMessage->Data[0];			//���ݰ����
			data_temp[128] = pRxMessage->Data[1];	//CRC
			printf("pack_no:%02X, CRC:%02X.\r\n",pRxMessage->Data[0],pRxMessage->Data[1]);
			__set_PRIMASK(0);
			break;
		default:	//���ݰ�����
			if( ((pRxMessage->StdId)&0x0FF) == pack_no )
			{
				__set_PRIMASK(1);
				for(i=0;i<8;i++)
				{
					data_temp[data_index++] = pRxMessage->Data[i];
//					printf(" %02X",pRxMessage->Data[i]);
				}
//				printf("\r\n");
				__set_PRIMASK(0);
			}
			if(data_index>=128)
			{
				crc_data = CRC8_Table(data_temp,128);
				printf("(data_index>=128) crc_data=%02X,data_temp[128]=%02X\r\n",crc_data,data_temp[128]);
				if( crc_data == data_temp[128] )
				{
					__set_PRIMASK(1);
					FLASH_Unlock();
					start_addr = ApplicationAddress + 128*(pRxMessage->StdId&0x0FF);
					CAN_BOOT_ProgramDatatoFlash(start_addr,data_temp,128);
					printf("start_addr=%08X, ProgramDatatoFlash:\r\n",start_addr);
					FLASH_Lock();	
					__set_PRIMASK(0);
//					for(i=0;i<16;i++)
//					{
//						printf("i=%02d: %02X%02X %02X%02X %02X%02X %02X%02X; \r\n",i,data_temp[i*8+0],data_temp[i*8+1],\
//						data_temp[i*8+2],data_temp[i*8+3],data_temp[i*8+4],data_temp[i*8+5],data_temp[i*8+6],data_temp[i*8+7]);
//					}
				}
				data_index = 0;
				for(i=0;i<136;i++)	data_temp[i]=0;
			}
			break;
	}
}


//����ָ�����������Flash���� ��
//StartPage ��ʼ������ַ
//EndPage ����������ַ
//��������״̬  
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

//��ȡ�ڵ��ַ��Ϣ
//�ڵ��ַ
uint16_t CAN_BOOT_GetAddrData(void)
{
	return 0x500;
}

/**
  * @brief  ��������д��ָ����ַ��Flash�� ��
  * @param  Address Flash��ʼ��ַ��
  * @param  Data ���ݴ洢����ʼ��ַ��
  * @param  DataNum �����ֽ�����
  * @retval ������д״̬��
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



