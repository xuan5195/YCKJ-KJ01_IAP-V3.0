/*******************************************************************************
** �ļ���: 		mian.c
** �汾��  		1.0
** ��������: 	RealView MDK-ARM 4.14
** ����: 		wuguoyana
** ��������: 	2011-04-28
** ����:		USART��ʼ����RCC���ã�Ȼ���common.c��ִ�����˵�
** ����ļ�:	stm32f10x.h
** �޸���־��	2011-04-29   �����ĵ�
*******************************************************************************/
/* ����ͷ�ļ� *****************************************************************/
#include "common.h"
#include "bsp_tm1639.h"		
#include "bsp_can.h"		


/* ���� ----------------------------------------------------------------------*/
extern pFunction Jump_To_Application;
extern uint32_t JumpAddress;
extern uint8_t g_RxMessage[8];	//CAN��������
extern uint8_t g_RxMessFlag;	//CAN�������� ��־

/* �������� ------------------------------------------------------------------*/
void Delay(__IO uint32_t nCount);


void RCC_Configuration(void)
{
//    RCC_DeInit();
//    RCC_HSEConfig(RCC_HSE_OFF);
//    RCC_HSICmd(ENABLE);
    RCC_HSEConfig(RCC_HSE_ON);
}
/*******************************************************************************
  * @��������	main
  * @����˵��   ������
  * @�������   ��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
int main(void)
{   
	RCC_Configuration();
    FLASH_Unlock();		//Flash ����
	BspTm1639_Config();	//TM1639��ʼ��
	BspTm1639_Show(0x02,0x00);
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_7tq,5,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������450Kbps    
	printf("Starting Up...\r\n");

    while (1)
    {
		if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)  == 0x00)
		{
			//���簴������
			//ִ��IAP�����������Flash����

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
		//����ִ���û�����
		else
		{
			//�ж��û��Ƿ��Ѿ����س�����Ϊ��������´˵�ַ��ջ��ַ��
			//��û����һ��Ļ�����ʹû�����س���Ҳ�����������ܷɡ�
			if (((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
			{
				SerialPutString("Execute user Program\r\n\n");
				//��ת���û�����
				JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
				Jump_To_Application = (pFunction) JumpAddress;

				//��ʼ���û�����Ķ�ջָ��
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
  * @��������	Delay
  * @����˵��   ����һ����ʱʱ��
  * @�������   nCount: ָ����ʱʱ�䳤��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
void Delay(__IO uint32_t nCount)
{
    for (; nCount != 0; nCount--);
}

#ifdef  USE_FULL_ASSERT

/*******************************************************************************
  * @��������	assert_failed
  * @����˵��   �����ڼ�������������ʱ��Դ�ļ����ʹ�������
  * @�������   file: Դ�ļ���
  				line: ������������
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* �û����������Լ��Ĵ������ڱ��������ļ�������������,
       ���磺printf("�������ֵ: �ļ��� %s �� %d��\r\n", file, line) */

    //��ѭ��
    while (1)
    {
    }
}
#endif

/*******************************�ļ�����***************************************/
