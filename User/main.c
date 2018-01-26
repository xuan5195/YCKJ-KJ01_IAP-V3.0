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
#include "bsp.h"


/* ���� ----------------------------------------------------------------------*/
uint8_t g_RxMessage[8]={0};	//CAN��������
uint8_t g_RxMessFlag=0;		//CAN�������� ��־
extern uint8_t Physical_ADD[4];//�����ַ
extern uint8_t FM1702_Key[7];
extern uint8_t WaterCost,CostNum;	//WaterCost=ˮ�� ��С�ۿ���  //������
extern uint8_t g_IAP_Flag;	//����������־


extern pFunction Jump_To_Application;
extern uint32_t JumpAddress;

/* �������� ------------------------------------------------------------------*/
void Delay(__IO uint32_t nCount);
static void IAP_Init(void);
void USART_Configuration(void);
/* �������� ------------------------------------------------------------------*/

/*******************************************************************************
  * @��������	main
  * @����˵��   ������
  * @�������   ��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
int main(void)
{
	uint8_t ProgramCount=0;
    FLASH_Unlock();//Flash ����
	BspTm1639_Config();
	BspTm1639_Show(0x03,0x00);
    IAP_Init();
	SerialPutString("\r\n\r\nYCKJ-KJ01_IAP V0.1...\r\nStarting Up...");
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,25,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������90Kbps    
	Read_Flash_Dat();	//��ȡFlash����
	printf("g_IAP_Flag:0x%02X;\r\n",g_IAP_Flag);
//	g_IAP_Flag = 0xAA;
    while (1)
    {
		if (g_IAP_Flag  == 0xAA)
		{//ִ��IAP�����������Flash����
			SerialPutString("\r\n======================================================================");
			SerialPutString("\r\n=     In-Application Programming Application  (Version 1.2.4)        =");
			SerialPutString("\r\n======================================================================");
			SerialPutString("\r\n\r\n");
			Main_Menu ();
		}		
		else//����ִ���û�����
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
				Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);
				Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);
				Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);Delay(0xFFFF);
				if(ProgramCount>200)	g_IAP_Flag = 0xAA;	//ת��IAP�������ģʽ
				else 					ProgramCount++;
			}
		}
    }
}


/*******************************************************************************
  * @��������	GPIO_Configuration
  * @����˵��   ����ʹ��USART1�����IO�ܽ�
  * @�������   ��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
void GPIO_Configuration(void)
{
}

/*******************************************************************************
  * @��������	IAP_Init
  * @����˵��   ����ʹ��IAP
  * @�������   ��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
void IAP_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* USART1 ���� ------------------------------------------------------------
         USART1 ��������:
          - ������      = 115200 baud
          - �ֳ�        = 8 Bits
          - һ��ֹͣλ
          - ��У��
          - ��Ӳ��������
          - ���ܺͷ���ʹ��
    --------------------------------------------------------------------------*/
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    // ���� USART1 Tx (PA.09) ��Ϊ�������Ų��������ģʽ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //���� USART1 Tx (PA.10) ��Ϊ�������Ų��Ǹ�������ģʽ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    USART_Init(USART1, &USART_InitStructure);
    // ʹ�� USART1
    USART_Cmd(USART1, ENABLE);
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
