#include "bsp_gpio.h"
#include	"STC32G_Switch.h"

void GPIO_config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;				//�ṹ����

	GPIO_InitStructure.Pin  = GPIO_Pin_HIGH | GPIO_Pin_3;			//ָ��Ҫ��ʼ����IO, GPIO_Pin_0 ~ GPIO_Pin_7, �����
	GPIO_InitStructure.Mode = GPIO_PullUp;			//ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P6,&GPIO_InitStructure);	//��ʼ��

//UART1
	GPIO_InitStructure.Pin  = GPIO_Pin_0 | GPIO_Pin_1 |	GPIO_Pin_2 | GPIO_Pin_3;		//ָ��Ҫ��ʼ����IO, GPIO_Pin_0 ~ GPIO_Pin_7
	GPIO_InitStructure.Mode = GPIO_PullUp;	//ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	//��ʼ��
	
//UART2	P4_6:RX P4_7:TX
	GPIO_InitStructure.Pin  = GPIO_Pin_6 | GPIO_Pin_7;		//ָ��Ҫ��ʼ����IO, GPIO_Pin_0 ~ GPIO_Pin_7
	GPIO_InitStructure.Mode = GPIO_PullUp;	//ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P4,&GPIO_InitStructure);	//��ʼ��
	
	
//UART3
	GPIO_InitStructure.Pin  = GPIO_Pin_0 | GPIO_Pin_1;		//ָ��Ҫ��ʼ����IO, GPIO_Pin_0 ~ GPIO_Pin_7
	GPIO_InitStructure.Mode = GPIO_PullUp;	//ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P0,&GPIO_InitStructure);	//��ʼ��	
	
	
	CAN1_SW(CAN1_P50_P51);				//CAN1_P00_P01,CAN1_P50_P51,CAN1_P42_P45,CAN1_P70_P71
//	CAN2_SW(CAN2_P52_P53);				//CAN2_P02_P03,CAN2_P52_P53,CAN2_P46_P47,CAN2_P72_P73
}
