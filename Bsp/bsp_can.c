#include "bsp_can.h"
#include	"STC32G_NVIC.h"

void CAN_config(void)
{
	CAN_InitTypeDef	CAN_InitStructure;					//结构定义

	CAN_InitStructure.CAN_Enable = ENABLE;		//CAN功能使能   ENABLE或DISABLE
	CAN_InitStructure.CAN_IMR    = CAN_ALLIM;	//CAN中断寄存器 	CAN_DOIM,CAN_BEIM,CAN_TIM,CAN_RIM,CAN_EPIM,CAN_EWIM,CAN_ALIM,CAN_ALLIM,DISABLE
	CAN_InitStructure.CAN_SJW    = 0;					//重新同步跳跃宽度  0~3
	CAN_InitStructure.CAN_SAM    = 0;					//总线电平采样次数  0:采样1次; 1:采样3次

//	//CAN总线波特率=Fclk/((1+(TSG1+1)+(TSG2+1))*(BRP+1)*2)
//	CAN_InitStructure.CAN_TSG1   = 2;					//同步采样段1       0~15
//	CAN_InitStructure.CAN_TSG2   = 1;					//同步采样段2       1~7 (TSG2 不能设置为0)
//	CAN_InitStructure.CAN_BRP    = 3;					//波特率分频系数    0~63
//	//24000000/((1+3+2)*4*2)=500KHz
	//CAN总线波特率=Fclk/((1+(TSG1+1)+(TSG2+1))*(BRP+1)*2)
	CAN_InitStructure.CAN_TSG1   = 2;					//同步采样段1       0~15
	CAN_InitStructure.CAN_TSG2   = 2;					//同步采样段2       1~7 (TSG2 不能设置为0)
	CAN_InitStructure.CAN_BRP    = 4;					//波特率分频系数    0~63
	//35000000/((1+3+3)*5*2)=500KHz	
	

	CAN_InitStructure.CAN_ACR0    = 0x00;			//总线验收代码寄存器 0~0xFF
	CAN_InitStructure.CAN_ACR1    = 0x00;
	CAN_InitStructure.CAN_ACR2    = 0x00;
	CAN_InitStructure.CAN_ACR3    = 0x00;
	CAN_InitStructure.CAN_AMR0    = 0xff;			//总线验收屏蔽寄存器 0~0xFF
	CAN_InitStructure.CAN_AMR1    = 0xff;
	CAN_InitStructure.CAN_AMR2    = 0xff;
	CAN_InitStructure.CAN_AMR3    = 0xff;
	CAN_Inilize(CAN1,&CAN_InitStructure);			//CAN1 初始化
//	CAN_Inilize(CAN2,&CAN_InitStructure);			//CAN2 初始化
	
	NVIC_CAN_Init(CAN1,ENABLE,Priority_1);		//中断使能, CAN1/CAN2; ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
//	NVIC_CAN_Init(CAN2,ENABLE,Priority_1);		//中断使能, CAN1/CAN2; ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}





void can1_reset(void)
{
	CAN1_Tx.FF = STANDARD_FRAME;    //标准帧
	CAN1_Tx.RTR = 0;                //0：数据帧，1：远程帧
	CAN1_Tx.DLC = 0x08;             //数据长度
	CAN1_Tx.ID = 0x201;            //CAN ID
	CAN1_Tx.DataBuffer[0] = 0x00;   //数据内容
	CAN1_Tx.DataBuffer[1] = 0x00;
	CAN1_Tx.DataBuffer[2] = 0x00;
	CAN1_Tx.DataBuffer[3] = 0x00;
	CAN1_Tx.DataBuffer[4] = 0x00;
	CAN1_Tx.DataBuffer[5] = 0x00;
	CAN1_Tx.DataBuffer[6] = 0x00;
	CAN1_Tx.DataBuffer[7] = 0x00;
}


void can1_send_data(u32 ID,u16 ecd,int16 speed_rpm,int16 current,u16 int_s)
{
	CAN1_Tx.FF = STANDARD_FRAME;    //标准帧
	CAN1_Tx.RTR = 0;                //0：数据帧，1：远程帧
	CAN1_Tx.DLC = 0x08;             //数据长度
	CAN1_Tx.ID = ID;            //CAN ID
	CAN1_Tx.DataBuffer[0] = ecd>>8;   //数据内容
	CAN1_Tx.DataBuffer[1] = ecd;
	CAN1_Tx.DataBuffer[2] = speed_rpm>>8;
	CAN1_Tx.DataBuffer[3] = speed_rpm;
	CAN1_Tx.DataBuffer[4] = current>>8;
	CAN1_Tx.DataBuffer[5] = current;
	CAN1_Tx.DataBuffer[6] = int_s>>8;
	CAN1_Tx.DataBuffer[7] = int_s;

	CANSEL = CAN1;		//选择CAN1模块 
	if(CanReadReg(SR) & 0x01)		//判断是否有 BS:BUS-OFF状态
	{
		CANAR = MR;
		CANDR &= ~0x04;  //清除 Reset Mode, 从BUS-OFF状态退出
	}
	else
	{
		CanSendMsg(&CAN1_Tx);   //发送一帧数据
	}
}








