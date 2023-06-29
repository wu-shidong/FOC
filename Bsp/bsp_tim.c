#include "bsp_tim.h"
#include	"config.h"
#include	"STC32G_NVIC.h"
#include	"STC32G_Timer.h"

/*************	功能说明	**************

程序演示5个定时器的使用, 均使用16位自动重装.

定时器0做16位自动重装, 中断频率为100000Hz，中断函数从P6.7取反输出50KHz方波信号.

定时器1做16位自动重装, 中断频率为10000Hz，中断函数从P6.6取反输出5KHz方波信号.

定时器2做16位自动重装, 中断频率为1000Hz，中断函数从P6.5取反输出500Hz方波信号.

定时器3做16位自动重装, 中断频率为100Hz，中断函数从P6.4取反输出50Hz方波信号.

定时器4做16位自动重装, 中断频率为50Hz，中断函数从P6.3取反输出25Hz方波信号.

下载时, 选择时钟 24MHz (可以在配置文件"config.h"中修改).

******************************************/


void	Timer_config(void)
{
	
	TIM_InitTypeDef		TIM_InitStructure;						//结构定义
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));		//中断频率, 1000次/秒
	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);					//初始化Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer0_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3	
	

	

	
	
	
	
//	TIM_InitTypeDef		TIM_InitStructure;						//结构定义
//	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
//	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000000UL));		//初值,
//	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
//	Timer_Inilize(Timer0,&TIM_InitStructure);					//初始化Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
//	NVIC_Timer0_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3

//	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源, TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
//	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 10000));			//初值,
//	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
//	Timer_Inilize(Timer1,&TIM_InitStructure);					//初始化Timer1	  Timer0,Timer1,Timer2,Timer3,Timer4
//	NVIC_Timer1_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3

//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
//	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000));				//初值
//	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
//	Timer_Inilize(Timer2,&TIM_InitStructure);					//初始化Timer2	  Timer0,Timer1,Timer2,Timer3,Timer4
//	NVIC_Timer2_Init(ENABLE,NULL);		//中断使能, ENABLE/DISABLE; 无优先级

//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;	//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = ENABLE;					//是否输出高速脉冲, ENABLE或DISABLE
//	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (100*12)));		//初值
//	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
//	Timer_Inilize(Timer3,&TIM_InitStructure);					//初始化Timer3	  Timer0,Timer1,Timer2,Timer3,Timer4
//	NVIC_Timer3_Init(ENABLE,NULL);		//中断使能, ENABLE/DISABLE; 无优先级

//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;	//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = ENABLE;					//是否输出高速脉冲, ENABLE或DISABLE
//	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (50*12)));		//初值
//	TIM_InitStructure.TIM_Run       = ENABLE;					//是否初始化后启动定时器, ENABLE或DISABLE
//	Timer_Inilize(Timer4,&TIM_InitStructure);					//初始化Timer4	  Timer0,Timer1,Timer2,Timer3,Timer4
//	NVIC_Timer4_Init(ENABLE,NULL);		//中断使能, ENABLE/DISABLE; 无优先级
}






