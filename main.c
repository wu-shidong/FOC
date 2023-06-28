#include "config.h"
#include "main.h"
#include "STC32G_Delay.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "intrins.h"
#include "BLDC.h"
void RCC_init(void);


void main(void)
{
	RCC_init();
	GPIO_config();
	UART_config();
	EA = 1;
	while(1)
	{

	}
}


void RCC_init(void)
{
	WTST = 0;
	EAXSFR();		//扩展SFR(XFR)访问使能 
	CKCON = 0;      //提高访问XRAM速度
}


