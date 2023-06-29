#include "config.h"
#include "main.h"
#include "STC32G_Delay.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"
#include "BLDC.h"
void RCC_init(void);


unsigned int cnt1;
unsigned int cnt2;
unsigned int dim;
unsigned int i;
void main(void)
{

	RCC_init();
	GPIO_config();
	UART_config();
	Timer_config();
	PWM_config();
	bldc_openloop_init(&bldc_oplp,16,7);
	EA = 1;
	while (1)
	{

//	TR0 = 0;
//	cnt1=(TH0<<8)|TL0;
//	TH0 = 0;
//	TL0 = 0;
//	TR0 = 1;
//	printf("%u\r\n",cnt1/35*12);
//	delay_ms(2);
	//velocityOpenloop(&bldc_oplp);
	//P67=~P67;
	velocityOpenloop(&bldc_oplp);	
//	velocityOpenloop(&bldc_oplp);	
//	velocityOpenloop(&bldc_oplp);

		
	}
}



void RCC_init(void)
{
	WTST = 0;
	EAXSFR();		//扩展SFR(XFR)访问使能 
	CKCON = 0;      //提高访问XRAM速度
}


