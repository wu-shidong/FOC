#include "config.h"
#include "main.h"
#include "bsp_delay.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"
#include "bsp_i2c.h"
#include "BLDC.h"
#include "AS5600.h"
/*
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
*/
void RCC_init(void);

unsigned int encode;
float angle;
unsigned int cnt;
void main(void)
{

	RCC_init();
	GPIO_config();
	UART_config();
	Timer_config();
	PWM_config();
	I2C_config();
	AS5600_init();
	EA = 1;
	bldc_init(&bldc_clslp,16,7);
//	bldc_clslp_init(&bldc_clslp);
	cnt=0;
	while (1)
	{
		cnt++;
		if(cnt==100)
		{
			cnt=0;
			printf("v:%f,fv:%f,e:%f,rl:%f\r\n",bldc_clslp.angular_velocity,bldc_clslp.filtered_Velocity,bldc_clslp.e,bldc_clslp.diff_angle);
		}
		velocityOpenloop(&bldc_clslp,-3);
	}
}



void RCC_init(void)
{
	WTST = 0;
	EAXSFR();		//扩展SFR(XFR)访问使能 
	CKCON = 0;      //提高访问XRAM速度
}


