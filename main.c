#include "config.h"
#include "main.h"
#include "bsp_delay.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"
#include "bsp_i2c.h"
#include "bsp_can.h"
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
int velocity;
void main(void)
{
	int n,i;
	RCC_init();
	GPIO_config();
	UART_config();
	Timer_config();
	PWM_config();
	I2C_config();
	CAN_config();
	AS5600_init();
	EA = 1;
	bldc_init(&bldc_clslp,16,7,0x201);
//	bldc_clslp_init(&bldc_clslp);
//    bldc_clslp.target_velocity=3.14;
//    bldc_clslp.control_mode=velocity_Closeloop;
	while (1)
	{
    BLDC_control(&bldc_clslp);
 		if(B_Can1Read)      //�ж� CAN1 �Ƿ���յ�����
 		{
 			B_Can1Read = 0;
 			CANSEL = CAN1;		//ѡ��CAN1ģ��
             n = CanReadMsg(CAN1_Rx);    //��ȡ��������
             if(n>0)
             {
                 for(i=0;i<n;i++)
                 {
 //                    printf("CAN1 ID=0x%08lX DLC=%d FF=%d RTR=%d ",CAN1_Rx[i].ID,CAN1_Rx[i].DLC,CAN1_Rx[i].FF,CAN1_Rx[i].RTR);   //���ڴ�ӡ֡��Ϣ
                    
                     if(CAN1_Rx[i].ID==bldc_clslp.ID)
                     {
                         bldc_clslp.control_mode=CAN1_Rx[i].DataBuffer[0];
                         if(bldc_clslp.control_mode!=bldc_clslp.prev_control_mode)
                         {
                             bldc_clslp.prev_control_mode=bldc_clslp.control_mode;

                         }
                         switch (bldc_clslp.control_mode)
                         {
                         case velocity_Closeloop:
 													velocity=CAN1_Rx[i].DataBuffer[1]<<8 | CAN1_Rx[i].DataBuffer[2];
 //                            bldc_clslp.target_velocity=(CAN1_Rx[i].DataBuffer[1]<<8 | CAN1_Rx[i].DataBuffer[2])*3.14/180;
 															bldc_clslp.target_velocity=(float)velocity*3.14/180;
 															printf("n:%d,velocity:%f\r\n",n,bldc_clslp.target_velocity);
                             break;
                         default:
                             break;
                         }
                     }
                     
                 }
             }
 		}
	}
}







void RCC_init(void)
{
	WTST = 0;
	EAXSFR();		//��չSFR(XFR)����ʹ�� 
	CKCON = 0;      //��߷���XRAM�ٶ�
}


