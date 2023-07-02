#include 	"bsp_i2c.h"
#include 	"STC32G_NVIC.h"
#include	"STC32G_Switch.h"
#define DIS_DOT     0x20
#define DIS_BLACK   0x10
#define DIS_        0x11



void I2C_config(void)
{
	I2C_InitTypeDef		I2C_InitStructure;
	I2C_InitStructure.I2C_Mode      = I2C_Mode_Master;		//主从选择   I2C_Mode_Master, I2C_Mode_Slave
	I2C_InitStructure.I2C_Enable    = ENABLE;
	I2C_InitStructure.I2C_MS_WDTA   = DISABLE;
	I2C_InitStructure.I2C_Speed			= 63;
	I2C_Init(&I2C_InitStructure);
	NVIC_I2C_Init(I2C_Mode_Master,DISABLE,Priority_0);	//主从模式, I2C_Mode_Master, I2C_Mode_Slave; 中断使能, I2C_ESTAI/I2C_ERXI/I2C_ETXI/I2C_ESTOI/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
	I2C_SW(I2C_P33_P32);					//I2C_P14_P15,I2C_P24_P25,I2C_P76_P77,I2C_P33_P32
}


