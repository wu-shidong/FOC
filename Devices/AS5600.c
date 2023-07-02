#include "AS5600.h"
#include "bsp_i2c.h"
#define M_PI				3.1415926f
AS5600 as5600;


void AS5600_init (void)
{
	  as5600.previous_angle = 0.0;
		as5600.revolutions = 0;
}

unsigned int _AS5600_get_encode(void)
{
		I2C_ReadNbyte(0x36<<1,0x0C, &as5600.str, 2);
		as5600.encode=((u16)as5600.str[0]<<8)|(u16)as5600.str[1];
		return as5600.encode;
}
float _AS5600_getAngle_Without_track(void)
{
	I2C_ReadNbyte(0x36<<1,0x0C, &as5600.str, 2);
	as5600.encode =((u16)as5600.str[0]<<8)|(u16)as5600.str[1];
	as5600.current_angle  =(float)as5600.encode/2048*3.1415f;
	return as5600.current_angle;
}


float _AS5600_get_absolute_angle(void)
{
		static float angle_change=0;
		I2C_ReadNbyte(0x36<<1,0x0C, &as5600.str, 2);
		as5600.encode =((u16)as5600.str[0]<<8)|(u16)as5600.str[1];
		as5600.current_angle  =(float)as5600.encode/2048*3.1415f;  
    angle_change = as5600.current_angle - as5600.previous_angle;
    if (angle_change < -M_PI) {
        as5600.revolutions++;
    } else if (angle_change > M_PI) {
        as5600.revolutions--;
    }
    as5600.absolute_angle = as5600.current_angle + (2 * M_PI * as5600.revolutions);
    as5600.previous_angle = as5600.current_angle;
    return as5600.absolute_angle;
}