#ifndef __AS5600_H
#define __AS5600_H
typedef struct
{
	unsigned char str[2];
	unsigned int encode; //0~4095
	unsigned int degree;//0~360
	float current_angle ;//0~2PI
	float previous_angle ;//
	float absolute_angle;
	int revolutions ;//
	
}AS5600;
extern AS5600 as5600;
extern unsigned int _AS5600_get_encode(void);
extern float _AS5600_getAngle_Without_track(void);
extern float _AS5600_get_absolute_angle(void);
extern void AS5600_init (void);
#endif
