#ifndef __BLDC_H
#define __BLDC_H

typedef struct
{
	float voltage_power_supply;//�����ѹ
	float zero_electric_angle;//�����λ�Ƕ�ֵ	
	float target_velocity;//Ŀ���ٶ�	
	unsigned char pole_pairs;	//���������
	
	float shaft_angle;//λ���
	float oplp_timestamp;//������������ʱ��

	float Uq;//������任��ѹq
	float Ud;//������任��ѹd
	float Ualpha;//������任��ѹalpha
	float Ubeta;//������任��ѹbeta
	float Ua;//���a���ѹ
	float Ub;//���b���ѹ
	float Uc;//���c���ѹ
	float dc_a;//a��ռ�ձȣ�duty cycle)
	float dc_b;//b��ռ�ձȣ�duty cycle)
	float dc_c;//c��ռ�ձȣ�duty cycle)
	float angle_elec;//��Ƕ�

}BLDC;
extern void bldc_openloop_init(BLDC *bldc_init,float voltage_power_supply,float pole_pairs);
extern void velocityOpenloop(BLDC *bldc_set);

extern BLDC bldc_oplp;//��������
#endif
