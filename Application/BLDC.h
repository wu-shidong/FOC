#ifndef __BLDC_H
#define __BLDC_H

typedef struct
{
	float voltage_power_supply;//供电电压
	float zero_electric_angle;//电机零位角度值	
	float target_velocity;//目标速度	
	unsigned char pole_pairs;	//电机极对数
	
	float shaft_angle;//位差角
	float oplp_timestamp;//开环计算周期时间

	float Uq;//帕克逆变换电压q
	float Ud;//帕克逆变换电压d
	float Ualpha;//帕克逆变换电压alpha
	float Ubeta;//帕克逆变换电压beta
	float Ua;//电机a相电压
	float Ub;//电机b相电压
	float Uc;//电机c相电压
	float dc_a;//a相占空比（duty cycle)
	float dc_b;//b相占空比（duty cycle)
	float dc_c;//c相占空比（duty cycle)
	float angle_elec;//电角度

}BLDC;
extern void bldc_openloop_init(BLDC *bldc_init,float voltage_power_supply,float pole_pairs);
extern void velocityOpenloop(BLDC *bldc_set);

extern BLDC bldc_oplp;//开环控制
#endif
