#include "BLDC.h"
#include <MATH.H>
#include <stdio.h>
#include "user_pid.h"
#include "bsp_pwm.h"
//初始变量及函数定义
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define PI 3.14159f
u16 Ts;
BLDC bldc_oplp;//开环控制

void bldc_openloop_init(BLDC *bldc_init,float voltage_power_supply,float pole_pairs)
{
  //输入电源电压
	bldc_init->voltage_power_supply=voltage_power_supply;

  //电机极对数
  bldc_init->pole_pairs=pole_pairs;

  //位差角为0
	bldc_init->shaft_angle=0;
  // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
  // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
  bldc_init->Uq = bldc_init->voltage_power_supply/5;
}




// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
   
	float a = fmod(angle, 2*PI);//取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  //三目运算符。格式：condition ? expr1 : expr2 
  
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}




void velocityOpenloop(BLDC *bldc_set)
{
  bldc_set->target_velocity=10;
	TR0 = 0;
	Ts=(TH0<<8)|TL0;
	TH0 = 0;
	TL0 = 0;
	TR0 = 1;
  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。 
	//以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
  //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。
//  bldc_set->shaft_angle = _normalizeAngle(bldc_set->shaft_angle + bldc_set->target_velocity*bldc_set->oplp_timestamp);
	bldc_set->shaft_angle = _normalizeAngle(bldc_set->shaft_angle + bldc_set->target_velocity*Ts/35*12*1e-6f);
  //计算电角度
  bldc_set->angle_elec=bldc_set->shaft_angle*bldc_set->pole_pairs;
  //电角度归一化
  bldc_set->angle_elec = _normalizeAngle(bldc_set->angle_elec + bldc_set->zero_electric_angle);
  // 帕克逆变换
  bldc_set->Ualpha =  -bldc_set->Uq*sin(bldc_set->angle_elec); 
  bldc_set->Ubeta =   bldc_set->Uq*cos(bldc_set->angle_elec); 
  // 克拉克逆变换
  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
	//设置各相的占空比
  bldc_set->dc_a = _constrain(bldc_set->Ua / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_b = _constrain(bldc_set->Ub / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_c = _constrain(bldc_set->Uc / bldc_set->voltage_power_supply, 0.0f , 1.0f );
	PWMA_Duty.PWM1_Duty=(u16)(bldc_set->dc_a*2047);
	PWMA_Duty.PWM2_Duty=(u16)(bldc_set->dc_b*2047);
  PWMA_Duty.PWM3_Duty=(u16)(bldc_set->dc_c*2047);
	UpdatePwm(PWMA, &PWMA_Duty);

}


