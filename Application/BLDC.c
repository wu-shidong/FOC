/**
  ********************************************************
  * @file       BLDC.c/h
  * @brief      包括对直流减速电机进行初始化参数设置速度开环、
  *             角度闭环、速度闭环、电流力矩环控制等函数
  * @note       
  * @history
  *  Version    Date            Author          Modification
  * V1.0.0     2023-6-26		    吴世栋          完成项目初始化
  * V1.0.1     2023-6-27		    吴世栋          完成PWM初始值设置
  * V1.0.2     2023-6-28		    吴世栋          完成运算周期计算
  * V1.0.3     2023-6-29		    吴世栋          完成开环速度环
  * V1.0.4     2023-6-30		    吴世栋          完成闭环角度环
  * V1.0.5     2023-7-01		    吴世栋          完成闭环速度环
  @verbatim
  ========================================================
  =========================================================
  @endverbatim
  ********************************************************
  */


#include "BLDC.h"
#include <MATH.H>
#include <stdio.h>
#include "bsp_pwm.h"
#include "bsp_i2c.h"
#include "bsp_delay.h"
#include "AS5600.h"
//初始变量及函数定义
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define PI 3.14159f
#define PI_2 1.57f
#define PI_4 0.7854f
BLDC bldc_oplp;//开环控制
BLDC bldc_clslp;//闭环控制
fp32 clslp_angle_pid[3]={CLSLP_ANGLE_KP,CLSLP_ANGLE_KI,CLSLP_ANGLE_KD};
fp32 clslp_velocity_pid[3]={CLSLP_VELOCITY_KP,CLSLP_VELOCITY_KI,CLSLP_VELOCITY_KD};
// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
   
	float a = fmod(angle, 2*PI);//取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  //三目运算符。格式：condition ? expr1 : expr2 
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}
// 归一化角度到 [-PI,PI]
float _normalizeAngle_PI(float angle)
{
	if(angle>PI)
	{
		angle=angle-2*PI;
	}else if(angle<-PI)
	{
		angle=angle+2*PI;
	}
return angle;
}

/**
 * @brief 电机参数初始化设置
*/

void bldc_init(BLDC *bldc_init,float voltage_power_supply,float pole_pairs)
{
  //设置输入电源电压
	bldc_init->voltage_power_supply=voltage_power_supply;

  //设置电机极对数
  bldc_init->pole_pairs=pole_pairs;

  //位差角为0
	bldc_init->shaft_angle=0;
  //零电角度为0
  bldc_init->zero_electric_angle=4.22;
  // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
  // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
  bldc_init->Uq = bldc_init->voltage_power_supply/3;
  //编码器方向
  bldc_init->dir=-1;
  //闭环角度环pid初始参数设置
	PID_init(&bldc_init->closeloop_angle_pid,PID_POSITION,clslp_angle_pid,bldc_init->voltage_power_supply/3,voltage_power_supply/20);
	//闭环速度环pid初始参数设置
  PID_init(&bldc_init->closeloop_velocity_pid,PID_DELTA,clslp_velocity_pid,bldc_init->voltage_power_supply/3,voltage_power_supply/20);
  //角速度低通滤波初始化
  initLowPassFilter(&bldc_init->velocity_lowPassFilter, 0.035);
}


/**
 * @brief 设置相电压函数
 * @param bldc_set 无刷电机结构体
 * @param Uq 电压幅值
 * @param Ud 一般是0
 * @param angle_el 给定电角度
 * 
*/
void setPhaseVoltage(BLDC *bldc_set,float Uq,float Ud, float angle_el)
 {
  //电角度归一化
  bldc_set->angle_elec = _normalizeAngle(angle_el);
  bldc_set->Uq=Uq;
	bldc_set->Ud=Ud;
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


void bldc_clslp_init(BLDC *bldc_set)
{
	
	float zero_electric_angle_CW;
	float zero_electric_angle_CCW;
	unsigned int i;
	printf("BLDC_init_start\r\n");
	while(fabs(_normalizeAngle_PI(bldc_set->relative_angle))>0.004f)
	{
		velocityOpenloop(bldc_set,0.5);
	}
  zero_electric_angle_CCW=bldc_set->angle_elec;
	for(i=0;i<1000;i++)
  {
		velocityOpenloop(bldc_set,0.5);
  }
	while(fabs(_normalizeAngle_PI(bldc_set->relative_angle))>0.004f)
	{
		velocityOpenloop(bldc_set,-0.5);
	}
	zero_electric_angle_CW=bldc_set->angle_elec;
//	printf("zero_electric_angle_CW=%f,%f\r\n",zero_electric_angle_CW,_normalizeAngle_PI(bldc_set->relative_angle));
	bldc_set->zero_electric_angle=(zero_electric_angle_CW+zero_electric_angle_CCW)/2;
	printf("BLDC zero_electric_angle=%f\r\n",bldc_set->zero_electric_angle);
}


/**
 * @brief 开环速度函数
 * @param bldc_set 电机对象结构体
 * @param velocity 目标速度
*/

void velocityOpenloop(BLDC *bldc_set,float velocity)
{
  bldc_set->target_velocity=velocity;
  bldc_set->Uq=bldc_set->voltage_power_supply/5;
  //计算时间周期
	TR0 = 0;
	bldc_set->Ts=(float)((TH0<<8)|TL0)/35*12*1e-6f;
	TH0 = 0;
	TL0 = 0;
	TR0 = 1;

  bldc_set->current_angle=_normalizeAngle(_AS5600_getAngle_Without_track()*bldc_set->dir);
  bldc_set->diff_angle=bldc_set->current_angle-bldc_set->prev_angle;
  if(fabs(bldc_set->diff_angle) > (0.8f*2*PI) ) bldc_set->current_full_rotations += ( bldc_set->diff_angle > 0 ) ? -1 : 1;
  bldc_set->angular_velocity= ((float)(bldc_set->current_full_rotations - bldc_set->prev_full_rotations) * 2 * PI + (bldc_set->diff_angle) ) / bldc_set->Ts;
  bldc_set->prev_angle=bldc_set->current_angle;
  bldc_set->prev_full_rotations=bldc_set->current_full_rotations;
  bldc_set->filtered_Velocity=lowPassFilter(&bldc_set->velocity_lowPassFilter,bldc_set->angular_velocity,bldc_set->Ts);

	  //获取绝对角度和相对角度
  bldc_set->absolute_angle=_AS5600_get_absolute_angle()*bldc_set->dir;
  bldc_set->relative_angle=_normalizeAngle(_AS5600_getAngle_Without_track()*bldc_set->dir);
	bldc_set->relative_angle_meg=_normalizeAngle(bldc_set->pole_pairs*bldc_set->relative_angle);
  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。 
	//以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
  //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。
	bldc_set->shaft_angle = _normalizeAngle(bldc_set->shaft_angle + bldc_set->target_velocity*bldc_set->Ts);
  //计算电角度并归一化
  bldc_set->angle_elec = _normalizeAngle(bldc_set->shaft_angle*bldc_set->pole_pairs+ bldc_set->zero_electric_angle);
	bldc_set->relative_angle_elec=_normalizeAngle(bldc_set->angle_elec-bldc_set->zero_electric_angle);
	bldc_set->e=_normalizeAngle_PI(bldc_set->relative_angle_meg-bldc_set->relative_angle_elec);
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
/**
 * @brief 开环电压函数
 * @param bldc_set 电机对象结构体
 * @param voltage 给定电压输出
*/
void voltageOpenloop (BLDC *bldc_set,int voltage)
{
  // 计算运算周期
  TR0 = 0;
	bldc_set->Ts=(TH0<<8)|TL0;
	TH0 = 0;
	TL0 = 0;
	TR0 = 1;
	//电压给值限幅
	bldc_set->target_voltage=_constrain(voltage,-30000,30000);
  //获取绝对角度和相对角度
  bldc_set->absolute_angle=_AS5600_get_absolute_angle()*bldc_set->dir;
  bldc_set->relative_angle=_normalizeAngle(_AS5600_getAngle_Without_track()*bldc_set->dir);
	bldc_set->relative_angle_meg=_normalizeAngle(bldc_set->pole_pairs*bldc_set->relative_angle);
	if(bldc_set->target_voltage)
	{
		bldc_set->angle_elec=_normalizeAngle((float)(bldc_set->pole_pairs*bldc_set->relative_angle)+bldc_set->zero_electric_angle+PI_2);
	}else{
		bldc_set->angle_elec=_normalizeAngle((float)(bldc_set->pole_pairs*bldc_set->relative_angle)+bldc_set->zero_electric_angle-PI_2);
	}
  bldc_set->relative_angle_elec=_normalizeAngle(bldc_set->angle_elec-bldc_set->zero_electric_angle);
	bldc_set->e=_normalizeAngle_PI(bldc_set->relative_angle_meg-bldc_set->relative_angle_elec);
  //计算Uq电压输出
  bldc_set->Uq=(float)(fabs(bldc_set->target_voltage)/30000)*(bldc_set->voltage_power_supply/3);
	// 帕克逆变换
  bldc_set->Ualpha = -bldc_set->Uq*sin(bldc_set->angle_elec); 
  bldc_set->Ubeta = bldc_set->Uq*cos(bldc_set->angle_elec); 
  // 克拉克逆变换
  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
	//设置各相的占空比
  bldc_set->dc_a = _constrain(bldc_set->Ua / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_b = _constrain(bldc_set->Ub / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_c = _constrain(bldc_set->Uc / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  //PWM通道赋值
	PWMA_Duty.PWM1_Duty=(u16)(bldc_set->dc_a*2047);
	PWMA_Duty.PWM2_Duty=(u16)(bldc_set->dc_b*2047);
  PWMA_Duty.PWM3_Duty=(u16)(bldc_set->dc_c*2047);
  //PWM状态更新
	UpdatePwm(PWMA, &PWMA_Duty);
}




/**
 * @brief 闭环角度函数
 * @param bldc_set 电机对象结构体
 * @param velocity 目标角度
*/
void angleCloseloop (BLDC *bldc_set,float angle)
{
  TR0 = 0;
	bldc_set->Ts=(TH0<<8)|TL0;
	TH0 = 0;
	TL0 = 0;
	TR0 = 1;
  bldc_set->target_angle=angle;
  //获取绝对角度
  bldc_set->absolute_angle=_AS5600_get_absolute_angle()*bldc_set->dir;
  //获取相对角度
  bldc_set->relative_angle=_normalizeAngle(_AS5600_getAngle_Without_track()*bldc_set->dir);
  //获取相对磁角度
	bldc_set->relative_angle_meg=_normalizeAngle(bldc_set->pole_pairs*bldc_set->relative_angle);
  bldc_set->target_voltage=PID_calc(&bldc_set->closeloop_angle_pid,bldc_set->absolute_angle,bldc_set->target_angle);
  bldc_set->e=_normalizeAngle_PI(bldc_set->target_angle-bldc_set->absolute_angle);
	
  if(bldc_set->target_voltage>0)
	{
		bldc_set->angle_elec=_normalizeAngle(bldc_set->relative_angle_meg+bldc_set->zero_electric_angle+PI_2);
	}else{
		bldc_set->angle_elec=_normalizeAngle(bldc_set->relative_angle_meg+bldc_set->zero_electric_angle-PI_2);
	}
  bldc_set->relative_angle_elec=_normalizeAngle(bldc_set->angle_elec-bldc_set->zero_electric_angle);
  //计算Uq电压输出
  bldc_set->Uq=fabs(bldc_set->target_voltage);
	// 帕克逆变换
  bldc_set->Ualpha = -bldc_set->Uq*sin(bldc_set->angle_elec); 
  bldc_set->Ubeta = bldc_set->Uq*cos(bldc_set->angle_elec); 
  // 克拉克逆变换
  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
	//设置各相的占空比
  bldc_set->dc_a = _constrain(bldc_set->Ua / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_b = _constrain(bldc_set->Ub / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_c = _constrain(bldc_set->Uc / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  //PWM通道赋值
	PWMA_Duty.PWM1_Duty=(u16)(bldc_set->dc_a*2047);
	PWMA_Duty.PWM2_Duty=(u16)(bldc_set->dc_b*2047);
  PWMA_Duty.PWM3_Duty=(u16)(bldc_set->dc_c*2047);
  //PWM状态更新
	UpdatePwm(PWMA, &PWMA_Duty);
}


/**
 * @brief 闭环速度函数
 * @param bldc_set 电机对象结构体
 * @param velocity 目标角速度
*/
void velocityCloseloop (BLDC *bldc_set,float velocity)
{

  TR0 = 0;//关闭定时器0
  //读取定时器值
	bldc_set->Ts=(float)((TH0<<8)|TL0)/35*12*1e-6f;
	TH0 = 0;// 定时器重装值高八位为0
	TL0 = 0;//定时器重装值低八位为0
	TR0 = 1;//开启定时器
	bldc_set->target_velocity=velocity;
  //计算角速度
  bldc_set->current_angle=_normalizeAngle(_AS5600_getAngle_Without_track()*bldc_set->dir);
  bldc_set->diff_angle=bldc_set->current_angle-bldc_set->prev_angle;
  if(fabs(bldc_set->diff_angle) > (0.8f*2*PI) ) bldc_set->current_full_rotations += ( bldc_set->diff_angle > 0 ) ? -1 : 1;
  bldc_set->angular_velocity= ((float)(bldc_set->current_full_rotations - bldc_set->prev_full_rotations) * 2 * PI + (bldc_set->diff_angle) ) / bldc_set->Ts;
  bldc_set->prev_angle=bldc_set->current_angle;
  bldc_set->prev_full_rotations=bldc_set->current_full_rotations;
  bldc_set->filtered_Velocity=lowPassFilter(&bldc_set->velocity_lowPassFilter,bldc_set->angular_velocity,bldc_set->Ts);

  //获取相对角度
  bldc_set->relative_angle=bldc_set->current_angle;
  //获取相对磁角度
	bldc_set->relative_angle_meg=_normalizeAngle(bldc_set->pole_pairs*bldc_set->relative_angle);
   bldc_set->target_voltage=PID_calc(&bldc_set->closeloop_velocity_pid,bldc_set->filtered_Velocity,bldc_set->target_velocity);
  //获取偏差值
   bldc_set->e=bldc_set->target_velocity-bldc_set->filtered_Velocity;
  //根据给定电压值的正负决定相位差角的正负
	if(bldc_set->target_voltage>0)
	{
		bldc_set->angle_elec=_normalizeAngle(bldc_set->relative_angle_meg+bldc_set->zero_electric_angle+PI_2);
	}else{
		bldc_set->angle_elec=_normalizeAngle(bldc_set->relative_angle_meg+bldc_set->zero_electric_angle-PI_2);
	}
  //求相对电角度
  bldc_set->relative_angle_elec=_normalizeAngle(bldc_set->angle_elec-bldc_set->zero_electric_angle);
  //计算Uq电压输出值
  bldc_set->Uq=fabs(bldc_set->target_voltage);
	// 帕克逆变换
  bldc_set->Ualpha = -bldc_set->Uq*sin(bldc_set->angle_elec); 
  bldc_set->Ubeta = bldc_set->Uq*cos(bldc_set->angle_elec); 
  // 克拉克逆变换
  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
	//设置各相的占空比
  bldc_set->dc_a = _constrain(bldc_set->Ua / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_b = _constrain(bldc_set->Ub / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_c = _constrain(bldc_set->Uc / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  //PWM通道赋值
	PWMA_Duty.PWM1_Duty=(u16)(bldc_set->dc_a*2047);
	PWMA_Duty.PWM2_Duty=(u16)(bldc_set->dc_b*2047);
  PWMA_Duty.PWM3_Duty=(u16)(bldc_set->dc_c*2047);
  //PWM状态更新
	UpdatePwm(PWMA, &PWMA_Duty);
}




//void angleCascadeloop (BLDC *bldc_set,float angle)
//{
//  TR0 = 0;
//	bldc_set->Ts=(TH0<<8)|TL0;
//	TH0 = 0;
//	TL0 = 0;
//	TR0 = 1;
//  bldc_set->target_angle=angle;
//  //获取绝对角度
//  bldc_set->absolute_angle=_AS5600_get_absolute_angle()*bldc_set->dir;
//  //获取相对角度
//  bldc_set->relative_angle=_normalizeAngle(_AS5600_getAngle_Without_track()*bldc_set->dir);
//  //获取相对磁角度
//	bldc_set->relative_angle_meg=_normalizeAngle(bldc_set->pole_pairs*bldc_set->relative_angle);
//  bldc_set->target_voltage=PID_calc(&bldc_set->closeloop_angle_pid,bldc_set->absolute_angle,bldc_set->target_angle);
//  bldc_set->e=_normalizeAngle_PI(bldc_set->target_angle-bldc_set->absolute_angle);
//	
//  if(bldc_set->target_voltage>0)
//	{
//		bldc_set->angle_elec=_normalizeAngle(bldc_set->relative_angle_meg+bldc_set->zero_electric_angle+PI_2);
//	}else{
//		bldc_set->angle_elec=_normalizeAngle(bldc_set->relative_angle_meg+bldc_set->zero_electric_angle-PI_2);
//	}
//  bldc_set->relative_angle_elec=_normalizeAngle(bldc_set->angle_elec-bldc_set->zero_electric_angle);
//  //计算Uq电压输出
//  bldc_set->Uq=fabs(bldc_set->target_voltage);
//	// 帕克逆变换
//  bldc_set->Ualpha = -bldc_set->Uq*sin(bldc_set->angle_elec); 
//  bldc_set->Ubeta = bldc_set->Uq*cos(bldc_set->angle_elec); 
//  // 克拉克逆变换
//  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
//  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
//  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
//	//设置各相的占空比
//  bldc_set->dc_a = _constrain(bldc_set->Ua / bldc_set->voltage_power_supply, 0.0f , 1.0f );
//  bldc_set->dc_b = _constrain(bldc_set->Ub / bldc_set->voltage_power_supply, 0.0f , 1.0f );
//  bldc_set->dc_c = _constrain(bldc_set->Uc / bldc_set->voltage_power_supply, 0.0f , 1.0f );
//  //PWM通道赋值
//	PWMA_Duty.PWM1_Duty=(u16)(bldc_set->dc_a*2047);
//	PWMA_Duty.PWM2_Duty=(u16)(bldc_set->dc_b*2047);
//  PWMA_Duty.PWM3_Duty=(u16)(bldc_set->dc_c*2047);
//  //PWM状态更新
//	UpdatePwm(PWMA, &PWMA_Duty);
//}