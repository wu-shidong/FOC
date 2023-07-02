/**
  ********************************************************
  * @file       BLDC.c/h
  * @brief      ������ֱ�����ٵ�����г�ʼ�����������ٶȿ�����
  *             �Ƕȱջ����ٶȱջ����������ػ����ƵȺ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  * V1.0.0     2023-6-26		    ������          �����Ŀ��ʼ��
  * V1.0.1     2023-6-27		    ������          ���PWM��ʼֵ����
  * V1.0.2     2023-6-28		    ������          ����������ڼ���
  * V1.0.3     2023-6-29		    ������          ��ɿ����ٶȻ�
  * V1.0.4     2023-6-30		    ������          ��ɱջ��ǶȻ�
  * V1.0.5     2023-7-01		    ������          ��ɱջ��ٶȻ�
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
//��ʼ��������������
//�궨��ʵ�ֵ�һ��Լ������,��������һ��ֵ�ķ�Χ��
//������˵���ú궨�������Ϊ _constrain�������������� amt��low �� high���ֱ��ʾҪ���Ƶ�ֵ����Сֵ�����ֵ���ú궨���ʵ��ʹ������Ԫ����������� amt �Ƿ�С�� low ����� high���������е�������Сֵ�����߷���ԭֵ��
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define PI 3.14159f
#define PI_2 1.57f
#define PI_4 0.7854f
BLDC bldc_oplp;//��������
BLDC bldc_clslp;//�ջ�����
fp32 clslp_angle_pid[3]={CLSLP_ANGLE_KP,CLSLP_ANGLE_KI,CLSLP_ANGLE_KD};
fp32 clslp_velocity_pid[3]={CLSLP_VELOCITY_KP,CLSLP_VELOCITY_KI,CLSLP_VELOCITY_KD};
// ��һ���Ƕȵ� [0,2PI]
float _normalizeAngle(float angle){
   
	float a = fmod(angle, 2*PI);//ȡ������������ڹ�һ�����г�����ֵ�������֪
  return a >= 0 ? a : (a + 2*PI);  //��Ŀ���������ʽ��condition ? expr1 : expr2 
  //fmod �����������ķ����������ͬ����ˣ��� angle ��ֵΪ����ʱ�������ķ��Ž��� _2PI �ķ����෴��Ҳ����˵����� angle ��ֵС�� 0 �� _2PI ��ֵΪ�������� fmod(angle, _2PI) ��������Ϊ������
  //���磬�� angle ��ֵΪ -PI/2��_2PI ��ֵΪ 2PI ʱ��fmod(angle, _2PI) ������һ������������������£�����ͨ������������������ _2PI �����Ƕȹ�һ���� [0, 2PI] �ķ�Χ�ڣ���ȷ���Ƕȵ�ֵʼ��Ϊ������
}
// ��һ���Ƕȵ� [-PI,PI]
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
 * @brief ���������ʼ������
*/

void bldc_init(BLDC *bldc_init,float voltage_power_supply,float pole_pairs)
{
  //���������Դ��ѹ
	bldc_init->voltage_power_supply=voltage_power_supply;

  //���õ��������
  bldc_init->pole_pairs=pole_pairs;

  //λ���Ϊ0
	bldc_init->shaft_angle=0;
  //���Ƕ�Ϊ0
  bldc_init->zero_electric_angle=4.22;
  // ʹ����ǰ���õ�voltage_power_supply��1/3��ΪUqֵ�����ֵ��ֱ��Ӱ���������
  // ���ֻ������ΪUq = voltage_power_supply/2������ua,ub,uc�ᳬ�������ѹ�޷�
  bldc_init->Uq = bldc_init->voltage_power_supply/3;
  //����������
  bldc_init->dir=-1;
  //�ջ��ǶȻ�pid��ʼ��������
	PID_init(&bldc_init->closeloop_angle_pid,PID_POSITION,clslp_angle_pid,bldc_init->voltage_power_supply/3,voltage_power_supply/20);
	//�ջ��ٶȻ�pid��ʼ��������
  PID_init(&bldc_init->closeloop_velocity_pid,PID_DELTA,clslp_velocity_pid,bldc_init->voltage_power_supply/3,voltage_power_supply/20);
  //���ٶȵ�ͨ�˲���ʼ��
  initLowPassFilter(&bldc_init->velocity_lowPassFilter, 0.035);
}


/**
 * @brief �������ѹ����
 * @param bldc_set ��ˢ����ṹ��
 * @param Uq ��ѹ��ֵ
 * @param Ud һ����0
 * @param angle_el ������Ƕ�
 * 
*/
void setPhaseVoltage(BLDC *bldc_set,float Uq,float Ud, float angle_el)
 {
  //��Ƕȹ�һ��
  bldc_set->angle_elec = _normalizeAngle(angle_el);
  bldc_set->Uq=Uq;
	bldc_set->Ud=Ud;
  // ������任
  bldc_set->Ualpha =  -bldc_set->Uq*sin(bldc_set->angle_elec); 
  bldc_set->Ubeta =   bldc_set->Uq*cos(bldc_set->angle_elec); 
  // ��������任
  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
	//���ø����ռ�ձ�
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
 * @brief �����ٶȺ���
 * @param bldc_set �������ṹ��
 * @param velocity Ŀ���ٶ�
*/

void velocityOpenloop(BLDC *bldc_set,float velocity)
{
  bldc_set->target_velocity=velocity;
  bldc_set->Uq=bldc_set->voltage_power_supply/5;
  //����ʱ������
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

	  //��ȡ���ԽǶȺ���ԽǶ�
  bldc_set->absolute_angle=_AS5600_get_absolute_angle()*bldc_set->dir;
  bldc_set->relative_angle=_normalizeAngle(_AS5600_getAngle_Without_track()*bldc_set->dir);
	bldc_set->relative_angle_meg=_normalizeAngle(bldc_set->pole_pairs*bldc_set->relative_angle);
  // ͨ������ʱ������Ŀ���ٶ���������Ҫת���Ļ�е�Ƕȣ��洢�� shaft_angle �����С��ڴ�֮ǰ������Ҫ����ǶȽ��й�һ������ȷ����ֵ�� 0 �� 2�� ֮�䡣 
	//��Ŀ���ٶ�Ϊ 10 rad/s Ϊ�������ʱ������ 1 �룬����ÿ��ѭ������Ҫ���� 10 * 1 = 10 ���ȵĽǶȱ仯��������ʹ���ת����Ŀ���ٶȡ�
  //���ʱ������ 0.1 �룬��ô��ÿ��ѭ������Ҫ���ӵĽǶȱ仯������ 10 * 0.1 = 1 ���ȣ�����ʵ����ͬ��Ŀ���ٶȡ���ˣ�������ת���Ƕ�ȡ����Ŀ���ٶȺ�ʱ�����ĳ˻���
	bldc_set->shaft_angle = _normalizeAngle(bldc_set->shaft_angle + bldc_set->target_velocity*bldc_set->Ts);
  //�����ǶȲ���һ��
  bldc_set->angle_elec = _normalizeAngle(bldc_set->shaft_angle*bldc_set->pole_pairs+ bldc_set->zero_electric_angle);
	bldc_set->relative_angle_elec=_normalizeAngle(bldc_set->angle_elec-bldc_set->zero_electric_angle);
	bldc_set->e=_normalizeAngle_PI(bldc_set->relative_angle_meg-bldc_set->relative_angle_elec);
  // ������任
  bldc_set->Ualpha =  -bldc_set->Uq*sin(bldc_set->angle_elec); 
  bldc_set->Ubeta =   bldc_set->Uq*cos(bldc_set->angle_elec); 
  // ��������任
  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
	//���ø����ռ�ձ�
  bldc_set->dc_a = _constrain(bldc_set->Ua / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_b = _constrain(bldc_set->Ub / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_c = _constrain(bldc_set->Uc / bldc_set->voltage_power_supply, 0.0f , 1.0f );
	PWMA_Duty.PWM1_Duty=(u16)(bldc_set->dc_a*2047);
	PWMA_Duty.PWM2_Duty=(u16)(bldc_set->dc_b*2047);
  PWMA_Duty.PWM3_Duty=(u16)(bldc_set->dc_c*2047);
	UpdatePwm(PWMA, &PWMA_Duty);
}
/**
 * @brief ������ѹ����
 * @param bldc_set �������ṹ��
 * @param voltage ������ѹ���
*/
void voltageOpenloop (BLDC *bldc_set,int voltage)
{
  // ������������
  TR0 = 0;
	bldc_set->Ts=(TH0<<8)|TL0;
	TH0 = 0;
	TL0 = 0;
	TR0 = 1;
	//��ѹ��ֵ�޷�
	bldc_set->target_voltage=_constrain(voltage,-30000,30000);
  //��ȡ���ԽǶȺ���ԽǶ�
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
  //����Uq��ѹ���
  bldc_set->Uq=(float)(fabs(bldc_set->target_voltage)/30000)*(bldc_set->voltage_power_supply/3);
	// ������任
  bldc_set->Ualpha = -bldc_set->Uq*sin(bldc_set->angle_elec); 
  bldc_set->Ubeta = bldc_set->Uq*cos(bldc_set->angle_elec); 
  // ��������任
  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
	//���ø����ռ�ձ�
  bldc_set->dc_a = _constrain(bldc_set->Ua / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_b = _constrain(bldc_set->Ub / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_c = _constrain(bldc_set->Uc / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  //PWMͨ����ֵ
	PWMA_Duty.PWM1_Duty=(u16)(bldc_set->dc_a*2047);
	PWMA_Duty.PWM2_Duty=(u16)(bldc_set->dc_b*2047);
  PWMA_Duty.PWM3_Duty=(u16)(bldc_set->dc_c*2047);
  //PWM״̬����
	UpdatePwm(PWMA, &PWMA_Duty);
}




/**
 * @brief �ջ��ǶȺ���
 * @param bldc_set �������ṹ��
 * @param velocity Ŀ��Ƕ�
*/
void angleCloseloop (BLDC *bldc_set,float angle)
{
  TR0 = 0;
	bldc_set->Ts=(TH0<<8)|TL0;
	TH0 = 0;
	TL0 = 0;
	TR0 = 1;
  bldc_set->target_angle=angle;
  //��ȡ���ԽǶ�
  bldc_set->absolute_angle=_AS5600_get_absolute_angle()*bldc_set->dir;
  //��ȡ��ԽǶ�
  bldc_set->relative_angle=_normalizeAngle(_AS5600_getAngle_Without_track()*bldc_set->dir);
  //��ȡ��ԴŽǶ�
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
  //����Uq��ѹ���
  bldc_set->Uq=fabs(bldc_set->target_voltage);
	// ������任
  bldc_set->Ualpha = -bldc_set->Uq*sin(bldc_set->angle_elec); 
  bldc_set->Ubeta = bldc_set->Uq*cos(bldc_set->angle_elec); 
  // ��������任
  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
	//���ø����ռ�ձ�
  bldc_set->dc_a = _constrain(bldc_set->Ua / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_b = _constrain(bldc_set->Ub / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_c = _constrain(bldc_set->Uc / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  //PWMͨ����ֵ
	PWMA_Duty.PWM1_Duty=(u16)(bldc_set->dc_a*2047);
	PWMA_Duty.PWM2_Duty=(u16)(bldc_set->dc_b*2047);
  PWMA_Duty.PWM3_Duty=(u16)(bldc_set->dc_c*2047);
  //PWM״̬����
	UpdatePwm(PWMA, &PWMA_Duty);
}


/**
 * @brief �ջ��ٶȺ���
 * @param bldc_set �������ṹ��
 * @param velocity Ŀ����ٶ�
*/
void velocityCloseloop (BLDC *bldc_set,float velocity)
{

  TR0 = 0;//�رն�ʱ��0
  //��ȡ��ʱ��ֵ
	bldc_set->Ts=(float)((TH0<<8)|TL0)/35*12*1e-6f;
	TH0 = 0;// ��ʱ����װֵ�߰�λΪ0
	TL0 = 0;//��ʱ����װֵ�Ͱ�λΪ0
	TR0 = 1;//������ʱ��
	bldc_set->target_velocity=velocity;
  //������ٶ�
  bldc_set->current_angle=_normalizeAngle(_AS5600_getAngle_Without_track()*bldc_set->dir);
  bldc_set->diff_angle=bldc_set->current_angle-bldc_set->prev_angle;
  if(fabs(bldc_set->diff_angle) > (0.8f*2*PI) ) bldc_set->current_full_rotations += ( bldc_set->diff_angle > 0 ) ? -1 : 1;
  bldc_set->angular_velocity= ((float)(bldc_set->current_full_rotations - bldc_set->prev_full_rotations) * 2 * PI + (bldc_set->diff_angle) ) / bldc_set->Ts;
  bldc_set->prev_angle=bldc_set->current_angle;
  bldc_set->prev_full_rotations=bldc_set->current_full_rotations;
  bldc_set->filtered_Velocity=lowPassFilter(&bldc_set->velocity_lowPassFilter,bldc_set->angular_velocity,bldc_set->Ts);

  //��ȡ��ԽǶ�
  bldc_set->relative_angle=bldc_set->current_angle;
  //��ȡ��ԴŽǶ�
	bldc_set->relative_angle_meg=_normalizeAngle(bldc_set->pole_pairs*bldc_set->relative_angle);
   bldc_set->target_voltage=PID_calc(&bldc_set->closeloop_velocity_pid,bldc_set->filtered_Velocity,bldc_set->target_velocity);
  //��ȡƫ��ֵ
   bldc_set->e=bldc_set->target_velocity-bldc_set->filtered_Velocity;
  //���ݸ�����ѹֵ������������λ��ǵ�����
	if(bldc_set->target_voltage>0)
	{
		bldc_set->angle_elec=_normalizeAngle(bldc_set->relative_angle_meg+bldc_set->zero_electric_angle+PI_2);
	}else{
		bldc_set->angle_elec=_normalizeAngle(bldc_set->relative_angle_meg+bldc_set->zero_electric_angle-PI_2);
	}
  //����Ե�Ƕ�
  bldc_set->relative_angle_elec=_normalizeAngle(bldc_set->angle_elec-bldc_set->zero_electric_angle);
  //����Uq��ѹ���ֵ
  bldc_set->Uq=fabs(bldc_set->target_voltage);
	// ������任
  bldc_set->Ualpha = -bldc_set->Uq*sin(bldc_set->angle_elec); 
  bldc_set->Ubeta = bldc_set->Uq*cos(bldc_set->angle_elec); 
  // ��������任
  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
	//���ø����ռ�ձ�
  bldc_set->dc_a = _constrain(bldc_set->Ua / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_b = _constrain(bldc_set->Ub / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  bldc_set->dc_c = _constrain(bldc_set->Uc / bldc_set->voltage_power_supply, 0.0f , 1.0f );
  //PWMͨ����ֵ
	PWMA_Duty.PWM1_Duty=(u16)(bldc_set->dc_a*2047);
	PWMA_Duty.PWM2_Duty=(u16)(bldc_set->dc_b*2047);
  PWMA_Duty.PWM3_Duty=(u16)(bldc_set->dc_c*2047);
  //PWM״̬����
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
//  //��ȡ���ԽǶ�
//  bldc_set->absolute_angle=_AS5600_get_absolute_angle()*bldc_set->dir;
//  //��ȡ��ԽǶ�
//  bldc_set->relative_angle=_normalizeAngle(_AS5600_getAngle_Without_track()*bldc_set->dir);
//  //��ȡ��ԴŽǶ�
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
//  //����Uq��ѹ���
//  bldc_set->Uq=fabs(bldc_set->target_voltage);
//	// ������任
//  bldc_set->Ualpha = -bldc_set->Uq*sin(bldc_set->angle_elec); 
//  bldc_set->Ubeta = bldc_set->Uq*cos(bldc_set->angle_elec); 
//  // ��������任
//  bldc_set->Ua = bldc_set->Ualpha + bldc_set->voltage_power_supply/2;
//  bldc_set->Ub = (sqrt(3)*bldc_set->Ubeta-bldc_set->Ualpha)/2 + bldc_set->voltage_power_supply/2;
//  bldc_set->Uc = (-bldc_set->Ualpha-sqrt(3)*bldc_set->Ubeta)/2 + bldc_set->voltage_power_supply/2;
//	//���ø����ռ�ձ�
//  bldc_set->dc_a = _constrain(bldc_set->Ua / bldc_set->voltage_power_supply, 0.0f , 1.0f );
//  bldc_set->dc_b = _constrain(bldc_set->Ub / bldc_set->voltage_power_supply, 0.0f , 1.0f );
//  bldc_set->dc_c = _constrain(bldc_set->Uc / bldc_set->voltage_power_supply, 0.0f , 1.0f );
//  //PWMͨ����ֵ
//	PWMA_Duty.PWM1_Duty=(u16)(bldc_set->dc_a*2047);
//	PWMA_Duty.PWM2_Duty=(u16)(bldc_set->dc_b*2047);
//  PWMA_Duty.PWM3_Duty=(u16)(bldc_set->dc_c*2047);
//  //PWM״̬����
//	UpdatePwm(PWMA, &PWMA_Duty);
//}