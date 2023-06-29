#include "BLDC.h"
#include <MATH.H>
#include <stdio.h>
#include "user_pid.h"
#include "bsp_pwm.h"
//��ʼ��������������
//�궨��ʵ�ֵ�һ��Լ������,��������һ��ֵ�ķ�Χ��
//������˵���ú궨�������Ϊ _constrain�������������� amt��low �� high���ֱ��ʾҪ���Ƶ�ֵ����Сֵ�����ֵ���ú궨���ʵ��ʹ������Ԫ����������� amt �Ƿ�С�� low ����� high���������е�������Сֵ�����߷���ԭֵ��
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define PI 3.14159f
u16 Ts;
BLDC bldc_oplp;//��������

void bldc_openloop_init(BLDC *bldc_init,float voltage_power_supply,float pole_pairs)
{
  //�����Դ��ѹ
	bldc_init->voltage_power_supply=voltage_power_supply;

  //���������
  bldc_init->pole_pairs=pole_pairs;

  //λ���Ϊ0
	bldc_init->shaft_angle=0;
  // ʹ����ǰ���õ�voltage_power_supply��1/3��ΪUqֵ�����ֵ��ֱ��Ӱ���������
  // ���ֻ������ΪUq = voltage_power_supply/2������ua,ub,uc�ᳬ�������ѹ�޷�
  bldc_init->Uq = bldc_init->voltage_power_supply/5;
}




// ��һ���Ƕȵ� [0,2PI]
float _normalizeAngle(float angle){
   
	float a = fmod(angle, 2*PI);//ȡ������������ڹ�һ�����г�����ֵ�������֪
  return a >= 0 ? a : (a + 2*PI);  //��Ŀ���������ʽ��condition ? expr1 : expr2 
  
  //fmod �����������ķ����������ͬ����ˣ��� angle ��ֵΪ����ʱ�������ķ��Ž��� _2PI �ķ����෴��Ҳ����˵����� angle ��ֵС�� 0 �� _2PI ��ֵΪ�������� fmod(angle, _2PI) ��������Ϊ������
  //���磬�� angle ��ֵΪ -PI/2��_2PI ��ֵΪ 2PI ʱ��fmod(angle, _2PI) ������һ������������������£�����ͨ������������������ _2PI �����Ƕȹ�һ���� [0, 2PI] �ķ�Χ�ڣ���ȷ���Ƕȵ�ֵʼ��Ϊ������
}




void velocityOpenloop(BLDC *bldc_set)
{
  bldc_set->target_velocity=10;
	TR0 = 0;
	Ts=(TH0<<8)|TL0;
	TH0 = 0;
	TL0 = 0;
	TR0 = 1;
  // ͨ������ʱ������Ŀ���ٶ���������Ҫת���Ļ�е�Ƕȣ��洢�� shaft_angle �����С��ڴ�֮ǰ������Ҫ����ǶȽ��й�һ������ȷ����ֵ�� 0 �� 2�� ֮�䡣 
	//��Ŀ���ٶ�Ϊ 10 rad/s Ϊ�������ʱ������ 1 �룬����ÿ��ѭ������Ҫ���� 10 * 1 = 10 ���ȵĽǶȱ仯��������ʹ���ת����Ŀ���ٶȡ�
  //���ʱ������ 0.1 �룬��ô��ÿ��ѭ������Ҫ���ӵĽǶȱ仯������ 10 * 0.1 = 1 ���ȣ�����ʵ����ͬ��Ŀ���ٶȡ���ˣ�������ת���Ƕ�ȡ����Ŀ���ٶȺ�ʱ�����ĳ˻���
//  bldc_set->shaft_angle = _normalizeAngle(bldc_set->shaft_angle + bldc_set->target_velocity*bldc_set->oplp_timestamp);
	bldc_set->shaft_angle = _normalizeAngle(bldc_set->shaft_angle + bldc_set->target_velocity*Ts/35*12*1e-6f);
  //�����Ƕ�
  bldc_set->angle_elec=bldc_set->shaft_angle*bldc_set->pole_pairs;
  //��Ƕȹ�һ��
  bldc_set->angle_elec = _normalizeAngle(bldc_set->angle_elec + bldc_set->zero_electric_angle);
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


