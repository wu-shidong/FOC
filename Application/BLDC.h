#ifndef __BLDC_H
#define __BLDC_H
#include "user_pid.h"
#include "user_filter.h"

#define CLSLP_ANGLE_KP 4
#define CLSLP_ANGLE_KI 0.0001
#define CLSLP_ANGLE_KD 2


#define CLSLP_VELOCITY_KP 0.5
#define CLSLP_VELOCITY_KI 0
#define CLSLP_VELOCITY_KD 1



//#define CASCADE_ANGLE_KP 4
//#define CASCADE_ANGLE_KI 0.0001
//#define CASCADE_ANGLE_KD 2


//#define CASCADE_VELOCITY_KP 0.7
//#define CASCADE_VELOCITY_KI 0.005
//#define CASCADE_VELOCITY_KD 1
typedef struct
{
	/*���ԭʼ����*/
	pid_type_def closeloop_angle_pid;//�ջ��ǶȻ�PID
	pid_type_def closeloop_velocity_pid;//�ջ��ٶȻ�PID
	pid_type_def cascade_angle_pid;//�����ǶȻ�PID
	pid_type_def cascade_velocity_pid;//�����ٶȻ�PID
	LowPassFilter velocity_lowPassFilter;
	float voltage_power_supply;//�����ѹ
	unsigned char pole_pairs;	//���������
	int dir; //��������ת����	
	float zero_electric_angle;//�����λ��Ƕ�ֵ

	/*���˶�ѧ��������*/
	float target_velocity;//Ŀ���ٶ�
	float target_voltage;//Ŀ���ѹ
	float target_angle;//Ŀ��Ƕ�
	float relative_angle;//��ԽǶȣ�0~360��
	float absolute_angle;//���ԽǶ�
	float relative_angle_meg; //��ԴŽǶ�
	float angle_elec; //������Ƕ�	
	float relative_angle_elec;// ��Ե�Ƕ�
	float e;//ƫ��
	float shaft_angle;//�����ٶ�λ���

	/*�ٶȲ��������ͨ�˲�*/
	unsigned int T_cnt;
	float Ts;//��������ʱ��
	float prev_angle;//ǰһ�β����ĽǶ�����
	float current_angle ;//��һ�β����ĽǶ�����
	float diff_angle;//�ǶȲ�
	float angular_velocity;//ʵ�ʲ����Ľ��ٶ�
	float filtered_Velocity;//����һ�׵�ͨ�˲���Ľ��ٶ�
	float current_full_rotations;//
	float prev_full_rotations;//
	
	/*���˶�ѧ��������*/
	float Uq;//������任��ѹq
	float Ud;//������任��ѹd
	float Ualpha;//������任��ѹalpha
	float Ubeta;//������任��ѹbeta
	float Ua;//��������任a���ѹ
	float Ub;//��������任b���ѹ
	float Uc;//��������任c���ѹ
	float dc_a;//a��ռ�ձȣ�duty cycle)
	float dc_b;//b��ռ�ձȣ�duty cycle)
	float dc_c;//c��ռ�ձȣ�duty cycle)
}BLDC;
extern void bldc_init(BLDC *bldc_init,float voltage_power_supply,float pole_pairs);
extern void velocityOpenloop(BLDC *bldc_set,float velocity);
extern void angleCloseloop (BLDC *bldc_set,float angle);
extern void bldc_clslp_init(BLDC *bldc_set);
extern void voltageOpenloop (BLDC *bldc_set,int voltage);
extern void velocityCloseloop (BLDC *bldc_set,float velocity);
extern BLDC bldc_oplp;//��������
extern BLDC bldc_clslp;//�ջ�����
#endif
