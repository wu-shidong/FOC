#ifndef __BLDC_H
#define __BLDC_H
#include "user_pid.h"
#include "user_filter.h"

#define CLSLP_ANGLE_KP 4
#define CLSLP_ANGLE_KI 0.0001
#define CLSLP_ANGLE_KD 2


#define CLSLP_VELOCITY_KP 0.4
#define CLSLP_VELOCITY_KI 0.001
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
	unsigned char control_mode; //����ģʽ
	unsigned char prev_control_mode; //��һ�ο���ģʽ
	float voltage_power_supply;//�����ѹ
	unsigned char pole_pairs;	//���������
	int dir; //��������ת����	
	float zero_electric_angle;//�����λ��Ƕ�ֵ
	unsigned long ID;

	/*���˶�ѧ��������*/
	float target_velocity;//Ŀ���ٶ�
	float target_voltage;//Ŀ���ѹ
	float target_angle;//Ŀ��Ƕ�
	unsigned int encode;//������ֵ
	float relative_angle;//��ԽǶȣ�0~360��
	float absolute_angle;//���ԽǶ�
	float relative_angle_meg; //��ԴŽǶ�
	float angle_elec; //������Ƕ�	
	float relative_angle_elec;// ��Ե�Ƕ�
	float e;//ƫ��
	float shaft_angle;//�����ٶ�λ���

	/*�ٶȲ��������ͨ�˲�*/
	unsigned int int_s;//��������ʱ��������ʽ
	float Ts;//��������ʱ�両������ʽ
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
typedef enum
{
  bldc_Zero_force = 0x30,//�������״̬
  velocity_Openloop = 0x31,//�����ٶ�ģʽ
  voltage_Openloop = 0x32,//������ѹģʽ
  angle_Closeloop = 0x33,//�Ƕȱջ�ģʽ
  velocity_Closeloop = 0x34,//�ٶȱջ�ģʽ
  angle_Cascadeloop = 0x35,//�Ƕȴ����ջ�ģʽ
  current_Closeloop = 0x36,//�����ջ�ģʽ
}foc_control_mode_e;
extern void bldc_init(BLDC *bldc_init,float voltage_power_supply,float pole_pairs,unsigned long ID);
extern void velocityOpenloop(BLDC *bldc_set);
extern void angleCloseloop (BLDC *bldc_set);
extern void bldc_clslp_init(BLDC *bldc_set);
extern void voltageOpenloop (BLDC *bldc_set);
extern void velocityCloseloop (BLDC *bldc_set);
extern void BLDC_control(BLDC *bldc_set);
extern BLDC bldc_oplp;//��������
extern BLDC bldc_clslp;//�ջ�����
#endif
