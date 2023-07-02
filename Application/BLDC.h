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
	/*电机原始参数*/
	pid_type_def closeloop_angle_pid;//闭环角度环PID
	pid_type_def closeloop_velocity_pid;//闭环速度环PID
	pid_type_def cascade_angle_pid;//串级角度环PID
	pid_type_def cascade_velocity_pid;//串级速度环PID
	LowPassFilter velocity_lowPassFilter;
	unsigned char control_mode; //控制模式
	unsigned char prev_control_mode; //上一次控制模式
	float voltage_power_supply;//供电电压
	unsigned char pole_pairs;	//电机极对数
	int dir; //编码器旋转方向	
	float zero_electric_angle;//电机零位电角度值
	unsigned long ID;

	/*逆运动学解算数据*/
	float target_velocity;//目标速度
	float target_voltage;//目标电压
	float target_angle;//目标角度
	unsigned int encode;//编码器值
	float relative_angle;//相对角度（0~360）
	float absolute_angle;//绝对角度
	float relative_angle_meg; //相对磁角度
	float angle_elec; //给定电角度	
	float relative_angle_elec;// 相对电角度
	float e;//偏差
	float shaft_angle;//开环速度位差角

	/*速度测量及其低通滤波*/
	unsigned int int_s;//计算周期时间整数形式
	float Ts;//计算周期时间浮点数形式
	float prev_angle;//前一次测量的角度数据
	float current_angle ;//这一次测量的角度数据
	float diff_angle;//角度差
	float angular_velocity;//实际测量的角速度
	float filtered_Velocity;//经过一阶低通滤波后的角速度
	float current_full_rotations;//
	float prev_full_rotations;//
	
	/*正运动学解算数据*/
	float Uq;//帕克逆变换电压q
	float Ud;//帕克逆变换电压d
	float Ualpha;//帕克逆变换电压alpha
	float Ubeta;//帕克逆变换电压beta
	float Ua;//克拉克逆变换a相电压
	float Ub;//克拉克逆变换b相电压
	float Uc;//克拉克逆变换c相电压
	float dc_a;//a相占空比（duty cycle)
	float dc_b;//b相占空比（duty cycle)
	float dc_c;//c相占空比（duty cycle)
}BLDC;
typedef enum
{
  bldc_Zero_force = 0x30,//电机无力状态
  velocity_Openloop = 0x31,//开环速度模式
  voltage_Openloop = 0x32,//开环电压模式
  angle_Closeloop = 0x33,//角度闭环模式
  velocity_Closeloop = 0x34,//速度闭环模式
  angle_Cascadeloop = 0x35,//角度串级闭环模式
  current_Closeloop = 0x36,//电流闭环模式
}foc_control_mode_e;
extern void bldc_init(BLDC *bldc_init,float voltage_power_supply,float pole_pairs,unsigned long ID);
extern void velocityOpenloop(BLDC *bldc_set);
extern void angleCloseloop (BLDC *bldc_set);
extern void bldc_clslp_init(BLDC *bldc_set);
extern void voltageOpenloop (BLDC *bldc_set);
extern void velocityCloseloop (BLDC *bldc_set);
extern void BLDC_control(BLDC *bldc_set);
extern BLDC bldc_oplp;//开环控制
extern BLDC bldc_clslp;//闭环控制
#endif
