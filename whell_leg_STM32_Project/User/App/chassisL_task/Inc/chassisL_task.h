#ifndef __CHASSISL_TASK_H
#define __CHASSISL_TASK_H

#include "main.h"
#include "dm4310_drv.h"

#define ROLL_PID_KP 100.0f
#define ROLL_PID_KI 0.0f //不用积分项
#define ROLL_PID_KD 5.0f
#define ROLL_PID_MAX_OUT  10.0f
#define ROLL_PID_MAX_IOUT 0.0f

#define TP_PID_KP 10.0f
#define TP_PID_KI 0.0f //不用积分项
#define TP_PID_KD 0.1f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP_P 2.0f
#define TURN_PID_KI_P 0.0f //不用积分项
#define TURN_PID_KD_P 0.2f

#define TURN_PID_KP_V 2.0f
#define TURN_PID_KI_V 0.0f //不用积分项
#define TURN_PID_KD_V 0.2f

#define TURN_PID_MAX_OUT  2.0f//轮毂电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f


typedef struct
{
  Joint_Motor_t joint_motor[4];
  Wheel_Motor_t wheel_motor[2];
	
	float v_set;//期望速度，单位是m/s
	float x_set;//期望位置，单位是m
	float turn_set;//期望yaw轴弧度
	float roll_set;	//期望roll轴弧度
	float leg_set;//期望腿长，单位是m
	float left_leg_set;
	float right_leg_set;

	float v_filter;//滤波后的车体速度，单位是m/s
	float x_filter;//滤波后的车体位置，单位是m
	//=============================================
	float L_v_filter;//滤波后的车体速度，单位是m/s
	float L_x_filter;//滤波后的车体位置，单位是m
	
	float R_v_filter;//滤波后的车体速度，单位是m/s
	float R_x_filter;//滤波后的车体位置，单位是m
	//=============================================
	float myPith;
	float myPithDot;
	float roll;
	
	float total_yaw;
	float yaw_velo_set;
	//float theta_err;//两腿夹角误差
	float pai0_err;//两腿我觉得更加合适定义夹角误差的变量	
	
	float turn_T;//yaw轴补偿
	float leg_tp;//防劈叉补偿
	
	uint8_t start_flag;//启动标志
	
	uint8_t attampt_jump_flag;//准备跳跃标志
	uint8_t jump_flag;//跳跃标志
	uint8_t F0_lock_flag;
	
	uint8_t recover_flag;//一种情况下的倒地自起标志

	
} chassis_t;

extern void LQR_Get_K_Given_Length(float lenth);
extern void mySaturate(float *in,float min,float max);
extern void ChassisL_task(void);

#endif

