#ifndef __VMC_CALC_H
#define __VMC_CALC_H

#include "main.h"
#include "ins_task.h"

#define pi 3.1415926f
#define LEG_PD_KP 450
#define ZETA 0.25
#define LEG_PID_MAX_OUT  90.0f //90牛
#define LEG_PID_MAX_IOUT 0.0f

typedef struct
{
	/*左右两腿的公共参数，固定不变*/
	float l5;//AE长度 //单位为m
	float	l1;//单位为m
	float l2;//单位为m
	float l3;//单位为m
	float l4;//单位为m
	
	float phi0,d_phi0;//现在C点角度phi0的变换率
		
	float phi1,phi4;
	float d_phi1,d_phi4;
	
	float torque_set[2];

	float F0;
	float Tp;
	
	float theta,d_theta;

	float L0,d_L0;//L0的一阶导数

	float FN;//支持力
	
	uint8_t first_flag;
	uint8_t leg_flag;//腿长完成标志
} vmc_leg_t;

extern void VMC_init(vmc_leg_t *vmc);//给杆长赋值
extern uint8_t ground_detectionL(vmc_leg_t *vmc,const INS_t *ins);
extern uint8_t ground_detectionR(vmc_leg_t *vmc,const INS_t *ins);
extern void VMC_calc_1_left(vmc_leg_t *vmc);
extern void VMC_calc_1_right(vmc_leg_t *vmc);
extern void VMC_calc_2(vmc_leg_t *vmc);
extern uint8_t fall_detection();
#endif
