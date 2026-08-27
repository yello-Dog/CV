#include "VMC.h"
#include "arm_math.h"
#include "fivebar_kinematics_func.h"
#include "fivebar_vmc_torque_func.h"
#include "fivebar_velocity_func.h"


const INS_t* INS_Data_Handle_VMC;

void VMC_init(vmc_leg_t *vmc)//给杆长赋值
{
	INS_Data_Handle_VMC = INS_GetData();
	// 其实下面这些，都在matlab里面。要改的话还得在matlab里面改
	vmc->l5=0.011f;//AE长度 //单位为m
	vmc->l1=0.075f;//单位为m
	vmc->l2=0.14f;//单位为m
	vmc->l3=0.14f;//单位为m
	vmc->l4=0.075f;//单位为m
}


float dxBD,dyBD,a,h;


void fivebar_theta_func(float phi0,float dphi0,float body_phi,float dbody_phi,float* theta,float* dtheta)
{
	
	*theta = -body_phi+phi0-pi/2;
	*dtheta = -dbody_phi+dphi0;
}

// 计算pai0和L0 dpai0和dL0 theta和dtheta
void VMC_calc_1_left(vmc_leg_t *vmc)
{
	// 计算pai0和L0
	fivebar_kinematics_func(vmc->phi1, vmc->phi4, &vmc->L0, &vmc->phi0);
	// 计算dpai0和dL0
	fivebar_velocity_func(vmc->phi1,vmc->phi4, vmc->d_phi1,vmc->d_phi4,
																						 &vmc->d_L0,&vmc->d_phi0);
	// 计算theta和dtheta -- 这里面假设gyrp[2]是pitch的角速度
  fivebar_theta_func(vmc->phi0 ,vmc->d_phi0,-1*INS_Data_Handle_VMC->Pitch,-1*INS_Data_Handle_VMC->Gyro[0],&vmc->theta,&vmc->d_theta);
	
}
void VMC_calc_1_right(vmc_leg_t *vmc)
{
	// 计算pai0和L0
	fivebar_kinematics_func(vmc->phi1, vmc->phi4, &vmc->L0, &vmc->phi0);
	// 计算dpai0和dL0
	fivebar_velocity_func(vmc->phi1,vmc->phi4, vmc->d_phi1,vmc->d_phi4,
																						 &vmc->d_L0,&vmc->d_phi0);
	// 计算theta和dtheta -- 这里面假设gyrp[2]是pitch的角速度
  fivebar_theta_func(vmc->phi0 ,vmc->d_phi0,-1*INS_Data_Handle_VMC->Pitch,-1*INS_Data_Handle_VMC->Gyro[0],&vmc->theta,&vmc->d_theta);
	
}



// 输入TP和F，输出T1 T4，计算五连杆动力学映射
void VMC_calc_2(vmc_leg_t *vmc)
{
	fivebar_vmc_torque_func(vmc->phi1,vmc->phi4,vmc->F0,vmc->Tp,&vmc->torque_set[0],&vmc->torque_set[1]);
}


// 粗略估计，5N保守了。离地检测属于新功能，到时候再调
uint8_t ground_detectionL(vmc_leg_t *vmc,const INS_t *ins)
{
	// +vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0
	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+6.0f;//腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度

	if(vmc->FN<5.0f)
	{//离地了
	  return 1;
	}
	else
	{
	  return 0;	
	}
}

uint8_t ground_detectionR(vmc_leg_t *vmc,const INS_t *ins)
{
	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+6.0f;//腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
	if(vmc->FN<5.0f)
	{//离地了

	  return 1;
	}
	else
	{
	  return 0;	
	}
}

uint8_t fall_detection()
{
	if(INS_Data_Handle_VMC->Pitch<-0.8 || INS_Data_Handle_VMC->Pitch>0.8)
	{
		return 1;	
	}
	else
	{
		return 0;
	}
}















