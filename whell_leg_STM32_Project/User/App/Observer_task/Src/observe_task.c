/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     该任务是对机体运动速度估计，用于抑制打滑
	* 					 原理来源于https://zhuanlan.zhihu.com/p/689921165
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "observe_task.h"
#include "cmsis_os.h"
#include "VMC.h"
#include "chassisL_task.h"
#include "ins_task.h"
#include "DebugConfig.h"

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.1f, 0.0f, 
                           0.0f, 0.1f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                            0.0f,  100.0f}; 	
														
float vaEstimateKF_K[4];
													 
float L_vel;
														
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
														 															 
const INS_t* INS_Data_Handle_Observe;		
extern chassis_t chassis_move;																 															 
																 
extern vmc_leg_t VMC_right;			
extern vmc_leg_t VMC_left;	

// 0是融合的速度，1是融合的加速度
static float vel_acc[2]; 
																 
uint32_t OBSERVE_TIME=3;//任务周期是3ms			
	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	static float aver_v=0.0f;																 
void 	Observe_task(void)
{
	INS_Data_Handle_Observe = INS_GetData();
	while(INS_Data_Handle_Observe->ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}

		
	xvEstimateKF_Init(&vaEstimateKF);
	
  while(1)
	{  
		
		// 模型预测部分
		wr = chassis_move.wheel_motor[0].para.vel+INS_Data_Handle_Observe->Gyro[0]+VMC_right.d_phi0;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vrb = wr*0.0603f+VMC_right.L0*VMC_right.d_theta*arm_cos_f32(VMC_right.theta)+VMC_right.d_L0*arm_sin_f32(VMC_right.theta);//机体b系的速度
		
		wl= -chassis_move.wheel_motor[1].para.vel+INS_Data_Handle_Observe->Gyro[0]+VMC_left.d_phi0;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vlb=wl*0.0603f+VMC_left.L0*VMC_left.d_theta*arm_cos_f32(VMC_left.theta)+VMC_left.d_L0*arm_sin_f32(VMC_left.theta);//机体b系的速度
		
		aver_v=(vrb+vlb)/2.0f;//取平均
		
    xvEstimateKF_Update(&vaEstimateKF,INS_Data_Handle_Observe->Y_MotionAccel_n,aver_v);
		
		//原地自转的过程中v_filter和x_filter应该都是为0
		#if KARMAN_OFF
		L_vel = -chassis_move.wheel_motor[1].para.vel;
		chassis_move.L_v_filter = wl*0.0603f;//得到卡尔曼滤波后的速度
		chassis_move.R_v_filter = wr*0.0603f;//得到卡尔曼滤波后的速度
		chassis_move.v_filter = (wl+wr)/2.0f*0.0603f;
		chassis_move.L_x_filter=chassis_move.L_x_filter+chassis_move.L_v_filter*((float)OBSERVE_TIME/1000.0f);
		chassis_move.R_x_filter=chassis_move.R_x_filter+chassis_move.R_v_filter*((float)OBSERVE_TIME/1000.0f);
		
		#else
		chassis_move.v_filter=vel_acc[0];//得到卡尔曼滤波后的速度
		chassis_move.x_filter=chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		#endif
		osDelay(OBSERVE_TIME);
	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}



void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

const float *Observer_Get_xdot_xdotdot(void)
{
    return vel_acc;
}
