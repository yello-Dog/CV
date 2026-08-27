#include "chassisR_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "VMC.h"
#include "pid.h"
#include "arm_math.h"
#include "DebugConfig.h"
#include "buzzer.h"
#include "yaw_gain_calc.h"
/**
 
 应该先获得当前状态和目标状态（定义数据结构体）
 -IMU数据
 -各个电机的角度，角速度数据
 
 --》然后五连杆运动学函数：输入pai1 pai4，解算出当前腿长和角度，以及腿长变化速度，角度变化速度
 --》然后五连杆动力学函数：输入五连杆等效的TP F，输出T1 T4
 
 --》写一个输入腿长，返回K矩阵数值的程序。
 --》然后开始写LQR输出：输入腿长，状态，目标状态，返回T TP
 --》然后写放劈叉的控制器，把控制量加到左右TP上
 
 --》开始算F：输入期望腿长，KP，阻尼系数，输出F（本质是一个PD控制器，但是经过了一层映射）
 
 把TP和F输入五连杆动力学里面，解算出T1 T4
 
 ### 然后用CAN把T1 T4 T的扭矩打包发送出去
 
**/

// 总的数据包
extern chassis_t chassis_move;

// IMU原始数据只读接口.const指针意思是不能通过这个指针修改指向的数据，但是指针本身是可以变的
const INS_t* INS_DATA_HANDLELER_R;

// 右腿的VMC对象
vmc_leg_t VMC_right;
// 右腿的腿长PID控制器对象
PidTypeDef LegR_Pid;

extern float Poly_Coefficient_Move[12][4];

// 右腿离地标志
uint8_t right_flag;
extern uint8_t left_flag;

float LQR_K_R[12]={ 
   0,0,0,0,0,0,
   0,0,0,0,0,0
};
extern float LQR_Cubic_Spline_calc(float *coe,float len);

void LQR_Get_K_Given_Length_R(float lenth)
{
		for(int i=0;i<12;i++)
	{
		LQR_K_R[i]=LQR_Cubic_Spline_calc(&Poly_Coefficient_Move[i][0],lenth);	
	}
}

#if SIMPLE_CHAIR_MODLEL
float x_out,theta_out,phi_out;
float x_dot_out,theta_dot_out,phi_dot_out;
#endif
float R_output; 
float legr_pid[3];
void ChassisR_init()
{

	VMC_init(&VMC_right);//给杆长赋值
	
	// FDCAN1: right side
	// joint_motor[0]: right leg phi1, tx id = 6, rx feedback id = 3
	// joint_motor[1]: right leg phi4, tx id = 8, rx feedback id = 4
	// wheel_motor[0]: right wheel,    tx id = 1, rx feedback id = 0
	joint_motor_init(&chassis_move.joint_motor[0],6,MIT_MODE);//发送id为6
	joint_motor_init(&chassis_move.joint_motor[1],8,MIT_MODE);//发送id为8
	
	wheel_motor_init(&chassis_move.wheel_motor[0],1,MIT_MODE);//发送id为1
	
	//腿长PD控制器初始化
	legr_pid[0] = LEG_PD_KP;
	FFF_Damper_Kp_calc_KpKd(11.f,ZETA,(float)LEG_PD_KP,&legr_pid[2]);
	legr_pid[1] = 0;
	PID_init(&LegR_Pid, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);
	
	// 发送十次，主要是怕没配置上
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis_move.joint_motor[1].para.id,chassis_move.joint_motor[1].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis_move.joint_motor[0].para.id,chassis_move.joint_motor[0].mode);
	  osDelay(1);
	}

	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis_move.wheel_motor[0].para.id,chassis_move.wheel_motor[0].mode);//右边轮毂电机
	  osDelay(1);
	}
}



/* 
 这个左右两边的电机数据必须得写两个函数来更新，还不能独立更新，因为两边用的两个CAN外设
 另外改一下，因为INS是只读的，就用只读的接口来读取
*/
void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc)
{
	// 为左边的VMC解算传数据
  vmc->phi1=pi + chassis->joint_motor[0].para.pos;
	vmc->phi4=chassis->joint_motor[1].para.pos;
		
	vmc->d_phi1 = chassis->joint_motor[0].para.vel;
	vmc->d_phi4 = chassis->joint_motor[1].para.vel;
	// IMU的pitch roll yaw数据和vmc解算的theta（我是pai0）数据都在L那边更新
	
}

extern uint8_t self_lock_flag;
void chassisR_control_loop()
{
	// VMC相关计算----待补充
	VMC_calc_1_right(&VMC_right);
	
		// 更新K
	LQR_Get_K_Given_Length_R(VMC_right.L0);
	
	
	// 下面用更新完的K来计算T TP
		// LQR计算左侧轮毂电机输出力矩
	#if (REMOVE_X_IN_LQR == 1)
	chassis_move.wheel_motor[0].wheel_T=-(LQR_K_R[0]*(VMC_right.theta-0.0f)
																+LQR_K_R[1]*(VMC_right.d_theta-0.0f)
																+LQR_K_R[2]*(0)
																+LQR_K_R[3]*(chassis_move.v_filter-chassis_move.v_set)
																+LQR_K_R[4]*(chassis_move.myPith-0.0f)
																+LQR_K_R[5]*(chassis_move.myPithDot-0.0f));
	#else
	chassis_move.wheel_motor[0].wheel_T=-(LQR_K_R[0]*(VMC_right.theta-0.0f)
																	+LQR_K_R[1]*(VMC_right.d_theta-0.0f)
																	+LQR_K_R[2]*(chassis_move.x_filter-chassis_move.x_set)
																	+LQR_K_R[3]*(chassis_move.v_filter-chassis_move.v_set)
																	+LQR_K_R[4]*(chassis_move.myPith-0.0f)
																	+LQR_K_R[5]*(chassis_move.myPithDot-0.0f));
	#endif
	#if SIMPLE_CHAIR_MODLEL
	x_out = -LQR_K_R[2]*(chassis_move.x_set-chassis_move.x_filter);
	theta_out = -LQR_K_R[0]*(VMC_right.theta-0.0f);
	phi_out = -LQR_K_R[4]*(chassis_move.myPith-0.0f);
	x_dot_out = LQR_K_R[3]*(chassis_move.v_set-chassis_move.v_filter);
	theta_dot_out = -LQR_K_R[1]*(VMC_right.d_theta-0.0f);
	phi_dot_out = -LQR_K_R[5]*(chassis_move.myPithDot-0.0f);
	#endif
	// LQR计算左侧髋关节输出力矩		
	#if (REMOVE_X_IN_LQR == 1)
	VMC_right.Tp=-(LQR_K_R[6]*(VMC_right.theta-0.0f)
					+LQR_K_R[7]*(VMC_right.d_theta-0.0f)
					+LQR_K_R[8]*(0)
					+LQR_K_R[9]*(chassis_move.v_filter-chassis_move.v_set)
					+LQR_K_R[10]*(chassis_move.myPith-0.0f)
					+LQR_K_R[11]*(chassis_move.myPithDot-0.0f));
	#else	
	VMC_right.Tp=-(LQR_K_R[6]*(VMC_right.theta-0.0f)
					+LQR_K_R[7]*(VMC_right.d_theta-0.0f)
					+LQR_K_R[8]*(chassis_move.x_filter-chassis_move.x_set)
					+LQR_K_R[9]*(chassis_move.v_filter-chassis_move.v_set)
					+LQR_K_R[10]*(chassis_move.myPith-0.0f)
					+LQR_K_R[11]*(chassis_move.myPithDot-0.0f));
	#endif
	// 四个PID的计算，分别是yaw转向PD，roll保持PD，防劈叉PD，腿长PD  
					
	// 左侧轮毂输出计算
	chassis_move.wheel_motor[0].wheel_T= chassis_move.wheel_motor[0].wheel_T+chassis_move.turn_T;	//轮毂电机输出力矩
	mySaturate(&chassis_move.wheel_motor[0].wheel_T,-1.0f,1.0f);	
	#if (THETA_MAINTAIN_TEST == 1)
	VMC_right.Tp = +chassis_move.leg_tp;		
	#else
	VMC_right.Tp=VMC_right.Tp+chassis_move.leg_tp;		
	#endif
	

	// 利用支持力估算，进行离地检测。估算中等效了一步：轮子静止的。所以实际上偏向于多算了支持力
	// 实际效果预计更加钝。当然其实能算上轮子加速度，因为知道dL0。
	#if (SIMPLE_CHAIR_MODLEL == 1)
	left_flag=0;
	chassis_move.recover_flag=0;
	#else
	right_flag=ground_detectionR(&VMC_right,INS_DATA_HANDLELER_R);//右腿离地检测
	#endif
	if(chassis_move.recover_flag==0 && self_lock_flag == 0)	
	{//倒地自起不需要检测是否离地
		if(right_flag==1&&VMC_right.leg_flag==0)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			chassis_move.wheel_motor[0].wheel_T=0.0f;
			VMC_right.Tp = -(LQR_K_R[6]*(VMC_right.theta-0.0f)+ LQR_K_R[7]*(VMC_right.d_theta-0.0f));
			
			chassis_move.x_filter=0.0f;//对位移清零
			
			chassis_move.x_set=chassis_move.x_filter;
			chassis_move.turn_set = INS_DATA_HANDLELER_R->YawTotalAngle;
			VMC_right.Tp=VMC_right.Tp+chassis_move.leg_tp;		
		 }
		 else
		 {//没有离地
			 VMC_right.leg_flag=0;//置为0		
	   }
	}
	else 
	{
		VMC_right.torque_set[1] = 0.0f;
		VMC_right.torque_set[0] = 0.0f;
		self_lock_flag = 1;
		chassis_move.wheel_motor[0].wheel_T = 0.0f;
	}
	
	// 这个结果加到两边F上
	if(1 != chassis_move.F0_lock_flag)// 不要上来就锁F0，因为第一步调节腿长需要F0计算。蹲下之后到起跳那一步再去锁
	{
		VMC_right.F0=11.0f/arm_cos_f32(VMC_right.theta)+(LegR_Pid.Kp*(chassis_move.right_leg_set - VMC_right.L0)-LegR_Pid.Kd*VMC_right.d_L0);//前馈+pd
	}
	mySaturate(&VMC_right.F0,-100.0f,100.0f);//限幅 
		
	if(self_lock_flag!=1)
	{
			VMC_calc_2(&VMC_right);//计算期望的关节输出力矩
	}

	
  //额定扭矩
  mySaturate(&VMC_right.torque_set[1],-3.0f,3.0f);	
	mySaturate(&VMC_right.torque_set[0],-3.0f,3.0f);	
}
void ChassisR_task(void)
{
	// 先把数据获取方式打通
	INS_DATA_HANDLELER_R = INS_GetData();
	
	while(INS_DATA_HANDLELER_R->ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}	
	
	//这里面先把左腿的VMC结构体初始化了
	ChassisR_init();
	LQR_Get_K_Given_Length_R(VMC_right.L0);
	
	uint32_t CHASSR_TIME=1;		
	while(1)
	{
		//1 更新右腿数据
		chassisR_feedback_update(&chassis_move,&VMC_right);
		
		//2 控制计算
		chassisR_control_loop();
		R_output = chassis_move.wheel_motor[0].wheel_T;
		
		//3 发送指令
		if(chassis_move.start_flag==1)	
		{
			#if (JOINT_OFF == 1)
			osDelay(CHASSR_TIME);
			#else
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,VMC_right.torque_set[1]);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,VMC_right.torque_set[0]);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			#endif
			
			#if (WHEEL_OFF == 1)
			osDelay(CHASSR_TIME);
			#else
			mit_ctrl2(&hfdcan1,chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f, chassis_move.wheel_motor[0].wheel_T);//右边轮毂电机
			osDelay(CHASSR_TIME);
			#endif
		}
		else if(chassis_move.start_flag==0)	
		{
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1,chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//右边轮毂电机	
			osDelay(CHASSR_TIME);
		}
	}
	
}
