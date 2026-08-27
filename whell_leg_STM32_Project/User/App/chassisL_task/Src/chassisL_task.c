#include "chassisL_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "VMC.h"
#include "pid.h"
#include "arm_math.h"
#include "DebugConfig.h"
#include "fai0_diff_pd.h"
#include "yaw_gain_calc.h"
#include "buzzer.h"

/**
 
 应该先获得当前状态和目标状态（定义数据结构体）
 -IMU数据
 -各个电机的角度，角速度数据
 -KF估计出来的xdot，出自于observetask
 
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
chassis_t chassis_move;

// IMU原始数据只读接口.const指针意思是不能通过这个指针修改指向的数据，但是指针本身是可以变的
const INS_t* INS_DATA_HANDLELER_L;

// 左腿的VMC对象
vmc_leg_t VMC_left;
extern vmc_leg_t VMC_right;

float L_output;

extern uint8_t yaw_pd_position_OR_velo_FLAG;

// LQR矩阵
float LQR_K_L[12]={ 
   0,0,0,0,0,0,
   0,0,0,0,0,0
};

uint8_t OFF_GROUND_FLAG = 0;

// 12条三次拟合曲线的系数

/*
    %定义QR矩阵
    %定义QR矩阵
    Q=diag([1 0.1 30 20 200 0.6]);
    R=[5 0;0 5];
		这个参数比较漂亮,思路对了但是有点激进
		核心思路是：
		1.主体平衡是主力
		2.腿杆要灵活，因为其是保持平衡必须用的。不要因为以达到平衡的目的来限制腿杆
		3.轮子关节其实都可以灵活，轮子很难达到1N限幅的（实验过）。但是轮子输出大了也不好，导致抖动
		4.路程和速度是关键。路程保证你能够跟上，有一个基准点不会乱动。速度保证能快速跟上路程响应（次关键）
		所以LQR调参依旧得考虑物理情景
*/    

/*
// 这个参数十分牛逼，别动了！！！！！！！但是还是动了，因为不能倒地自启
    Q=diag([1 0.7 30 15 300 0.6]);
    R=[10 0;0 1]; 10可能欠调节，但是就那样吧，这个是下下面的
*/
/*
    Q=diag([1 0.5 30 15 300 0.6]);
    R=[8 0;0 1];
*/
//float Poly_Coefficient_Move[12][4]={
//																{-170.8863,111.0430,-38.7686,-0.2419},
//																{-12.0913,5.0053,-4.0491,-0.0589},
//																{-121.3727,70.3043,-14.0160,-0.9469},
//																{-103.6976,59.4866,-12.6153,-1.1120},
//																{-486.3987,322.1030,-78.2426,7.9282},
//																{-20.7129,14.1385,-3.6005,0.4431},
//																{-183.2944,153.2616,-47.2673,6.7141},
//																{-37.4534,27.8461,-7.6826,1.0490},
//																{-430.7917,283.8701,-68.3538,6.6751},
//																{-485.0096,314.7457,-74.3924,7.1351},
//																{1.2260e+03,-713.3025,143.2889,7.1500},
//																{69.9390,-41.4522,8.5860,0.1853}}; 
float Poly_Coefficient_Move[12][4]={
																{-190.6264,123.0793,-39.7820,-0.0365},
																{-17.2403,8.4036,-4.4357,-0.0303},
																{-127.3789,74.6610,-15.0994,-0.6483},
																{-114.8402,66.5805,-14.0353,-0.8149},
																{-392.7641,273.1558,-70.0617,7.4907},
																{-16.1884,11.7345,-3.1857,0.4147},
																{-105.0202,124.2808,-47.2078,7.5718},
																{-32.1098,27.3445,-8.5063,1.2586},
																{-392.3093,270.8734,-68.7149,7.0737},
																{-464.5862,313.1095,-77.3172,7.7586},
																{1.4317e+03,-842.7391,171.6797,4.9203},
																{79.2344,-47.5263,9.9832,0.0681}};  

// 下下面这个参数其实不错，备选。但是问题是杆太灵活可能回不来。下面给杆子加阻尼
/*
    Q=diag([3 1 25 10 300 0.6]);
    R=[3 0;0 5];
		这个不行的话改下一个：    
		Q=diag([3 0.5 30 15 300 0.6]);
    R=[3 0;0 5];
*/
// 这个不错。稳定性很高的参数。但是灵活性和定点不行
//float Poly_Coefficient_Move[12][4]={
//																{-133.2155,88.0218,-36.2713,-2.3913},
//																{-3.1533,-1.2416,-3.3297,-0.4908},
//																{-31.2371,19.1839,-4.1365,-2.5633},
//																{14.9366,-9.9337,1.3627,-3.0161},
//																{-277.0293,198.2003,-54.1033,6.4422},
//																{-15.1905,10.8227,-2.9676,0.4246},
//																{-10.2609,16.5257,-7.7872,1.5151},
//																{-7.5741,6.0068,-1.9165,0.2933},
//																{-50.7494,37.1252,-10.3406,1.2048},
//																{-67.7681,46.8119,-12.2225,1.3347},
//																{115.3213,-70.7524,15.3083,6.6379},
//																{7.5102,-4.7075,1.0604,0.3184}};
//float Poly_Coefficient_Move[12][4]={
//																{-43.8336,41.6621,-29.6962,-1.0442},
//																{8.3677,-6.6109,-2.7892,-0.0910},
//																{-22.4596,12.9172,-2.5694,-2.0522},
//																{-0.8389,1.0711,-1.5639,-2.2626},
//																{-296.4330,183.8934,-41.9379,4.2482},
//																{-16.0619,10.2152,-2.4424,0.3243},
//																{-49.5098,31.3734,-7.5032,1.0320},
//																{-5.3237,3.2893,-0.7489,0.1103},
//																{-86.7708,53.7272,-12.1612,1.1527},
//																{-95.3808,58.5419,-13.0831,1.2247},
//																{92.3870,-53.4698,10.7676,6.3854},
//																{7.4507,-4.4152,0.9284,0.3296}};
// 这个参数比较漂亮,思路对了但是有点激进
//float Poly_Coefficient_Move[12][4]={
//																{-59.3785,54.6021,-35.8243,-0.7214},
//																{9.3050,-7.2433,-3.2526,-0.0766},
//																{-25.8175,15.0271,-3.0488,-2.2232},
//																{-13.3056,8.8906,-3.4645,-2.4096},
//																{-243.6732,153.0475,-35.8232,3.9062},
//																{-14.4850,9.3342,-2.2891,0.3291},
//																{-27.7002,18.1003,-4.7198,0.9290},
//																{-4.1862,2.5125,-0.5425,0.1089},
//																{-88.5062,55.4304,-12.8430,1.2909},
//																{-96.0584,59.7349,-13.7067,1.3745},
//																{86.1291,-50.4770,10.3840,5.6344},
//																{7.6398,-4.5776,0.9810,0.3164}};

// PID对象 太对了，全都弹簧阻尼
PidTypeDef LegL_PD;//左腿的腿长pd
PidTypeDef Tp_PD;//防劈叉补偿pd
PidTypeDef Turn_PD_Position;//转向pd

															
void Pensation_init(PidTypeDef *Tp,PidTypeDef *turnp)
{//补偿pid初始化：横滚角补偿、防劈叉补偿、偏航角补偿

	const static float tp_pid[3] = {0,0, 0}; //这个初始化无所谓，后面会实时计算
	const static float turn_pidp[3] = {TURN_PID_KP_P, TURN_PID_KI_P, TURN_PID_KD_P};
	const static float turn_pidv[3] = {TURN_PID_KP_V, TURN_PID_KI_V, TURN_PID_KD_V};

	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	
	PID_init(turnp, PID_POSITION, turn_pidp, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
}													
float LQR_Cubic_Spline_calc(float *coe,float len)
{
   
  return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];
}
void LQR_Get_K_Given_Length_L(float lenth)
{
		for(int i=0;i<12;i++)
	{
		LQR_K_L[i]=LQR_Cubic_Spline_calc(&Poly_Coefficient_Move[i][0],lenth);	
	}
}

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
		Start_buzzer();
  }
  else if(*in > max)
  {
    *in = max;
		Start_buzzer();
  }
	else
	{
		End_buzzer();
	}
}

float legl_pid[3];
void ChassisL_init()
{

	VMC_init(&VMC_left);//给杆长赋值
	
	// FDCAN2: left side
	// joint_motor[2]: left leg phi1, tx id = 6, rx feedback id = 3
	// joint_motor[3]: left leg phi4, tx id = 8, rx feedback id = 4
	// wheel_motor[1]: left wheel,    tx id = 1, rx feedback id = 0
	joint_motor_init(&chassis_move.joint_motor[2],6,MIT_MODE);//发送id为6
	joint_motor_init(&chassis_move.joint_motor[3],8,MIT_MODE);//发送id为8
	wheel_motor_init(&chassis_move.wheel_motor[1],1,MIT_MODE);//发送id为1
	
	// 腿长PD控制器初始化
	legl_pid[0] = LEG_PD_KP;
	FFF_Damper_Kp_calc_KpKd(11.f,ZETA,(float)LEG_PD_KP,&legl_pid[2]);
	legl_pid[1] = 0;
	PID_init(&LegL_PD, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

  // PD控制器初始化
	Pensation_init(&Tp_PD,&Turn_PD_Position);
	// 发送十次，主要是怕没配置上
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan2,chassis_move.joint_motor[3].para.id,chassis_move.joint_motor[3].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan2,chassis_move.joint_motor[2].para.id,chassis_move.joint_motor[2].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan2,chassis_move.wheel_motor[1].para.id,chassis_move.wheel_motor[1].mode);//左边轮毂电机
	  osDelay(1);
	}
}



/* 
 这个左右两边的电机数据必须得写两个函数来更新，还不能独立更新，因为两边用的两个CAN外设
 另外改一下，因为INS是只读的，就用只读的接口来读取
*/
void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc)
{
	// 为左边的VMC解算传数据
  vmc->phi1=pi-chassis->joint_motor[2].para.pos;
	vmc->phi4=-chassis->joint_motor[3].para.pos;
	
	// 传速度
	vmc->d_phi1 = -chassis->joint_motor[2].para.vel;
	vmc->d_phi4 = -chassis->joint_motor[3].para.vel;
		
	// IMU的俯仰角的欧拉角和角速度
	chassis->myPith = 0.0f-INS_DATA_HANDLELER_L->Pitch;
	chassis->myPithDot = 0.0f-INS_DATA_HANDLELER_L->Gyro[0];
	
	// L腿更新
	chassis->total_yaw=INS_DATA_HANDLELER_L->YawTotalAngle;
	chassis->roll=INS_DATA_HANDLELER_L->Roll;
	chassis->pai0_err=(vmc->phi0-VMC_right.phi0); //左减右，是和仿真对的
	
}

// 左腿离地标志
uint8_t left_flag;
uint8_t self_lock_flag = 0;
extern uint8_t right_flag;
uint8_t task_counter = 0;	// 状态机变量
float absf(float a)
{
	if(a>=0)
	{
		return a;
	}else
	{
		return -a;
	}
}
void chassisL_control_loop()
{
	// VMC相关计算----待补充
	VMC_calc_1_left(&VMC_left);
	
	// 更新K
	LQR_Get_K_Given_Length_L(VMC_left.L0);
	
	/*-----------------------------------------------LQR-------------------------------------------------------*/
	// 下面用更新完的K来计算T TP
	// LQR计算左侧轮毂电机输出力矩
	#if (REMOVE_X_IN_LQR == 1)
	chassis_move.wheel_motor[1].wheel_T=-(LQR_K_L[0]*(VMC_left.theta-0.0f)
																	+LQR_K_L[1]*(VMC_left.d_theta-0.0f)
																	+LQR_K_L[2]*(0)
																	+LQR_K_L[3]*(chassis_move.v_filter-chassis_move.v_set)
																	+LQR_K_L[4]*(chassis_move.myPith-0.0f)
																	+LQR_K_L[5]*(chassis_move.myPithDot-0.0f));// 这里面负负抵消了，因为左轮确实是应该给负的T
	
	#else
	chassis_move.wheel_motor[1].wheel_T=-(LQR_K_L[0]*(VMC_left.theta-0.0f)
																	+LQR_K_L[1]*(VMC_left.d_theta-0.0f)
																	+LQR_K_L[2]*(chassis_move.x_filter-chassis_move.x_set)
																	+LQR_K_L[3]*(chassis_move.v_filter-chassis_move.v_set)
																	+LQR_K_L[4]*(chassis_move.myPith-0.0f)
																	+LQR_K_L[5]*(chassis_move.myPithDot-0.0f));// 这里面负负抵消了，因为左轮确实是应该给负的T
	#endif
	// LQR计算左侧髋关节输出力矩	
	#if (REMOVE_X_IN_LQR == 1)
	VMC_left.Tp=-(LQR_K_L[6]*(VMC_left.theta-0.0f)
					+LQR_K_L[7]*(VMC_left.d_theta-0.0f)
					+LQR_K_L[8]*(0)
					+LQR_K_L[9]*(chassis_move.v_filter-chassis_move.v_set)
					+LQR_K_L[10]*(chassis_move.myPith-0.0f)
					+LQR_K_L[11]*(chassis_move.myPithDot-0.0f));
	#else	
	VMC_left.Tp=-(LQR_K_L[6]*(VMC_left.theta-0.0f)
					+LQR_K_L[7]*(VMC_left.d_theta-0.0f)
					+LQR_K_L[8]*(chassis_move.x_filter-chassis_move.x_set)
					+LQR_K_L[9]*(chassis_move.v_filter-chassis_move.v_set)
					+LQR_K_L[10]*(chassis_move.myPith-0.0f)
					+LQR_K_L[11]*(chassis_move.myPithDot-0.0f));
	#endif
	// 四个PID的计算，分别是yaw转向PD，roll保持PD，防劈叉PD，腿长PD  
	
	
	// yaw转向PD，因为D项误差可以直接观测，不用暴力微分  这个会直接差分作用在左右轮子输出上。然后是位置和速度分开控制
//	if(yaw_pd_position_OR_velo_FLAG == 0)	// 航向固定控制器
//	{
		// 标准负反馈控制格式
		// e = target - measure; u = e*kp + e*kd;
		// 如果err是正的，那么比例项计算的输出就会让err减小
		YawPositionGain_Calc(2,1.2,&Turn_PD_Position.Kp,&Turn_PD_Position.Kd);
		float err_Position = -chassis_move.turn_set + INS_DATA_HANDLELER_L->YawTotalAngle;
		float err_position_dot = INS_DATA_HANDLELER_L->Gyro[2];
		chassis_move.turn_T = Turn_PD_Position.Kp*err_Position + Turn_PD_Position.Kd*err_position_dot;

	// roll保持PD，因为D项误差可以直接观测，不用暴力微分  这个会并联到腿长控制器的输出上

	
	// 防劈叉PD
	Fai0DiffPD_Calculate(10,0.5,VMC_left.L0,VMC_right.L0,&Tp_PD.Kp,&Tp_PD.Kd);
	chassis_move.leg_tp = Tp_PD.Kp*chassis_move.pai0_err+Tp_PD.Kd*(VMC_left.d_phi0-VMC_right.d_phi0);//防劈叉pid计算，和仿真是对上的
					
	// 左侧轮毂输出计算
	chassis_move.wheel_motor[1].wheel_T= chassis_move.wheel_motor[1].wheel_T-chassis_move.turn_T;	//轮毂电机输出力矩，这个加法方向对了
	mySaturate(&chassis_move.wheel_motor[1].wheel_T,-1.0f,1.0f);	
	#if (THETA_MAINTAIN_TEST == 1)
	VMC_left.Tp = -chassis_move.leg_tp;		
	#else
	VMC_left.Tp=VMC_left.Tp-chassis_move.leg_tp;//髋关节输出力矩--和达妙思路一致--这个符号也和仿真是对上的
	#endif
	// 这个结果加到两边F上，然后roll补偿对其进行差分
	/*--------------------------------------------------------------准备起跳----------------------------------------------------------------------*/
	left_flag=ground_detectionL(&VMC_left,INS_DATA_HANDLELER_L);//左腿离地检测
	if(chassis_move.attampt_jump_flag == 1)
	{
		if(absf(chassis_move.myPithDot)<0.1&&absf(VMC_left.d_theta)<0.1&&absf(VMC_right.d_theta)<0.1&&absf(VMC_left.theta)<0.1&&absf(VMC_right.theta)<0.1)
		{
			// 条件满足，进入起跳流程
			chassis_move.jump_flag = 1;
			chassis_move.attampt_jump_flag = 0;
		}else
		{
			// 条件不满足，驳回
			chassis_move.attampt_jump_flag = 0;
		}
	}
/*--------------------------------------------------------------开始起跳----------------------------------------------------------------------*/
	if(1 != chassis_move.F0_lock_flag)// 不要上来就锁F0，因为第一步调节腿长需要F0计算。蹲下之后到起跳那一步再去锁
	{
		VMC_left.F0=11.0f/arm_cos_f32(VMC_left.theta)+(LegL_PD.Kp*(chassis_move.left_leg_set - VMC_left.L0)-LegL_PD.Kd*VMC_left.d_L0);//前馈+pd
	}
	if(1 == chassis_move.jump_flag)
	{

		switch(task_counter)
		{
			case 0:
				// 在外面已经把有关操作腿长的代码锁住了
				chassis_move.right_leg_set = 0.07;
				chassis_move.left_leg_set = 0.07;
				if(absf(VMC_left.L0-VMC_right.L0)<=0.007&&VMC_left.L0<0.8)
				{
					task_counter++;
				}
				break;
			case 1:
				// 在外面要把操作F0的代码锁住，左右都要锁
				chassis_move.F0_lock_flag = 1;
				// 这里面自己设F0
				VMC_left.F0 = 60;
				VMC_right.F0 = 60;
				if(VMC_left.L0 > 0.16f &&VMC_right.L0 > 0.16f)
				{
					chassis_move.F0_lock_flag = 0;
					task_counter++;
				}
				break;
			case 2:
				chassis_move.left_leg_set = 0.11;
				chassis_move.right_leg_set = 0.11;
				if(left_flag == 0&&right_flag == 0)
				{
					task_counter++;
				}
				break;
			case 3:
				// 这个是恢复锁以及状态位的收尾环节
				chassis_move.jump_flag = 0;
				chassis_move.attampt_jump_flag = 0;
				task_counter = 0;
				chassis_move.F0_lock_flag = 0;
				break;
			default:
				break;
		}
	}
	
	// 利用支持力估算，进行离地检测。估算中等效了一步：轮子静止的。所以实际上偏向于多算了支持力
	// 实际效果预计更加钝。当然其实能算上轮子加速度，因为知道dL0。
	
	#if (SIMPLE_CHAIR_MODLEL == 1)
	left_flag=0;
	chassis_move.recover_flag=0;
	#else

	//==============================AI写的延时判断，给轮足1S倒地自起的恢复时间，其中不会判定是否倒地=============================
	{
			static uint8_t last_self_lock_flag = 1;
			static uint8_t start_blank_flag = 0;
			static uint32_t start_blank_tick = 0;

			// 检测 self_lock_flag 从 1 -> 0
			// 也就是从自锁状态被解锁的那一瞬间
			if(last_self_lock_flag == 1 && self_lock_flag == 0)
			{
					start_blank_flag = 1;
					start_blank_tick = osKernelGetTickCount();
			}

			// 只有非自锁状态下，才考虑倒地检测
			if(self_lock_flag == 0)
			{
					// 1s 空窗期内，不执行倒地检测
					if(start_blank_flag == 1)
					{
							if((osKernelGetTickCount() - start_blank_tick) >= osKernelGetTickFreq())
							{
									start_blank_flag = 0;
									chassis_move.recover_flag = fall_detection();
							}
							else
							{
								chassis_move.recover_flag = 0;
									// 空窗期，什么都不判
									// 不执行 fall_detection()
							}
					}
					else
					{
							chassis_move.recover_flag = fall_detection();
					}
			}
			else
			{
					// 重新进入自锁后，清掉空窗期状态
					// 等下一次 self_lock_flag 从 1 -> 0 时重新触发
					start_blank_flag = 0;
			}

			last_self_lock_flag = self_lock_flag;
	}
	#endif
	
	if(chassis_move.recover_flag==0 && self_lock_flag == 0)	
	{//倒地自起不需要检测是否离地
		if(left_flag==1&&right_flag==1&&VMC_left.leg_flag==0)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			OFF_GROUND_FLAG = 1;
			chassis_move.wheel_motor[1].wheel_T=0.0f;
			VMC_left.Tp = -(LQR_K_L[6]*(VMC_left.theta-0.0f)+ LQR_K_L[7]*(VMC_left.d_theta-0.0f));
			
			chassis_move.x_filter=0.0f;//对位移清零
			chassis_move.x_set = chassis_move.x_filter;
			chassis_move.turn_set = INS_DATA_HANDLELER_L->YawTotalAngle;
			#if (THETA_MAINTAIN_TEST == 1)
			VMC_left.Tp = -chassis_move.leg_tp;		
			#else
			VMC_left.Tp=VMC_left.Tp-chassis_move.leg_tp;		
			#endif
		 }
		 else
		 {//没有离地
			 OFF_GROUND_FLAG = 0;
			 VMC_left.leg_flag=1;//置为0		
	   }
	}
	else 
	{
		VMC_left.torque_set[1] = 0.0f;
		VMC_left.torque_set[0] = 0.0f;
		chassis_move.wheel_motor[1].wheel_T = 0.f;
		self_lock_flag = 1;
	}

	
	mySaturate(&VMC_left.F0,-100.0f,100.0f);//限幅 

	
	if(self_lock_flag!=1)
	{
			VMC_calc_2(&VMC_left);//计算期望的关节输出力矩
	}

	
  //额定扭矩
  mySaturate(&VMC_left.torque_set[1],-3.0f,3.0f);	
	mySaturate(&VMC_left.torque_set[0],-3.0f,3.0f);	
	if(absf(VMC_left.torque_set[1]) >= 3||absf(VMC_left.torque_set[0]) >= 3)
	{
		Start_buzzer();
	}
}

void ChassisL_task(void)
{
	// 先把数据获取方式打通
	INS_DATA_HANDLELER_L = INS_GetData();
	
	#if DEBUGGING_WITHOUT_IMU == 0
		while(INS_DATA_HANDLELER_L->ins_flag==0)
		{//等待加速度收敛
			osDelay(1);	
		}	
	#endif
	//这里面先把左腿的VMC结构体初始化了
	ChassisL_init();
	LQR_Get_K_Given_Length_L(VMC_left.L0);
	uint32_t CHASSL_TIME=1;		
	while(1)
	{
		#if (DEBUGGING_WITHOUT_IMU == 0)
			//1 更新左腿数据
			chassisL_feedback_update(&chassis_move,&VMC_left);
			
			//2 控制计算
			chassisL_control_loop();
			L_output = chassis_move.wheel_motor[1].wheel_T;
		
			//3 发送指令
			if(chassis_move.start_flag==1)	
			{
				#if (JOINT_OFF == 1)
				osDelay(CHASSL_TIME);
				#else
				mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,-VMC_left.torque_set[1]);//left.torque_set[1]
				osDelay(CHASSL_TIME);
				mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,-VMC_left.torque_set[0]);
				osDelay(CHASSL_TIME);
				#endif
				#if (WHEEL_OFF == 1)
				osDelay(CHASSL_TIME);
				#else
				mit_ctrl2(&hfdcan2,chassis_move.wheel_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,-chassis_move.wheel_motor[1].wheel_T);//左边边轮毂电机，最后发送再给负号，之前全是假设T正往前转
				osDelay(CHASSL_TIME);
				#endif
			}
			else if(chassis_move.start_flag==0)	
			{
				mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left.torque_set[1]
				osDelay(CHASSL_TIME);
				mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
				osDelay(CHASSL_TIME);
				mit_ctrl2(&hfdcan2,chassis_move.wheel_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//左边轮毂电机	
				osDelay(CHASSL_TIME);
			}
		#else

			mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 1.0f,0.0f, 1.0f,0.0f);//左边轮毂电机	
			osDelay(CHASSL_TIME*3);
		#endif 
	}
	
}





