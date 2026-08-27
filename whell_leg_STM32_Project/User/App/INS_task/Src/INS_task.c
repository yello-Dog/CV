#include "ins_task.h"
#include "mahony_filter.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
//现在是读取到原始数据了，下面进行姿态解算

static INS_t INS;


struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
float gravity[3] = {0, 0, -9.81f};

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;

const IMU_Data_t* BMI088_DATA_HANDLE;

void INS_Init(void)
{ 
	 mahony_init(&mahony,1.0f,0.0f,0.001f);
   INS.AccelLPF = 0.0089f;
}


void INS_task(void)
{
	 INS_Init();
	 
	 while(1)
	 {  
		ins_dt = DWT_GetDeltaT(&INS_DWT_Count);

		mahony.dt = ins_dt;

    BMI088_Read();
		 
 		// 这一步是我代码结构新加的获取数据途径
    BMI088_DATA_HANDLE = BMI088_GetData();
		 
    INS.Accel[X_INS] = BMI088_DATA_HANDLE->Accel[X_INS];
    INS.Accel[Y_INS] = BMI088_DATA_HANDLE->Accel[Y_INS];
    INS.Accel[Z_INS] = BMI088_DATA_HANDLE->Accel[Z_INS];
	  Accel.x = BMI088_DATA_HANDLE->Accel[0];
	  Accel.y = BMI088_DATA_HANDLE->Accel[1];
		Accel.z = BMI088_DATA_HANDLE->Accel[2];
    INS.Gyro[X_INS] = -BMI088_DATA_HANDLE->Gyro[X_INS];
    INS.Gyro[Y_INS] = BMI088_DATA_HANDLE->Gyro[Y_INS];
    INS.Gyro[Z_INS] = BMI088_DATA_HANDLE->Gyro[Z_INS];
  	Gyro.x = BMI088_DATA_HANDLE->Gyro[0];
		Gyro.y = BMI088_DATA_HANDLE->Gyro[1];
		Gyro.z = BMI088_DATA_HANDLE->Gyro[2];

		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
	  RotationMatrix_update(&mahony);
				

       

    		
		//==================================================================================
		if(ins_time>3000.0f)
		{
			// 这个时候左右底盘任务开始。所以前面估计运动加速度虽然不准但是没使用，放在前面也可以
			INS.ins_flag=1;
			// 更新
      INS.Pitch=-mahony.roll;
		  INS.Roll=mahony.pitch;
		  INS.Yaw=mahony.yaw;
			
			// 轮足走的稳定的关键：
			// 1.不单纯用轮速估计速度，而是轮速送进建模部分，然后把建模和观测送进卡尔曼滤波做预测-修正
			// 2.加速度自身用机体系的前向加速度，然后过一个一阶低通滤波，减小噪声
			{ 
				// 对最后的去重力的前向加速度结果做一阶低通滤波，截至频率约为17HZ，given 1 tick = 1ms
				{
					INS.Y_MotionAccel_n = (INS.Accel[Y_INS]*arm_cos_f32(INS.Pitch) + INS.Accel[Z_INS]*arm_sin_f32(INS.Pitch)) * ins_dt / (INS.AccelLPF + ins_dt) 
																+ INS.Y_MotionAccel_n * INS.AccelLPF / (INS.AccelLPF + ins_dt); 
				}

				// 地球前向运动加速度死区判定
				if(fabsf(INS.Y_MotionAccel_n)<0.02f)
				{
					INS.Y_MotionAccel_n=0.0f;
				}
			}

			// 没用，但是带着了
			if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
			{
					INS.YawRoundCount--;
			}
			else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
			{
					INS.YawRoundCount++;
			}
			INS.YawTotalAngle = 6.283f* INS.YawRoundCount + INS.Yaw;
			INS.YawAngleLast = INS.Yaw;
		}
		else
		{
		 ins_time++;
		}
    osDelay(1);
	}
} 




const INS_t *INS_GetData(void)
{
    return &INS;
}


