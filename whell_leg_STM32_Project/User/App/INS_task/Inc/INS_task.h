#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"

#define X_INS 0
#define Y_INS 1
#define Z_INS 2

#define INS_TASK_PERIOD 1

// 这个是实时姿态数据结构体
typedef struct
{
    //float q[4]; // 四元数

		// IMU原始数据量
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
	
	
		// 这个应该没用到，暂时不需要
    // float MotionAccel_b[3]; // 机体坐标加速度
    // float MotionAccel_n[3]; // 绝对系加速度

		// 世界系下的前向加速度
		float Y_MotionAccel_n;
	
	
    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿，这个是最关心的
    float Roll;
    float Pitch;
    float Yaw;
		
    float YawTotalAngle;	// 考虑旋转多圈的yaw
		float YawAngleLast;
		float YawRoundCount;
		
		uint8_t ins_flag;
} INS_t;


// 这个不是实时IMU数据，这个更像是一些修正，比如说IMU安装角偏差
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern void INS_Init(void);
extern void INS_task(void);

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
extern const INS_t *INS_GetData(void);

#endif
