#include "yaw_gain_calc.h"
#include "arm_math.h"
#include "VMC.h"

#define TWO_PI 6.28318530718f

YawPhysicalParam_t wheel_leg_robot_para;

extern vmc_leg_t VMC_right;
extern vmc_leg_t VMC_left;


void YawModel_Para_Update()
{
	wheel_leg_robot_para.body_length = 0.18;
	wheel_leg_robot_para.body_width = 0.14;
	wheel_leg_robot_para.leg_angle = (VMC_right.theta+VMC_left.theta)/2.0f;
	wheel_leg_robot_para.leg_length = (VMC_right.L0+VMC_left.L0)/2.0f;
	wheel_leg_robot_para.m_body = 2.027;
	wheel_leg_robot_para.m_leg = 0.075;
	wheel_leg_robot_para.m_wheel = 0.62;
	wheel_leg_robot_para.sign = 1.0; // 输出和效果反向
	wheel_leg_robot_para.track_width = 0.3; // 轮距
	wheel_leg_robot_para.wheel_radius = 0.0603;
}

static float YawInertia_Calc(const YawPhysicalParam_t *p)
{
    float d = 0.5f * p->track_width;
    float x = p->leg_length * arm_sin_f32(p->leg_angle);

    /* 车身按矩形刚体；腿按均匀杆；轮子按端部质点 */
    float J_body = p->m_body
                 * (p->body_length * p->body_length
                 +  p->body_width  * p->body_width) / 12.0f;

    float J_leg = 2.0f * p->m_leg
                * (d * d + x * x / 3.0f);

    float J_wheel = 2.0f * p->m_wheel
                  * (d * d + x * x);

    return J_body + J_leg + J_wheel;
}

void YawPositionGain_Calc(float fn,
                          float zeta,
                          float *kp,
                          float *kd)
{
		YawModel_Para_Update();
    float wn = TWO_PI * fn;
    float Jz = YawInertia_Calc(&wheel_leg_robot_para);
    float scale = wheel_leg_robot_para.sign * Jz * wheel_leg_robot_para.wheel_radius / wheel_leg_robot_para.track_width;

    *kp = scale * wn * wn;
    *kd = scale * 2.0f * zeta * wn;
}

// 闭环时间常数tao = 1/（2pifn）
void YawVelocityGain_Calc(float fn,
                          float *kp)
{
		YawModel_Para_Update();
    float wc = TWO_PI * fn;
    float Jz = YawInertia_Calc(&wheel_leg_robot_para);
    float scale = wheel_leg_robot_para.sign * Jz * wheel_leg_robot_para.wheel_radius / wheel_leg_robot_para.track_width;
    *kp = scale * wc;
}

void FFF_Damper_Kp_calc_KpKd(float FFF,float damp_ratio,float Kp,float* Kd)
{
	float equal_mass = FFF/9.81f;
	*Kd = 2 * damp_ratio * sqrtf(Kp * equal_mass);
}