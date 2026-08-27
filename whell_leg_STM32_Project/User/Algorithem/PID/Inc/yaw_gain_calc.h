#ifndef YAW_GAIN_CALC_H
#define YAW_GAIN_CALC_H

typedef struct
{
    float m_body;       // 车身质量, kg
    float body_length;  // 车身前后长度, m
    float body_width;   // 车身左右宽度, m

    float m_leg;        // 单侧腿杆总质量, kg
    float m_wheel;      // 单侧轮子及端部等效质量, kg

    float leg_length;   // 等效腿长, m
    float leg_angle;    // 等效腿相对竖直方向角度, rad

    float track_width;  // 两轮中心距, m
    float wheel_radius; // 轮半径, m

    float sign;         // turn_T>0使yaw_rate>0填+1，否则填-1
} YawPhysicalParam_t;

/* 航向角保持：根据fn、zeta计算KP、KD */
void YawPositionGain_Calc(float fn,
                          float zeta,
                          float *kp,
                          float *kd);

/* 航向角速度控制：根据fn计算KP */
void YawVelocityGain_Calc(float fn,
                          float *kp);
void FFF_Damper_Kp_calc_KpKd(float FFF,float damp_ratio,float Kp,float* Kd);


#endif
