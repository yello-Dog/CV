#ifndef ROLL_GAIN_CALC_H
#define ROLL_GAIN_CALC_H

#define ROLL_ZETA 0.7
#define ROLL_FREQ 5
/*
 * roll_gain_calc
 *
 * 简化 roll 轴物理建模：
 *   1. 只考虑竖直站立条件；
 *   2. 不考虑纵向腿角；
 *   3. 不考虑腿和轮子质量；
 *   4. 只把机体看成长方体刚体；
 *   5. roll 控制输出 roll_f0 表示左右腿支撑力差分的一半：
 *
 *      F_R = F_base_R + roll_f0
 *      F_L = F_base_L - roll_f0
 *
 *   因此：
 *      M_roll = track_width * roll_f0
 *
 *   roll 动力学：
 *      Jx * roll_ddot = track_width * roll_f0
 *
 *   目标二阶系统：
 *      e_ddot + 2*zeta*wn*e_dot + wn^2*e = 0
 *
 *   得到：
 *      Kp = Jx * wn^2 / track_width
 *      Kd = 2*zeta*wn*Jx / track_width
 */

typedef struct
{
    float m_body;       /* 机体质量, kg */
    float body_width;   /* 机体左右宽度, m */
    float body_height;  /* 机体高度, m */
    float track_width;  /* 左右支撑点距离/轮距, m */

    /*
     * 方向修正：
     *   sign =  1.0f: 默认方向
     *   sign = -1.0f: 如果实机 roll 反馈方向反了，改成 -1
     *
     * 注意：
     *   不建议直接把 Kp/Kd 写成负数。
     *   实机方向反了，统一改 sign。
     */
    float sign;
} RollPhysicalParam_t;

/* 全局参数对象。用户在任务中直接给它赋值即可。 */
extern RollPhysicalParam_t roll_robot_para;

/* 计算机体绕 roll 轴的转动惯量 Jx, 单位 kg*m^2 */
float RollInertia_Calc(void);

/*
 * 根据自然频率 fn 和阻尼比 zeta 计算 roll_f0 的等效弹簧阻尼增益。
 *
 * 输入：
 *   fn   : 目标无阻尼自然频率, Hz
 *   zeta : 阻尼比
 *
 * 输出：
 *   kp : roll_f0 对 roll 角误差的增益, N/rad
 *   kd : roll_f0 对 roll 角速度的增益, N*s/rad
 *
 * 推荐初值：
 *   fn   = 1.5f ~ 2.5f
 *   zeta = 0.8f ~ 1.0f
 */
void RollGain_Calc(float fn, float zeta, float *kp, float *kd);



#endif /* ROLL_GAIN_CALC_H */
