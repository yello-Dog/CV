#ifndef FAI0_DIFF_PD_H
#define FAI0_DIFF_PD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief  根据自然频率、阻尼比和左右腿长计算摆角差分 PD 参数
 *
 * 物理模型：
 *  1. 单侧两根连杆近似为绕一端转动的细杆：
 *         I_rod = (1/3) * m_rod * L^2
 *
 *  2. 单侧轮子/端部质量近似为端点质点：
 *         I_point = m_point * L^2
 *
 *  3. 左右腿差分模态的等效转动惯量：
 *         I_eq = I_L * I_R / (I_L + I_R)
 *
 *  4. PD 参数：
 *         omega_n = 2*pi*fn
 *         KP = I_eq * omega_n^2
 *         KD = 2*zeta*I_eq*omega_n
 *
 * @param fn_hz               目标无阻尼自然频率，单位 Hz
 * @param zeta                阻尼比，无量纲
 * @param left_leg_length_m   左腿当前等效腿长，单位 m
 * @param right_leg_length_m  右腿当前等效腿长，单位 m
 * @param kp_theta            输出比例系数，单位 N*m/rad
 * @param kd_theta            输出微分系数，单位 N*m*s/rad
 *
 * @return true  参数有效，计算成功
 * @return false 参数无效，输出 KP、KD 被置为 0
 */
bool Fai0DiffPD_Calculate(float fn_hz,
                         float zeta,
                         float left_leg_length_m,
                         float right_leg_length_m,
                         float *kp_theta,
                         float *kd_theta);

#ifdef __cplusplus
}
#endif

#endif /* FAI0_DIFF_PD_H */
