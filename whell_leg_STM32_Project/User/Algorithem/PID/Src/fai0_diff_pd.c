#include "fai0_diff_pd.h"

/* 圆周率。避免依赖部分编译器未定义的 M_PI。 */
#define FAI0_DIFF_PD_PI            (3.14159265358979323846f)

/* 单侧两根杆的总质量，单位 kg。 */
#define FAI0_DIFF_PD_ROD_MASS      (0.075f * 2.0f)

/* 单侧轮子/端部的等效质点质量，单位 kg。 */
#define FAI0_DIFF_PD_POINT_MASS    (0.625f)

bool Fai0DiffPD_Calculate(float fn_hz,
                         float zeta,
                         float left_leg_length_m,
                         float right_leg_length_m,
                         float *kp_theta,
                         float *kd_theta)
{
    float left_length_sq;
    float right_length_sq;

    float left_rod_inertia;
    float left_point_inertia;
    float left_inertia;

    float right_rod_inertia;
    float right_point_inertia;
    float right_inertia;

    float inertia_sum;
    float equivalent_inertia;
    float omega_n;

    /* 输出指针不能为空。 */
    if ((kp_theta == 0) || (kd_theta == 0))
    {
        return false;
    }

    /* 默认输出安全值。 */
    *kp_theta = 0.0f;
    *kd_theta = 0.0f;

    /*
     * 基本参数检查：
     * 自然频率必须大于 0；
     * 阻尼比不能为负；
     * 左右腿长必须大于 0。
     */
    if ((fn_hz <= 0.0f) ||
        (zeta < 0.0f) ||
        (left_leg_length_m <= 0.0f) ||
        (right_leg_length_m <= 0.0f))
    {
        return false;
    }

    left_length_sq  = left_leg_length_m  * left_leg_length_m;
    right_length_sq = right_leg_length_m * right_leg_length_m;

    /*
     * 左腿单侧转动惯量：
     * I_rod   = (1/3) * m_rod * L^2
     * I_point = m_point * L^2
     */
    left_rod_inertia =
        (1.0f / 3.0f) * FAI0_DIFF_PD_ROD_MASS * left_length_sq;

    left_point_inertia =
        FAI0_DIFF_PD_POINT_MASS * left_length_sq;

    left_inertia = left_rod_inertia + left_point_inertia;

    /* 右腿单侧转动惯量。 */
    right_rod_inertia =
        (1.0f / 3.0f) * FAI0_DIFF_PD_ROD_MASS * right_length_sq;

    right_point_inertia =
        FAI0_DIFF_PD_POINT_MASS * right_length_sq;

    right_inertia = right_rod_inertia + right_point_inertia;

    /*
     * 左右差分模态的约化转动惯量：
     * I_eq = I_L * I_R / (I_L + I_R)
     */
    inertia_sum = left_inertia + right_inertia;

    if (inertia_sum <= 0.0f)
    {
        return false;
    }

    equivalent_inertia =
        (left_inertia * right_inertia) / inertia_sum;

    /* omega_n = 2*pi*fn */
    omega_n = 2.0f * FAI0_DIFF_PD_PI * fn_hz;

    /*
     * KP = I_eq * omega_n^2
     * KD = 2*zeta*I_eq*omega_n
     */
    *kp_theta =
        equivalent_inertia * omega_n * omega_n;

    *kd_theta =
        2.0f * zeta * equivalent_inertia * omega_n;

    return true;
}
