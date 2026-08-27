#include "roll_gain_calc.h"

/* 不依赖 math.h，避免部分嵌入式工程链接 libm 的麻烦 */
#define ROLL_PI_F              (3.14159265358979323846f)
#define ROLL_MIN_TRACK_WIDTH   (1.0e-6f)

RollPhysicalParam_t roll_robot_para = {
    .m_body      = 2.027f,
    .body_width  = 0.14f,
    .body_height = 0.07f,
    .track_width = 0.30f,
    .sign        = 1.0f,
};

static float RollAbs_Fast(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float RollSafeTrackWidth(float track_width)
{
    if (RollAbs_Fast(track_width) < ROLL_MIN_TRACK_WIDTH)
    {
        return ROLL_MIN_TRACK_WIDTH;
    }

    return track_width;
}

static float RollSafeSign(float sign)
{
    if (sign == 0.0f)
    {
        return 1.0f;
    }

    return sign;
}

float RollInertia_Calc(void)
{
    const float m = roll_robot_para.m_body;
    const float w = roll_robot_para.body_width;
    const float h = roll_robot_para.body_height;

    /*
     * 机体近似为长方体，绕前后方向 roll 轴转动：
     *
     *   Jx = m * (w^2 + h^2) / 12
     *
     * 其中：
     *   w: 左右宽度
     *   h: 上下高度
     *
     * 不使用 body_length，因为 roll 轴是前后方向。
     */
    return m * (w * w + h * h) / 12.0f;
}

void RollGain_Calc(float fn, float zeta, float *kp, float *kd)
{
    float wn;
    float Jx;
    float B;
    float sign;
    float scale;

    if ((kp == 0) || (kd == 0))
    {
        return;
    }

    if (fn < 0.0f)
    {
        fn = -fn;
    }

    if (zeta < 0.0f)
    {
        zeta = -zeta;
    }

    wn = 2.0f * ROLL_PI_F * fn;
    Jx = RollInertia_Calc();
    B = RollSafeTrackWidth(roll_robot_para.track_width);
    sign = RollSafeSign(roll_robot_para.sign);

    /*
     * 简化动力学：
     *
     *   Jx * roll_ddot = B * roll_f0
     *
     * 所以从目标 roll 力矩换算到 roll_f0：
     *
     *   roll_f0 = M_roll / B
     */
    scale = sign * Jx / B;

    *kp = scale * wn * wn;
    *kd = scale * 2.0f * zeta * wn;
}
