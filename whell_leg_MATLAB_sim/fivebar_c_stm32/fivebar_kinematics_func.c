#include <math.h>
#include "fivebar_kinematics_func.h"

#ifndef M_PI
#define M_PI 3.1415926f
#endif

void fivebar_kinematics_func(float phi1, float phi4, float *L0, float *phi0)
{
    {
        float t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t13, t12, t15, t14, t16, t17, t18, t19, t20, t0;
        t2 = arm_cos_f32(phi1);
        t3 = arm_cos_f32(phi4);
        t4 = arm_sin_f32(phi1);
        t5 = arm_sin_f32(phi4);
        t6 = t2*1.5E+1;
        t7 = t3*1.5E+1;
        t8 = -t5;
        t9 = -t6;
        t10 = t4+t8;
        t11 = t10*t10;
        t13 = t7+t9+1.6E+1;
        t12 = t11*2.25E+2;
        t15 = t13*t13;
        t14 = -t12;
        t16 = -t15;
        t17 = t12+t15;
        t18 = 1.0/sqrt(t17);
        t19 = t14+t16+3.136E+3;
        t20 = sqrt(t19);
        t0 = sqrt(powf(t2*(3.0/8.0E+1)+t3*(3.0/8.0E+1)+(t18*t20*(t4*(3.0/4.0E+1)-t5*(3.0/4.0E+1)))/2.0,2.0)+powf(t4*(3.0/8.0E+1)+t5*(3.0/8.0E+1)+(t18*t20*(t2*(-3.0/4.0E+1)+t3*(3.0/4.0E+1)+2.0/2.5E+1))/2.0,2.0));
        *L0 = t0;
    }

    {
        float t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t13, t12, t15, t14, t16, t17, t18, t19, t20, t0;
        t2 = arm_cos_f32(phi1);
        t3 = arm_cos_f32(phi4);
        t4 = arm_sin_f32(phi1);
        t5 = arm_sin_f32(phi4);
        t6 = t2*1.5E+1;
        t7 = t3*1.5E+1;
        t8 = -t5;
        t9 = -t6;
        t10 = t4+t8;
        t11 = t10*t10;
        t13 = t7+t9+1.6E+1;
        t12 = t11*2.25E+2;
        t15 = t13*t13;
        t14 = -t12;
        t16 = -t15;
        t17 = t12+t15;
        t18 = 1.0/sqrt(t17);
        t19 = t14+t16+3.136E+3;
        t20 = sqrt(t19);
        t0 = atan2(t4*(3.0/8.0E+1)+t5*(3.0/8.0E+1)+(t18*t20*(t2*(-3.0/4.0E+1)+t3*(3.0/4.0E+1)+2.0/2.5E+1))/2.0,t2*(3.0/8.0E+1)+t3*(3.0/8.0E+1)+(t18*t20*(t4*(3.0/4.0E+1)-t5*(3.0/4.0E+1)))/2.0);
        *phi0 = t0;
    }

}
