#include <Arduino.h>
#include "FOC.h"

/**********************************************************************************************************
park逆变换：输入Uq、Ud得到Ualpha、Ubeta
Uα = Ud · cosθ - Uq · sinθ
Uβ = Ud · sinθ + Uq · cosθ
**********************************************************************************************************/
void FOC::Inv_Park(float u_q, float u_d, float theta)
{
    float sinAngle = sin(theta);
    float cosAngle = cos(theta);

    u_alpha = u_d * cosAngle - u_q * sinAngle;
    u_beta = u_d * sinAngle + u_q * cosAngle;
}

/**********************************************************************************************************
Step1：扇区判断：参考Clarke逆变换的公式，计算扇区
sqrt(3)/2 ≈ 0.86603
u1 = Uβ
u2 = 0.86603 * Uα - 0.5 * Uβ
u3 = -0.86603 * Uα - 0.5 * Uβ
N = A + 2B+ 4C
Step2：计算矢量作用时间Ta、Tb、Tc
**********************************************************************************************************/
void FOC::SVPWM()
{
    static float k = 1.73205 * Ts / Udc; // sqrt(3) ≈ 1.73205, K = sqrt(3) * Ts / Udc

    // 1.计算SVPWM算法中三个控制电压
    float u1 = u_beta * k;                              // A
    float u2 = (0.86603 * u_alpha - 0.5 * u_beta) * k;  // B
    float u3 = (-0.86603 * u_alpha - 0.5 * u_beta) * k; // C

    // 2.扇区判断
    uint8_t sector = (u1 > 0.0) + ((u2 > 0.0) << 1) + ((u3 > 0.0) << 2); // sector = A + 2B + 4C

    // 3.计算三相PWM的占空比
    switch (sector)
    {
    case 3: // 扇区1
        t4 = u2;
        t6 = u1;
        sum = t4 + t6;
        if (sum > Ts)
        {
            float k_svpwm = Ts / sum; // 计算缩放系数
            t4 *= k_svpwm;
            t6 *= k_svpwm;
        }
        t7 = (Ts - t4 - t6) / 2;
        Ta = t4 + t6 + t7;
        Tb = t6 + t7;
        Tc = t7;
        break;
    case 1: // 扇区2
        t2 = -u2;
        t6 = -u3;
        sum = t2 + t6;
        if (sum > Ts)
        {
            float k_svpwm = Ts / sum;
            t2 *= k_svpwm;
            t6 *= k_svpwm;
        }
        t7 = (Ts - t2 - t6) / 2;
        Ta = t6 + t7;
        Tb = t2 + t6 + t7;
        Tc = t7;
        break;
    case 5: // 扇区3
        t2 = u1;
        t3 = u3;
        sum = t2 + t3;
        if (sum > Ts)
        {
            float k_svpwm = Ts / sum;
            t2 *= k_svpwm;
            t3 *= k_svpwm;
        }
        t7 = (Ts - t2 - t3) / 2;
        Ta = t7;
        Tb = t2 + t3 + t7;
        Tc = t3 + t7;
        break;
    case 4: // 扇区4
        t1 = -u1;
        t3 = -u2;
        sum = t1 + t3;
        if (sum > Ts)
        {
            float k_svpwm = Ts / sum;
            t1 *= k_svpwm;
            t3 *= k_svpwm;
        }
        t7 = (Ts - t1 - t3) / 2;
        Ta = t7;
        Tb = t3 + t7;
        Tc = t1 + t3 + t7;
        break;
    case 6: // 扇区5
        t1 = u3;
        t5 = u2;
        sum = t1 + t5;
        if (sum > Ts)
        {
            float k_svpwm = Ts / sum;
            t1 *= k_svpwm;
            t5 *= k_svpwm;
        }
        t7 = (Ts - t1 - t5) / 2;
        Ta = t5 + t7;
        Tb = t7;
        Tc = t1 + t5 + t7;
        break;
    case 2: // 扇区6
        t4 = -u3;
        t5 = -u1;
        sum = t4 + t5;
        if (sum > Ts)
        {
            float k_svpwm = Ts / sum;
            t4 *= k_svpwm;
            t5 *= k_svpwm;
        }
        t7 = (Ts - t4 - t5) / 2;
        Ta = t4 + t5 + t7;
        Tb = t7;
        Tc = t5 + t7;
        break;
    default:
        break;
    }
}

void FOC::Open_velocity()
{
    float u_d = 0;
    float u_q = 2;

    for (float theta = 0; theta < 6.2831853f; theta += 0.275f)
    {
        Inv_Park(u_q, u_d, theta);
        SVPWM();
        // 使用vofa进行输出测试
        Serial.printf("%f, %f, %f\n", Ta, Tb, Tc);
        delay(1);
    }
}