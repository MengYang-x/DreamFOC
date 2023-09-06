#ifndef FOC_H
#define FOC_H

class FOC
{
private:
public:
    float u_d;
    float u_q;
    float theta;

    float u_alpha;
    float u_beta;

    void Inv_Park(); // Park逆变换
    void SVPWM();    // 计算矢量作用时间

    float Udc; // 输入电压
    float Ts;  // PWM的周期

    float Ta; // PWM占空比
    float Tb;
    float Tc;
};

#endif