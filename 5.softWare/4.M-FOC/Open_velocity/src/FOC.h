#ifndef FOC_H
#define FOC_H

class FOC
{
private:
    float u_alpha;
    float u_beta;

    float Udc = 12.5; // 输入电压
    float Ts = 1.0;   // SVPWM的采样周期，设置为1.0，这样输出的Ta、Tb、Tc即为占空比

    float t1;
    float t2;
    float t3;
    float t4;
    float t5;
    float t6;
    float t7;
    float sum;

public:
    void Inv_Park(float u_q, float u_d, float theta); // Park逆变换
    void SVPWM();                                     // 计算矢量作用时间

    float Ta; // PWM占空比
    float Tb;
    float Tc;

    void Open_velocity(); // 开环速度测试
};

#endif