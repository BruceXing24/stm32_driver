#ifndef HEIMAROBOTV4_PID_H
#define HEIMAROBOTV4_PID_H


class PID {
public:
    PID(float kp,float ki,float kd);
    float kp;
    float ki;
    float kd;

    float pwm = 0;
    // 上一次的误差
    float prevError;
    // 积分
    float intergral;
    // 微分
    float derivative;
    /**
     * pid的计算函数
     * @param target  目标值
     * @param current 当前值
     * @return  pwm
     */
    float compute(float target,float current);
    /**
     *  重置所有的误差: 当设置的速度 和 上一次不一样
     */
    void reset();

    void update(float kp,float ki,float kd);
};


#endif //HEIMAROBOTV4_PID_H
