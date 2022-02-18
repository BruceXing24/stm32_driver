#ifndef HEIMAROBOTV4_WHEEL_H
#define HEIMAROBOTV4_WHEEL_H

#include "Encoder.h"
#include "Motor.h"
#include "PID.h"

class Wheel {
public:
    Wheel(Motor* motor,Encoder* encoder);
    ~Wheel();
    Motor* motor;
    Encoder* encoder;
    PID* pid;
    // 更新的时间
    uint32_t vel_update_time;
    float vel;
    float targetVel;

    uint32_t last_calc_time;
public:
    void init();
    void tick();
    /**
     * 设置轮子的转速
     * @param vel  单位m/s
     */
    void setVel(float vel);
    float getVel();
};


#endif //HEIMAROBOTV4_WHEEL_H
