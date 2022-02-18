//
// Created by Kaijun on 2020/9/11.
//

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
    // ���µ�ʱ��
    uint32_t vel_update_time;
    float vel;
    float targetVel;

    uint32_t last_calc_time;
public:
    void init();
    void tick();
    /**
     * �������ӵ�ת��
     * @param vel  ��λm/s
     */
    void setVel(float vel);
    float getVel();
};


#endif //HEIMAROBOTV4_WHEEL_H
