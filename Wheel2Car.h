//
// Created by Kaijun on 2020/9/17.
//

#ifndef HEIMAROBOTV4_WHEEL2CAR_H
#define HEIMAROBOTV4_WHEEL2CAR_H


#include "Wheel.h"

class Wheel2Car {
public:
    Wheel2Car();
    ~Wheel2Car();

    Wheel* wheel3;
    Wheel* wheel4;
    uint8_t carType = 1;

public:
    void init();
    void tick();

    void updateVel(float xVel,float yVel,float angularVel);
    void updatePID(float kp,float ki,float kd);
    float getXVel();
    float getYVel();
    float getAngularVel();

};
#endif //HEIMAROBOTV4_WHEEL2CAR_H
