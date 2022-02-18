//
// Created by Kaijun on 2020/9/11.
//

#ifndef HEIMAROBOTV4_WHEEL4CAR_H
#define HEIMAROBOTV4_WHEEL4CAR_H

#include "Wheel.h"

class Wheel4Car {
public:
    Wheel4Car();
    Wheel* wheel1;
    Wheel* wheel2;
    Wheel* wheel3;
    Wheel* wheel4;
    ~Wheel4Car();
    uint8_t carType = 2;
public:
    void init();
    void tick();

    void updateVel(float xVel,float yVel,float angularVel);
    void updatePID(float kp,float ki,float kd);
    float getXVel();
    float getYVel();
    float getAngularVel();

};


#endif //HEIMAROBOTV4_WHEEL4CAR_H
