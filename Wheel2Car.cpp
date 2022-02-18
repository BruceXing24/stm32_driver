//
// Created by Kaijun on 2020/9/17.
//

#include "Wheel2Car.h"
#include "config.h"


Wheel2Car::Wheel2Car() {
    //初始化3号电机
    Motor* motor3 = new Motor(MOTOR3_A_GPIO_PORT,MOTOR3_A_PIN,MOTOR3_B_GPIO_PORT,MOTOR3_B_PIN,MOTOR3_TIM,MOTOR3_CHANNEL,MOTOR3_DIRECTION);
    //初始化3号编码器
    Encoder* encoder3 = new Encoder(ENCODER3_TIM,ENCODER3_CHANNEL,ENCODER3_DIRECTION);
    // 初始化3号轮子
    wheel3 = new Wheel(motor3,encoder3);

    //初始化4号电机
    Motor* motor4 = new Motor(MOTOR4_A_GPIO_PORT,MOTOR4_A_PIN,MOTOR4_B_GPIO_PORT,MOTOR4_B_PIN,MOTOR4_TIM,MOTOR4_CHANNEL,MOTOR4_DIRECTION);
    //初始化4号编码器
    Encoder* encoder4 = new Encoder(ENCODER4_TIM,ENCODER4_CHANNEL,ENCODER4_DIRECTION);
    // 初始化4号轮子
    wheel4 = new Wheel(motor4,encoder4);

}

void Wheel2Car::init(){

    wheel3->init();
    wheel4->init();


    wheel3->setVel(0.0001);
    wheel4->setVel(0.0001);
}
/**
 * 更新小车速度, 并且分别计算每个轮子的转速
 * @param xVel
 * @param yVel
 * @param angularVel
 */
void Wheel2Car::updateVel(float xVel,float yVel,float angularVel){

    float wd = angularVel*(WHEEL_DISTANCE)*0.5;

    float vel3 = xVel + 0 - wd;

    float vel4 = xVel - 0 + wd;

    wheel3->setVel(vel3);
    wheel4->setVel(vel4);

}

void Wheel2Car::tick(){

    wheel3->tick();
    wheel4->tick();
}


float Wheel2Car::getXVel(){

    float vel3 = wheel3->vel;
    float vel4 = wheel4->vel;

    return (vel3+vel4)/2;
}

float Wheel2Car::getYVel(){
    return 0;
}

float Wheel2Car::getAngularVel(){
    float vel3 = wheel3->vel;
    float vel4 = wheel4->vel;

    float angular = (-vel3 + vel4)/2/(WHEEL_DISTANCE*0.5);
    return angular;
}

void Wheel2Car::updatePID(float kp, float ki, float kd) {
    wheel3->pid->update(kp,ki,kd);
    wheel4->pid->update(kp,ki,kd);
}

Wheel2Car::~Wheel2Car(){
    delete wheel3;
    delete wheel4;
}