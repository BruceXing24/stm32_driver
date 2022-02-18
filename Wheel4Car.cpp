//
// Created by Kaijun on 2020/9/11.
//

#include "Wheel4Car.h"
#include "config.h"

/**
 * 有一台车,四个轮子,每个轮子包含一个电机和一个编码器
 * 车的对象:Car
 *            输入目标速度
 *            输出当前速度
 * 轮子的对象: Wheel
 *        电机: Motor  : 控制电机转动
 *        编码器: Encoder : 读取编码器的数据
 *  Car
 *     Wheel
 *         motor
 *         encoder
 *
 */
Wheel4Car::Wheel4Car() {
    //初始化1号电机
    Motor* motor1 = new Motor(MOTOR1_A_GPIO_PORT,MOTOR1_A_PIN,MOTOR1_B_GPIO_PORT,MOTOR1_B_PIN,MOTOR1_TIM,MOTOR1_CHANNEL,MOTOR1_DIRECTION);
    //初始化1号编码器
    Encoder* encoder1 = new Encoder(ENCODER1_TIM,ENCODER1_CHANNEL,ENCODER1_DIRECTION);
    // 初始化1号轮子
    wheel1 = new Wheel(motor1,encoder1);

    //初始化2号电机
    Motor* motor2 = new Motor(MOTOR2_A_GPIO_PORT,MOTOR2_A_PIN,MOTOR2_B_GPIO_PORT,MOTOR2_B_PIN,MOTOR2_TIM,MOTOR2_CHANNEL,MOTOR2_DIRECTION);
    //初始化2号编码器
    Encoder* encoder2 = new Encoder(ENCODER2_TIM,ENCODER2_CHANNEL,ENCODER2_DIRECTION);
    // 初始化2号轮子
    wheel2 = new Wheel(motor2,encoder2);

    //初始化3号电机
    Motor* motor3 = new Motor(MOTOR3_A_GPIO_PORT,MOTOR3_A_PIN,MOTOR3_B_GPIO_PORT,MOTOR3_B_PIN,MOTOR3_TIM,MOTOR3_CHANNEL,MOTOR3_DIRECTION);
    //初始化3号编码器
    Encoder* encoder3 = new Encoder(ENCODER3_TIM,ENCODER3_CHANNEL,ENCODER3_DIRECTION);
    // 初始化3号轮子
    wheel3 = new Wheel(motor3,encoder3);

    //初始化3号电机
    Motor* motor4 = new Motor(MOTOR4_A_GPIO_PORT,MOTOR4_A_PIN,MOTOR4_B_GPIO_PORT,MOTOR4_B_PIN,MOTOR4_TIM,MOTOR4_CHANNEL,MOTOR4_DIRECTION);
    //初始化3号编码器
    Encoder* encoder4 = new Encoder(ENCODER4_TIM,ENCODER4_CHANNEL,ENCODER4_DIRECTION);
    // 初始化3号轮子
    wheel4 = new Wheel(motor4,encoder4);

}

void Wheel4Car::init(){
    wheel1->init();
    wheel2->init();
    wheel3->init();
    wheel4->init();

    wheel1->setVel(0.0001);
    wheel2->setVel(0.0001);
    wheel3->setVel(0.0001);
    wheel4->setVel(0.0001);
}
/**
 * 更新小车速度, 并且分别计算每个轮子的转速
 * @param xVel
 * @param yVel
 * @param angularVel
 */
void Wheel4Car::updateVel(float xVel,float yVel,float angularVel){

    float wd = angularVel*(WHEEL_DISTANCE+WHEEL_AXIS)*0.5;
    float vel1 = xVel - yVel - wd;
    float vel3 = xVel + yVel - wd;
    float vel2 = xVel + yVel + wd;
    float vel4 = xVel - yVel + wd;

    wheel1->setVel(vel1);
    wheel2->setVel(vel2);
    wheel3->setVel(vel3);
    wheel4->setVel(vel4);

}

void Wheel4Car::tick(){
    wheel1->tick();
    wheel2->tick();
    wheel3->tick();
    wheel4->tick();
}


float Wheel4Car::getXVel(){
    float vel1 = wheel1->vel;
    float vel2 = wheel2->vel;
    float vel3 = wheel3->vel;
    float vel4 = wheel4->vel;

    return (vel1+vel2+vel3+vel4)/4;
}

float Wheel4Car::getYVel(){
    return 0;
}

float Wheel4Car::getAngularVel(){
    float vel1 = wheel1->vel;
    float vel2 = wheel2->vel;
    float vel3 = wheel3->vel;
    float vel4 = wheel4->vel;

    float angular = (-vel1-vel3 + vel2 + vel4)/4/((WHEEL_AXIS+WHEEL_DISTANCE)*0.5);
    return angular;
}

void Wheel4Car::updatePID(float kp, float ki, float kd) {
    wheel1->pid->update(kp,ki,kd);
    wheel2->pid->update(kp,ki,kd);
    wheel3->pid->update(kp,ki,kd);
    wheel4->pid->update(kp,ki,kd);
}
Wheel4Car::~Wheel4Car(){
    delete wheel1;
    delete wheel2;
    delete wheel3;
    delete wheel4;
}