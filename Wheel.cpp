//
// Created by Kaijun on 2020/9/11.
//

#include "Wheel.h"
#include "config.h"
#include <cstdio>

Wheel::Wheel(Motor* motor, Encoder* encoder) {
    this->motor = motor;
    this->encoder = encoder;
    this->pid = new PID(500,0.1,150);
    this->targetVel = 0;
}

void Wheel::init(){
    this->encoder->init();
    this->motor->init();
}
void Wheel::tick(){
    // 单位时间内测量轮子的转速: 每隔1000/MOVE_CTRL_RATE 计算一次轮子的转速
    if(HAL_GetTick() - vel_update_time > (1000/MOVE_CTRL_RATE)){
        vel_update_time = HAL_GetTick();

        // 读取编码器当前的数据
        short count = this->encoder->read();
        /**
         * 计算速度 m/s:
         *      count/一圈的数量 => 圈数
         *  圈数转成距离:   圈数 * 轮子的周长 =  圈数 * ( 3.14 * 轮子的直径)  ==> m  D
         *  单位时间:   1000/MOVE_CTRL_RATE ms
         *  速度:    D/单位时间 ===>  D/1000/MOVE_CTRL_RATE  m/ms ===> m/s
         *                           D*MOVE_CTRL_RATE/1000 ===> D*MOVE_CTRL_RATE
         *         圈数 * ( 3.14 * 轮子的直径)*MOVE_CTRL_RATE
         *         count/一圈的数量 * ( 3.14 * 轮子的直径)*MOVE_CTRL_RATE
         */
        uint32_t curr_time = HAL_GetTick();
        uint32_t unit_time = curr_time - last_calc_time;
        vel = (double)count/WHEEL_TPR*PI*WHEEL_DIAMETER/((double)unit_time/1000.0);

        last_calc_time = curr_time;
        /*
         * pid 控制逻辑
         * */
        float pwm = this->pid->compute(this->targetVel,this->vel);
        //printf("vel=%f tar=%f  count=%d pwm=%f \r\n",vel,targetVel,count,pwm);
        this->motor->spin(pwm);
    }
}

void Wheel::setVel(float vel){
    // 要让轮子转动到指定的目标速度: PID  闭环控制, 轮子当前的速度
    if(vel != this->targetVel){
        this->pid->reset();
    }

    this->targetVel = vel;
}
float Wheel::getVel(){
    return 0;
}

Wheel::~Wheel() {
    delete this->motor;
    delete this->encoder;
    delete this->pid;
}