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
    // ��λʱ���ڲ������ӵ�ת��: ÿ��1000/MOVE_CTRL_RATE ����һ�����ӵ�ת��
    if(HAL_GetTick() - vel_update_time > (1000/MOVE_CTRL_RATE)){
        vel_update_time = HAL_GetTick();

        // ��ȡ��������ǰ������
        short count = this->encoder->read();
        /**
         * �����ٶ� m/s:
         *      count/һȦ������ => Ȧ��
         *  Ȧ��ת�ɾ���:   Ȧ�� * ���ӵ��ܳ� =  Ȧ�� * ( 3.14 * ���ӵ�ֱ��)  ==> m  D
         *  ��λʱ��:   1000/MOVE_CTRL_RATE ms
         *  �ٶ�:    D/��λʱ�� ===>  D/1000/MOVE_CTRL_RATE  m/ms ===> m/s
         *                           D*MOVE_CTRL_RATE/1000 ===> D*MOVE_CTRL_RATE
         *         Ȧ�� * ( 3.14 * ���ӵ�ֱ��)*MOVE_CTRL_RATE
         *         count/һȦ������ * ( 3.14 * ���ӵ�ֱ��)*MOVE_CTRL_RATE
         */
        uint32_t curr_time = HAL_GetTick();
        uint32_t unit_time = curr_time - last_calc_time;
        vel = (double)count/WHEEL_TPR*PI*WHEEL_DIAMETER/((double)unit_time/1000.0);

        last_calc_time = curr_time;
        /*
         * pid �����߼�
         * */
        float pwm = this->pid->compute(this->targetVel,this->vel);
        //printf("vel=%f tar=%f  count=%d pwm=%f \r\n",vel,targetVel,count,pwm);
        this->motor->spin(pwm);
    }
}

void Wheel::setVel(float vel){
    // Ҫ������ת����ָ����Ŀ���ٶ�: PID  �ջ�����, ���ӵ�ǰ���ٶ�
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