//
// Created by Kaijun on 2020/9/11.
//

#ifndef HEIMAROBOTV4_MOTOR_H
#define HEIMAROBOTV4_MOTOR_H

#include "stm32f1xx_hal.h"

class Motor {
public:
    /**
     * ���
     * @param portA  ��������ת
     * @param pinA ��������ת
     * @param portB ��������ת
     * @param pinB ��������ת
     * @param tim   ����PWM��
     * @param channel   ��ʱ��ͨ��
     * @param direction  ����������������ת������
     */
    Motor(GPIO_TypeDef *portA, uint16_t pinA,GPIO_TypeDef *portB, uint16_t pinB, TIM_HandleTypeDef *tim,uint16_t channel,int direction);
    // A_PORT A_PIN
    GPIO_TypeDef *portA;
    uint16_t pinA;
    // B_PORT B_PIN
    GPIO_TypeDef *portB;
    uint16_t pinB;
    // TIM
    TIM_HandleTypeDef *tim;

    // CHANNEL
    uint16_t channel;
    int direction;


    void spin(int pwm);
    void init();
    int lastPwm;
//    void tick();
};


#endif //HEIMAROBOTV4_MOTOR_H
