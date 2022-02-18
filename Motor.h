#ifndef HEIMAROBOTV4_MOTOR_H
#define HEIMAROBOTV4_MOTOR_H

#include "stm32f1xx_hal.h"

class Motor {
public:
    /**
     * 电机
     * @param portA  控制正反转
     * @param pinA 控制正反转
     * @param portB 控制正反转
     * @param pinB 控制正反转
     * @param tim   控制PWM的
     * @param channel   定时器通道
     * @param direction  后续方便调整电机的转动方向
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
