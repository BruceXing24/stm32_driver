
#include "Servo.h"
#include <math.h>


Servo::Servo(TIM_HandleTypeDef *tim, uint32_t channel){
    this->tim  = tim;
    this->channel = channel;
}

Servo::~Servo() {}

void Servo::init() {
    HAL_TIM_PWM_Start(this->tim,this->channel);
//    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    setAngle(0);
}

void Servo::setAngle(double angle) {
    int pwm = SERVO_INIT - angle * SERVO_K;
    setPwm(pwm);
}

void Servo::setPwm(int pwm) {
    //HAL_TIM_PWM_Start(this->tim,this->channel);
    __HAL_TIM_SET_COMPARE(this->tim,this->channel, abs(pwm));
}