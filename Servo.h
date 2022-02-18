#ifndef ZXCAR_ZET6_SERVO_H
#define ZXCAR_ZET6_SERVO_H

#include "stm32f1xx_hal.h"
#include "config.h"

#ifdef __cplusplus
class Servo {
public:
    Servo(TIM_HandleTypeDef *tim, uint32_t channel);
    ~Servo();

private:
    TIM_HandleTypeDef *tim;
    uint32_t channel;

public:
    void init();
    /**
     * 控制舵机转向,单位为弧度
     * @param angle
     */
    void setAngle(double angle);

    void setPwm(int pwm);
};
#endif

#endif //ZXCAR_ZET6_SERVO_H
