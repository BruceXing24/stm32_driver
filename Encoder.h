
#ifndef HEIMAROBOTV4_ENCODER_H
#define HEIMAROBOTV4_ENCODER_H

#include "stm32f1xx_hal.h"

class Encoder {
public:
    Encoder(TIM_HandleTypeDef *tim,uint16_t channel,int direction);

    TIM_HandleTypeDef *tim;
    uint16_t channel;
    int direction;

    void init();

    short read();
};


#endif //HEIMAROBOTV4_ENCODER_H
