//
// Created by Kaijun on 2020/9/11.
//

#include "Encoder.h"
Encoder::Encoder(TIM_HandleTypeDef *tim,uint16_t channel,int direction){
    this->tim = tim;
    this->channel = channel;
    this->direction = direction;
}

void Encoder::init(){
    // ������ʱ��
    HAL_TIM_Encoder_Start(this->tim,this->channel);
}

short Encoder::read(){
    // ��ȡ����������
    short count = __HAL_TIM_GET_COUNTER(this->tim);
    // ͨ��direction�����������ķ���
    count *= this->direction;
    // ����֮���������
    __HAL_TIM_SET_COUNTER(this->tim,0);
    return count;
}