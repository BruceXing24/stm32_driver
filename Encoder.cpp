#include "Encoder.h"
Encoder::Encoder(TIM_HandleTypeDef *tim,uint16_t channel,int direction){
    this->tim = tim;
    this->channel = channel;
    this->direction = direction;
}

void Encoder::init(){
    // 启动定时器
    HAL_TIM_Encoder_Start(this->tim,this->channel);
}

short Encoder::read(){
    // 读取编码器数据
    short count = __HAL_TIM_GET_COUNTER(this->tim);
    // 通过direction调整编码器的方向
    count *= this->direction;
    // 读完之后清空数据
    __HAL_TIM_SET_COUNTER(this->tim,0);
    return count;
}
