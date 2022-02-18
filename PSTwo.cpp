//
// Created by Kaijun on 2020/9/11.
//

#include "PSTwo.h"
#include <math.h>
#include <Legacy/stm32_hal_legacy.h>
#include "config.h"

PSTwo::PSTwo(GPIO_TypeDef *di_port, uint16_t di_pin,
             GPIO_TypeDef *do_port, uint16_t do_pin,
             GPIO_TypeDef *cs_port, uint16_t cs_pin,
             GPIO_TypeDef *clk_port, uint16_t clk_pin,
             TIM_HandleTypeDef *tim) :
        di_port(di_port), di_pin(di_pin),
        do_port(do_port), do_pin(do_pin),
        cs_port(cs_port), cs_pin(cs_pin),
        clk_port(clk_port), clk_pin(clk_pin),
        tim(tim) {
}

PSTwo::~PSTwo() {}

void PSTwo::init() {
    shortPoll();
    shortPoll();
    shortPoll();
    enterConfig();
    turnOnVibrationMode();
    turnOnAnalogMode();
    exitConfig();
}

void PSTwo::setEnable(bool flag) {
    isStart = flag;
}

void PSTwo::tick() {
    if(HAL_GetTick() - update_time < PS_CHECK_RATE ){
        return;
    }
    update_time = HAL_GetTick();


    button = readButtonData();

    joystickLeftX = readAnalogData(PSS_LX);
    joystickLeftY = readAnalogData(PSS_LY);
    joystickRightX = readAnalogData(PSS_RX);
    joystickRightY = readAnalogData(PSS_RY);


    if (button == PSB_START && isStart == false) {
        isStart = true;
        // 第一次启动的时候,防止小车飞速
//        joystickLeftX = 128;
//        joystickLeftY = 128;
//        joystickRightX = 128;
//        joystickRightY = 128;
    }
    if (button == PSB_L2) {
        isStart = false;
    }

    if (!isStart) return;

    printf("button:%d lx=%d ly=%d rx=%d ry=%d\r\n",button,joystickLeftX,joystickLeftY,joystickRightX,joystickRightY);

    // 对用的按键进行判断
    switch (button){
        case PSB_PAD_UP:
            HAL_GPIO_WritePin(BUZZER_PORT,BUZZER_PIN,GPIO_PIN_SET);
            break;
        case PSB_PAD_DOWN:
            HAL_GPIO_WritePin(BUZZER_PORT,BUZZER_PIN,GPIO_PIN_RESET);
            break;
    }

}


uint8_t PSTwo::readButtonData() {
    clearData();
    readData();

    uint16_t key = (data[4] << 8) | data[3];     //这是16个按键  按下为0， 未按下为1
    for (int i = 0; i < 16; ++i) {
        if ((key & (1 << (mask[i] - 1))) == 0)
            return i + 1;
    }
    return 0; //没有任何按键按下
}

uint8_t PSTwo::readAnalogData(uint8_t button) {
    return data[button];
}


void PSTwo::readData() {
    setCS(false);
    execCmd(0x01);  //开始命令
    execCmd(0x42);  //请求数据

    for (int i = 2; i < 9; ++i) {
        for (int ref = 0x01; ref < 0x100; ref <<= 1) {
            setCLK(true);
            delay(5);
            setCLK(false);
            delay(5);
            setCLK(true);
            if (isDIState()) {
                data[i] = ref | data[i];
            }
        }
        delay(16);
    }
    setCS(true);
}

void PSTwo::clearData() {
    for (unsigned char &i : data) {
        i = 0x00;
    }
}

bool PSTwo::isRedLightMode() {
    setCS(false);
    execCmd(0x01);//开始命令
    execCmd(0x42);//请求数据
    setCS(true);

    return data[1] != 0x73;
}

void PSTwo::vibration(uint8_t left, uint8_t right) {
    setCS(false);
    delay(16);
    execCmd(0x01);//开始命令
    execCmd(0x42);//请求数据
    execCmd(0x00);
    execCmd(left);
    execCmd(right);
    execCmd(0x00);
    execCmd(0x00);
    execCmd(0x00);
    execCmd(0x00);
    setCS(true);
    delay(16);
}

void PSTwo::shortPoll() {
    setCS(false);
    delay(16);
    execCmd(0x01);//开始命令
    execCmd(0x42);//请求数据
    execCmd(0x00);
    execCmd(0x00);
    execCmd(0x00);
    setCS(true);
    delay(16);
}

void PSTwo::enterConfig() {
    setCS(false);
    delay(16);
    execCmd(0x01);
    execCmd(0x43);
    execCmd(0X00);
    execCmd(0x01);
    execCmd(0x00);
    execCmd(0X00);
    execCmd(0X00);
    execCmd(0X00);
    execCmd(0X00);
    setCS(true);
    delay(16);
}

void PSTwo::exitConfig() {
    setCS(false);
    delay(16);
    execCmd(0x01);
    execCmd(0x43);
    execCmd(0X00);
    execCmd(0x00);
    execCmd(0x5A);
    execCmd(0x5A);
    execCmd(0x5A);
    execCmd(0x5A);
    execCmd(0x5A);
    setCS(true);
    delay(16);
}

void PSTwo::turnOnVibrationMode() {
    setCS(false);
    delay(16);
    execCmd(0x01);
    execCmd(0x4D);
    execCmd(0X00);
    execCmd(0x00);
    execCmd(0X01);
    setCS(true);
    delay(16);
}

void PSTwo::turnOnAnalogMode() {
    setCS(false);
    execCmd(0x01);
    execCmd(0x44);
    execCmd(0X00);
    execCmd(0x01); //analog=0x01;digital=0x00  软件设置发送模式
    execCmd(0x03); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
    //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
    execCmd(0X00);
    execCmd(0X00);
    execCmd(0X00);
    execCmd(0X00);
    setCS(true);
    delay(16);
}

void PSTwo::setDO(bool high) {
    HAL_GPIO_WritePin(do_port, do_pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void PSTwo::setCLK(bool high) {
    HAL_GPIO_WritePin(clk_port, clk_pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void PSTwo::setCS(bool high) {
    HAL_GPIO_WritePin(cs_port, cs_pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void PSTwo::execCmd(uint8_t cmd) {
    volatile uint16_t ref = 0x01;
    data[1] = 0;
    for (ref = 0x01; ref < 0x0100; ref <<= 1) {
        if (ref & cmd) {
            setDO(true);                   //输出一位控制位
        } else {
            setDO(false);
        }

        setCLK(true);                        //时钟拉高
        delay(5);
        setCLK(false);
        delay(5);
        setCLK(true);
        if (isDIState()) {
            data[1] = ref | data[1];
        }
    }
    delay(16);
}

void PSTwo::delay(uint16_t t) {
    uint16_t differ = 0xffff - t - 5;
    /*为防止因中断打断延时，造成计数错误.
     如从0xfffE开始延时1us,但由于中断打断
    （此时计数器仍在计数），本因计数至0xffff）
    便停止计数，但由于错过计数值，并重载arr值，
    导致实际延时(0xffff+1)t
    */
    HAL_TIM_Base_Start(tim);
    __HAL_TIM_SetCounter(tim, differ);

    while (differ < 0xffff - 5) {
        differ = __HAL_TIM_GetCounter(tim);
    }

    HAL_TIM_Base_Stop(tim);
}

bool PSTwo::isDIState() {
    return HAL_GPIO_ReadPin(di_port, di_pin) == GPIO_PIN_SET;
}
