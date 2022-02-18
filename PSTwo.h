//
// Created by Kaijun on 2020/9/11.
//

#ifndef ZXCAR_ZET6_PSTWO_H
#define ZXCAR_ZET6_PSTWO_H

#include "stm32f1xx_hal.h"
#include "tim.h"
#include <stdio.h>


#ifdef __cplusplus

class PSTwo {
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

#define PSS_RX 5                //右摇杆X轴数据
#define PSS_RY 6                //左摇杆Y轴数据
#define PSS_LX 7                //右摇杆X轴数据
#define PSS_LY 8                //左摇杆Y轴数据

public:
    PSTwo(){};
    PSTwo(GPIO_TypeDef *di_port, uint16_t di_pin,
          GPIO_TypeDef *do_port, uint16_t do_pin,
          GPIO_TypeDef *cs_port, uint16_t cs_pin,
          GPIO_TypeDef *clk_port, uint16_t clk_pin,
          TIM_HandleTypeDef *tim);

    ~PSTwo();

private:
    GPIO_TypeDef *di_port;
    uint16_t di_pin;

    GPIO_TypeDef *do_port;
    uint16_t do_pin;

    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    GPIO_TypeDef *clk_port;
    uint16_t clk_pin;

    TIM_HandleTypeDef *tim;

    uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    uint16_t mask[16] = {
            PSB_SELECT,
            PSB_L3,
            PSB_R3,
            PSB_START,
            PSB_PAD_UP,
            PSB_PAD_RIGHT,
            PSB_PAD_DOWN,
            PSB_PAD_LEFT,
            PSB_L2,
            PSB_R2,
            PSB_L1,
            PSB_R1,
            PSB_GREEN,
            PSB_RED,
            PSB_BLUE,
            PSB_PINK
    };    //按键值与按键明

    // 左侧摇杆的值
    uint8_t joystickLeftX;
    uint8_t joystickLeftY;
    // 右侧摇杆的值
    uint8_t joystickRightX;
    uint8_t joystickRightY;
    // 被操控的按钮的值
    uint8_t button;

    bool isStart = false;



    uint32_t update_time;


private:
    bool isDIState();

    void setDO(bool high);

    void setCLK(bool high);

    void setCS(bool high);

    void execCmd(uint8_t cmd);

    void shortPoll();

    void enterConfig();

    void exitConfig();

    void turnOnVibrationMode();

    void turnOnAnalogMode();

    void delay(uint16_t t);

    void clearData();

    void readData();

    uint8_t readButtonData();

    uint8_t readAnalogData(uint8_t button);

public:
    void init();

    void tick();

    bool isRedLightMode();

    void setEnable(bool flag);
    /**
     * 震动
     * @param left： 左侧电机震动
     * @param right：右侧电机震动
     */
    void vibration(uint8_t left, uint8_t right);
};

#endif


#endif //ZXCAR_ZET6_PSTWO_H