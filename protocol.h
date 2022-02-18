//
// Created by Kaijun on 2020/9/13.
//

#ifndef HEIMAROBOTV4_PROTOCOL_H
#define HEIMAROBOTV4_PROTOCOL_H

#include <stdint.h>
#define FLAG_HEAD0 0xce
#define FLAG_HEAD1 0xfa
#define FLAG_TAIL  0xad

//CE FA 00 00 4B 0E D8 02 C8 FE 58
//3C E9 FF 91 00 8A 00 00 91 FF E9
//00 8A F4 01 20 03 AD


#pragma pack(1)
struct TXProtocol{
    uint8_t head0;
    uint8_t head1;
    uint8_t type;
    uint8_t len;

    short temperature;

    short ax;
    short ay;
    short az;

    short gx;
    short gy;
    short gz;

    short mx;
    short my;
    short mz;

    short velocity;
    short angular;
    uint8_t tail;
};


// 发送的数据协议 // 让数据按1字节进行对齐
/*#pragma pack(1)
typedef struct TXProtocol{
    // 使用两个帧头不那么容易出错
    uint8_t head0;
    uint8_t head1;
    // 定义数据帧的类型: 0x01 速度信息  0x02 陀螺仪信息 0x03 所有信息
    uint8_t type;
    // 数据的长度
    uint8_t length;
    // 温度
    short temperature;
    // 加速度
    short ax;
    short ay;
    short az;
    // 角速度
    short gx;
    short gy;
    short gz;
    // 地磁
    short mx;
    short my;
    short mz;
    // 速度信息
    short velocity;
    short angular;

//    uint8_t tail;
    // 29个字节
};*/


#endif //HEIMAROBOTV4_PROTOCOL_H
