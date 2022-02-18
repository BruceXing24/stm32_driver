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


// ���͵�����Э�� // �����ݰ�1�ֽڽ��ж���
/*#pragma pack(1)
typedef struct TXProtocol{
    // ʹ������֡ͷ����ô���׳���
    uint8_t head0;
    uint8_t head1;
    // ��������֡������: 0x01 �ٶ���Ϣ  0x02 ��������Ϣ 0x03 ������Ϣ
    uint8_t type;
    // ���ݵĳ���
    uint8_t length;
    // �¶�
    short temperature;
    // ���ٶ�
    short ax;
    short ay;
    short az;
    // ���ٶ�
    short gx;
    short gy;
    short gz;
    // �ش�
    short mx;
    short my;
    short mz;
    // �ٶ���Ϣ
    short velocity;
    short angular;

//    uint8_t tail;
    // 29���ֽ�
};*/


#endif //HEIMAROBOTV4_PROTOCOL_H
