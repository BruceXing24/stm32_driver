//
// Created by Kaijun on 2020/9/11.
//
#include "robot.h"
#include <cstdio>
#include "stm32f1xx_hal.h"
#include "PSTwo.h"
#include "config.h"
#include "oled.h"
#include "mpu9250.h"
#include "Wheel4Car.h"
#include "usart.h"
#include "common_uart.h"
#include "protocol.h"
#include "Wheel2Car.h"
#include "Ackermann.h"
#include "crc_code.h"

void show_oled();
short param_kp = KP*10;
short param_ki = KI*10;
short param_kd = KD*10;
// 创建手柄对象
PSTwo psTwo(PS_DI_PORT,PS_DI_PIN,PS_DO_PORT,PS_DO_PIN,PS_CS_PORT,PS_CS_PIN,PS_CLK_PORT,PS_CLK_PIN,PS_TIM);
//Wheel2Car car;
Ackermann car;
//Wheel4Car car;
uint8_t buffer[10]={0};

// 将下位机的所有数据发送出去
void publish_data();
// 数据每隔一段时间对外发送
uint32_t publish_time;

/**
 * 初始化的操作
 */
void HeimarobotInit(){
    printf("heimarobot init..\r\n");

    // 初始化串口通讯
    common_uart_init();

    //psTwo.init();
    // 初始化MPU_9250
    MPU9250_Init();

    // 初始化OLED
    OLED_Init();

    car.init();



    HAL_UART_Receive_IT(&huart2,buffer,1);

//    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
//    HAL_Delay(1000);
//    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
}



int flag=1;
int32_t change_time = 0;
/**
 * Tick机器人心脏的跳动
 */
void HeimarobotTick(){
//    printf("heimarobot tick..\r\n");
 /*   printf("protocol=%d \r\n", sizeof(TXProtocol));
    HAL_Delay(500);
*/


    car.tick();

//    car.updateVel(-0.1,0,0);

    publish_data();
  /*  uint8_t data[] = {0x01,0x02,0x03};
    common_uart_send(data, sizeof(data));*/

    //psTwo.tick();

    show_oled();
}

void show_oled(){
    if(car.carType == 0){
        uint8_t data1[]="akerman";
        OLED_ShowString(0,0,data1);
    }else if(car.carType == 1){
        uint8_t data1[]="wheel2";
        OLED_ShowString(0,0,data1);
    }else if(car.carType == 2){
        uint8_t data1[]="wheel4";
        OLED_ShowString(0,0,data1);
    }

    uint8_t data2[]="heimarobot";
    OLED_ShowString(0,12,data2);
    uint8_t data3[]="robot.czxy.com";
    OLED_ShowString(0,24,data3);


    // 显示100倍的PID
    OLED_ShowString(0,48,(uint8_t*)"P:");
    OLED_ShowNumber(12,48,param_kp,3,12);

    OLED_ShowString(36,48,(uint8_t*)"I:");
    OLED_ShowNumber(48,48,param_ki,3,12);

    OLED_ShowString(72,48,(uint8_t*)"D:");
    OLED_ShowNumber(84,48,param_kd,3,12);


    // 刷新屏幕
    OLED_Refresh_Gram();
}





void publish_data(){
    // 按照一定的频率对外发送数据
    if(HAL_GetTick() - publish_time < 1000/IMU_PUSH_RATE){
        return ;
    }
    publish_time = HAL_GetTick();

    // 温度
    short temp = MPU_Get_Temperature();
    // 三轴加速度
    short ax,ay,az = 0;
    MPU_Get_Accelerometer(&ax,&ay,&az);
    // 三轴角速度
    short gx,gy,gz = 0;
    MPU_Get_Gyroscope(&gx,&gy,&gz);
    // 三轴地磁数据
    short mx,my,mz = 0;
    MPU_Get_Magnetometer(&mx,&my,&mz);

    // 将小车的线速度 ,角速度  放大1000倍的目的是为了减少数据量 , 方便处理
    short xVel = (short)(car.getXVel()*1000);
    short angularVel = (short)(car.getAngularVel()*1000);

    uint8_t protocolSize = sizeof(TXProtocol);


    struct TXProtocol* protocol = new TXProtocol;
    protocol->head0 = FLAG_HEAD0;
    protocol->head1 = FLAG_HEAD1;
    protocol->type = 0x03;
    protocol->len = protocolSize - 4;
    // 温度
    protocol->temperature = temp;
    // 加速度
    protocol->ax = ax;
    protocol->ay = ay;
    protocol->az = az;
    // 陀螺仪的角速度
    protocol->gx = gx;
    protocol->gy = gy;
    protocol->gz = gz;
    // 地磁
    protocol->mx = mx;
    protocol->my = my;
    protocol->mz = mz;

    // 线速度和角速度
    protocol->velocity = xVel;
    protocol->angular = angularVel;


    protocol->tail = FLAG_TAIL;

    //  将速度发送出去
    common_uart_send((uint8_t*)protocol,protocolSize);

    // 切记 new 出来的东西一定要删除
    delete protocol;
}





/**
 * 接收外部发送过来的数据
 * @param receive_buf
 * @param receive_len
 */
void common_uart_idle_callback(uint8_t receive_buf[],uint16_t receive_len){

    //                类  长  低   高  低  高
    // [0xce, 0xfa, 0x05, 4, 250, 0, 244, 1]
    uint8_t i = 0;
    while(i < receive_len){
        // 先找到帧头0
        if(receive_buf[i] == FLAG_HEAD0){
            // 再找帧头1
            if(receive_buf[i+1] == FLAG_HEAD1){
                // 若进到这里来了,说明帧头完全匹配
                // 获取类型
                uint8_t type = receive_buf[i+2];
                if(type == 0x05){  // 判断是否为上位机发送过来的速度信息
                    // 获取数据长度
                    uint8_t len = receive_buf[i+3];
                    // 判断是否有足够多的数据, 为了防止获取数据越界
                    if( len < receive_len - i -3 ){

                        // 还原线速度
                        uint8_t vel_low = receive_buf[i+4];
                        uint8_t vel_high = receive_buf[i+5];
                        short vel = vel_high<<8|vel_low;
                        float velocity = (float)vel/1000;
                        // 还原角速度
                        uint8_t angular_low = receive_buf[i+6];
                        uint8_t angular_high = receive_buf[i+7];
                        short ang = angular_high<<8|angular_low;
                        float angular = (float)ang/1000;
                        // 为了表示数据在传输

                        // 为了表示数据在传输
//                        HAL_GPIO_TogglePin(BUZZER_PORT,BUZZER_PIN);
                        if(receive_buf[i+8] == 0xad) {
                            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
                            car.updateVel(velocity, 0, angular);
                        }
                    }
                }else if(type == 0x06){
                    // 获取数据长度
                    uint8_t len = receive_buf[i+3];
                    // 判断是否有足够多的数据, 为了防止获取数据越界
                    if( len < receive_len - i -3 ) {

                        float ratio = 10;
                        // 还原KP
                        uint8_t kp_l = receive_buf[i + 4];
                        uint8_t kp_h = receive_buf[i + 5];
                        short p = kp_h << 8 | kp_l;
                        float kp = (float) p / ratio;
                        // 还原KI
                        uint8_t ki_l = receive_buf[i + 6];
                        uint8_t ki_h = receive_buf[i + 7];
                        short ki_ = ki_h << 8 | ki_l;
                        float ki = (float) ki_ / ratio;

                        // 还原KI
                        uint8_t kd_l = receive_buf[i + 8];
                        uint8_t kd_h = receive_buf[i + 9];
                        unsigned short kd_ = kd_h << 8 | kd_l;
                        // 为了表示数据在传输

                        float kd = (float) kd_ / ratio;
                        // 为了表示数据在传输
                        if(receive_buf[i+10] == 0xad){
                            car.updatePID(kp,ki,kd);
                            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
                            param_kp = kp;
                            param_ki = ki;
                            param_kd = kd;
                        }
                    }
                }
            }
        }

        i++;
    }

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 判断串口1的中断处理逻辑
    if(huart->Instance == USART2){
        // 切换led灯的状态
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_10);
        uint8_t key = buffer[0];

        switch (key){
            case 0x47:
                car.updateVel(0.2,0,0);
                break;
            case 0x48:
                car.updateVel(0,0,0.8);
                break;
            case 0x49:
                car.updateVel(0,0,0);
                break;
            case 0x4A:
                car.updateVel(0,0,-0.8);
                break;
            case 0x4B:
                car.updateVel(-0.2,0,0);
                break;

            // 麦克娜姆轮
            case 0x41:
                car.updateVel(0.2,0.2,0); // 左前方
                break;
            case 0x42:
                car.updateVel(0,0.2,0); // 水平向左
                break;
            case 0x43:
                car.updateVel(0.2,-0.2,0); // 右前方
                break;
            case 0x44:
                car.updateVel(-0.2,0.2,0); // 左后方
                break;
            case 0x45:
                car.updateVel(0,-0.2,0); // 水平向右
                break;
            case 0x46:
                car.updateVel(-0.2,-0.2,0); // 右后方
                break;
            default:
                car.updateVel(0,0,0);
                break;
        }
        HAL_UART_Receive_IT(&huart2,buffer,1);
    }
}
