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
// �����ֱ�����
PSTwo psTwo(PS_DI_PORT,PS_DI_PIN,PS_DO_PORT,PS_DO_PIN,PS_CS_PORT,PS_CS_PIN,PS_CLK_PORT,PS_CLK_PIN,PS_TIM);
//Wheel2Car car;
Ackermann car;
//Wheel4Car car;
uint8_t buffer[10]={0};

// ����λ�����������ݷ��ͳ�ȥ
void publish_data();
// ����ÿ��һ��ʱ����ⷢ��
uint32_t publish_time;

/**
 * ��ʼ���Ĳ���
 */
void HeimarobotInit(){
    printf("heimarobot init..\r\n");

    // ��ʼ������ͨѶ
    common_uart_init();

    //psTwo.init();
    // ��ʼ��MPU_9250
    MPU9250_Init();

    // ��ʼ��OLED
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
 * Tick���������������
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


    // ��ʾ100����PID
    OLED_ShowString(0,48,(uint8_t*)"P:");
    OLED_ShowNumber(12,48,param_kp,3,12);

    OLED_ShowString(36,48,(uint8_t*)"I:");
    OLED_ShowNumber(48,48,param_ki,3,12);

    OLED_ShowString(72,48,(uint8_t*)"D:");
    OLED_ShowNumber(84,48,param_kd,3,12);


    // ˢ����Ļ
    OLED_Refresh_Gram();
}





void publish_data(){
    // ����һ����Ƶ�ʶ��ⷢ������
    if(HAL_GetTick() - publish_time < 1000/IMU_PUSH_RATE){
        return ;
    }
    publish_time = HAL_GetTick();

    // �¶�
    short temp = MPU_Get_Temperature();
    // ������ٶ�
    short ax,ay,az = 0;
    MPU_Get_Accelerometer(&ax,&ay,&az);
    // ������ٶ�
    short gx,gy,gz = 0;
    MPU_Get_Gyroscope(&gx,&gy,&gz);
    // ����ش�����
    short mx,my,mz = 0;
    MPU_Get_Magnetometer(&mx,&my,&mz);

    // ��С�������ٶ� ,���ٶ�  �Ŵ�1000����Ŀ����Ϊ�˼��������� , ���㴦��
    short xVel = (short)(car.getXVel()*1000);
    short angularVel = (short)(car.getAngularVel()*1000);

    uint8_t protocolSize = sizeof(TXProtocol);


    struct TXProtocol* protocol = new TXProtocol;
    protocol->head0 = FLAG_HEAD0;
    protocol->head1 = FLAG_HEAD1;
    protocol->type = 0x03;
    protocol->len = protocolSize - 4;
    // �¶�
    protocol->temperature = temp;
    // ���ٶ�
    protocol->ax = ax;
    protocol->ay = ay;
    protocol->az = az;
    // �����ǵĽ��ٶ�
    protocol->gx = gx;
    protocol->gy = gy;
    protocol->gz = gz;
    // �ش�
    protocol->mx = mx;
    protocol->my = my;
    protocol->mz = mz;

    // ���ٶȺͽ��ٶ�
    protocol->velocity = xVel;
    protocol->angular = angularVel;


    protocol->tail = FLAG_TAIL;

    //  ���ٶȷ��ͳ�ȥ
    common_uart_send((uint8_t*)protocol,protocolSize);

    // �м� new �����Ķ���һ��Ҫɾ��
    delete protocol;
}





/**
 * �����ⲿ���͹���������
 * @param receive_buf
 * @param receive_len
 */
void common_uart_idle_callback(uint8_t receive_buf[],uint16_t receive_len){

    //                ��  ��  ��   ��  ��  ��
    // [0xce, 0xfa, 0x05, 4, 250, 0, 244, 1]
    uint8_t i = 0;
    while(i < receive_len){
        // ���ҵ�֡ͷ0
        if(receive_buf[i] == FLAG_HEAD0){
            // ����֡ͷ1
            if(receive_buf[i+1] == FLAG_HEAD1){
                // ��������������,˵��֡ͷ��ȫƥ��
                // ��ȡ����
                uint8_t type = receive_buf[i+2];
                if(type == 0x05){  // �ж��Ƿ�Ϊ��λ�����͹������ٶ���Ϣ
                    // ��ȡ���ݳ���
                    uint8_t len = receive_buf[i+3];
                    // �ж��Ƿ����㹻�������, Ϊ�˷�ֹ��ȡ����Խ��
                    if( len < receive_len - i -3 ){

                        // ��ԭ���ٶ�
                        uint8_t vel_low = receive_buf[i+4];
                        uint8_t vel_high = receive_buf[i+5];
                        short vel = vel_high<<8|vel_low;
                        float velocity = (float)vel/1000;
                        // ��ԭ���ٶ�
                        uint8_t angular_low = receive_buf[i+6];
                        uint8_t angular_high = receive_buf[i+7];
                        short ang = angular_high<<8|angular_low;
                        float angular = (float)ang/1000;
                        // Ϊ�˱�ʾ�����ڴ���

                        // Ϊ�˱�ʾ�����ڴ���
//                        HAL_GPIO_TogglePin(BUZZER_PORT,BUZZER_PIN);
                        if(receive_buf[i+8] == 0xad) {
                            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
                            car.updateVel(velocity, 0, angular);
                        }
                    }
                }else if(type == 0x06){
                    // ��ȡ���ݳ���
                    uint8_t len = receive_buf[i+3];
                    // �ж��Ƿ����㹻�������, Ϊ�˷�ֹ��ȡ����Խ��
                    if( len < receive_len - i -3 ) {

                        float ratio = 10;
                        // ��ԭKP
                        uint8_t kp_l = receive_buf[i + 4];
                        uint8_t kp_h = receive_buf[i + 5];
                        short p = kp_h << 8 | kp_l;
                        float kp = (float) p / ratio;
                        // ��ԭKI
                        uint8_t ki_l = receive_buf[i + 6];
                        uint8_t ki_h = receive_buf[i + 7];
                        short ki_ = ki_h << 8 | ki_l;
                        float ki = (float) ki_ / ratio;

                        // ��ԭKI
                        uint8_t kd_l = receive_buf[i + 8];
                        uint8_t kd_h = receive_buf[i + 9];
                        unsigned short kd_ = kd_h << 8 | kd_l;
                        // Ϊ�˱�ʾ�����ڴ���

                        float kd = (float) kd_ / ratio;
                        // Ϊ�˱�ʾ�����ڴ���
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
    // �жϴ���1���жϴ����߼�
    if(huart->Instance == USART2){
        // �л�led�Ƶ�״̬
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

            // �����ķ��
            case 0x41:
                car.updateVel(0.2,0.2,0); // ��ǰ��
                break;
            case 0x42:
                car.updateVel(0,0.2,0); // ˮƽ����
                break;
            case 0x43:
                car.updateVel(0.2,-0.2,0); // ��ǰ��
                break;
            case 0x44:
                car.updateVel(-0.2,0.2,0); // ���
                break;
            case 0x45:
                car.updateVel(0,-0.2,0); // ˮƽ����
                break;
            case 0x46:
                car.updateVel(-0.2,-0.2,0); // �Һ�
                break;
            default:
                car.updateVel(0,0,0);
                break;
        }
        HAL_UART_Receive_IT(&huart2,buffer,1);
    }
}
