#include "SETTING.H"
#include "USART.H"
#include "IIC.H"
#include "MPU6050.H"
#include "HMC5983.H"
#include "GPS.H"
#include "PWM.H"

int main(){
USART1_Init(115200);//与电脑通信
USART3_Init(115200);//与GPS通信
USART4_Init(115200);//与电机驱动通信
PWM_Init(100,900);// 舵机控制信号
UART_SendData(UART4,0x14);//20速度初始值
printf("USART_Init \r\n");
while(1){//全程无中断介入
	GPS_UpDate();
}
}
