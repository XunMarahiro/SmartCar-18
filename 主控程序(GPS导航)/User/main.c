#include "SETTING.H"
#include "USART.H"
#include "IIC.H"
#include "MPU6050.H"
#include "HMC5983.H"
#include "GPS.H"
#include "PWM.H"
#include "vector.h"
int t=65535;
int main(){
	
USART1_Init(115200);//与电脑通信  PA9 PA10
USART3_Init(115200);//与GPS通信  PB10 PB11
USART4_Init(115200);//与电机驱动通信 PC10 PC11 
PWM_Init(100,3000);// 舵机控制信号 PA6
UART_SendData(UART4,0x14);//20速度初始值
printf("Init_Success\r\n");
//GPS_UpDate();
//while(t--){};
while(1){//全程无中断介入
GPS_UpDate();
//	printf("%f\r\n",Find_target(0,0,0,0));
	//printf("%f\r\n",xiangchajiao(112.39795887,34.58948150,112.39795887,0));
}

}
