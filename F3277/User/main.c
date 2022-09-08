#include "SETTING.H"
#include "USART.H"
#include "IIC.H"
#include "MPU6050.H"
#include "HMC5983.H"
#include "GPS.H"
#include "PWM.H"

int main(){
USART1_Init(115200);//与电脑通信  板上串口接口
USART3_Init(115200);//与GPS通信   TX--PB10  RX--PB11
USART4_Init(115200);//与电机驱动通信   TX--PC10  RX--PC11
PWM_Init(100,900);// 舵机控制信号    PA6
	
UART_SendData(UART4,0x20);//32速度初始值
printf("USART_Init \r\n");
while(1){//全程无中断介入
	GPS_UpDate();
}
}
