#include "SETTING.H"
#include "USART.H"
#include "IIC.H"
#include "MPU6050.H"
#include "HMC5983.H"
#include "GPS.H"
#include "PWM.H"

int main(){
USART1_Init(115200);//�����ͨ��  ���ϴ��ڽӿ�
USART3_Init(115200);//��GPSͨ��   TX--PB10  RX--PB11
USART4_Init(115200);//��������ͨ��   TX--PC10  RX--PC11
PWM_Init(100,900);// ��������ź�    PA6
	
UART_SendData(UART4,0x20);//32�ٶȳ�ʼֵ
printf("USART_Init \r\n");
while(1){//ȫ�����жϽ���
	GPS_UpDate();
}
}
