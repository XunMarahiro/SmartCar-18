#include "SETTING.H"
#include "USART.H"
#include "IIC.H"
#include "MPU6050.H"
#include "HMC5983.H"
#include "GPS.H"
#include "PWM.H"

int main(){
USART1_Init(115200);//�����ͨ��
USART3_Init(115200);//��GPSͨ��
USART4_Init(115200);//��������ͨ��
PWM_Init(100,900);// ��������ź�
UART_SendData(UART4,0x14);//20�ٶȳ�ʼֵ
printf("USART_Init \r\n");
while(1){//ȫ�����жϽ���
	GPS_UpDate();
}
}
