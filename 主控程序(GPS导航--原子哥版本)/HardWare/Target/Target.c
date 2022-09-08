#include "SETTING.H"
#include "target.h"

#include "USART.H"
#include "IIC.H"
#include "PWM.H"
#include "Timer.h"

#include "HMC5983.H"
#include "GPS.H"

double jiaodu;

extern double lng_val;
extern double lat_val;

extern double way_val;
extern double target;
extern int daohangcanshu;

void Init_For_All_Device(){
USART1_Init(115200);//�����ͨ��  PA9 PA10

USART4_Init(115200);//��������ͨ�� PC10 PC11 
PWM_Init(100,3000);// ��������ź� PA6
HMC5983_Init();
GPS_Init();
UART_SendData(UART4,0x0A);//�������
TIM2_Init(500,6000);	
USART3_Init(115200);//��GPSͨ��  PB10 PB11

printf("Init_Success\r\n");
}


void Main_Loop(){
}

void TIM2_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
		{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
		way_val=HMC5983_Read();
		jiaodu=target-way_val;
		printf("ת��Ƕ�:%f��\r\n��ǰ����:%f\r\nĿ��Ƕ�:%f\r\n\r\n",jiaodu,way_val,target);
		if(jiaodu<-180){jiaodu=jiaodu+360;}
		if(jiaodu>90){daohangcanshu++;}
		if(jiaodu<-90){daohangcanshu++;}// ֤��Ŀ��λ�������ڷ����
		jiaodu=jiaodu/90*8+24;//ֱ�� ��ɽǶ�����
		if(((int)jiaodu<=32)&&((int)jiaodu>=16)){
		TIM3->CCR1=jiaodu	;//���нǶ��޸� 
		printf("ת��Ƕ�(ռ�ձ�):%f%\r\n\r\n",jiaodu);
		}
}
}