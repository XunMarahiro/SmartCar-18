#include "SETTING.H"
#include "target.h"

#include "USART.H"
#include "IIC.H"
#include "PWM.H"
#include "Timer.h"
#include "DMA.h"
#include "vector.H"
#include "HMC5983.H"
#include "GPS.H"
#include "MPU6050.h"
#include "PID.H"
#include "Key.h"
double jiaodu,way_val,zhankongbi=32;

extern double lng_val;
extern double lat_val;

extern double target;
extern int daohangcanshu,Point_Sum;

extern double Base_way;
int SWITCH=1;
void Init_For_All_Device(){
USART1_Init(115200);//�����ͨ��  PA9 PA10
USART4_Init(115200);//��������ͨ�� PC10 PC11 

PWM_Init(100,396);// ��������ź� PA8
USART3_Init(115200);//��GPSͨ��  PB10 PB11
DELAY_Init();
//MPU6050_Init();	
UART3_DMA_Init();
GPS_Init();//�������DMA��ʼ����
Key_Init();
HMC5983_Init(); 
Base_way=HMC5983_Read();
UART_SendData(UART4,0x10);//�������
TIM2_Init(100,6000);

printf("Init_Success\r\n");
TIM1->CCR1=32;
	
}

void Main_Loop(){
	Key_Scan();
	printf("Test\r\n");
}

double StraightWay(double Exchange){
		zhankongbi=PID(Exchange);
		if(zhankongbi>50){zhankongbi=50;}if(zhankongbi<24){zhankongbi=24;}//ת���޷�
		TIM1->CCR1=zhankongbi	;//���нǶ��޸� 
//		printf("zhankongbi=%d\r\n���ĽǶ�:%f\r\n",TIM1->CCR1,Exchange);
//		printf("jizhunxian=%f\r\njiaodu=%f\r\n",Base_way,way_val);
}

void TIM2_IRQHandler(void)   //TIM2�ж�
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
		{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //���TIMx�����жϱ�־ 
		way_val=HMC5983_Read();
//		printf("TARGET=%f\r\nNow=%f\r\n",Base_way,way_val);
		StraightWay(Find_Exchange_Way(way_val,Base_way));
		}
}

void UART1_IRQHandler(void) 
{
	int data;
	if(UART_GetITStatus(UART1,UART_IT_RXIEN)){
		 UART_ClearITPendingBit(UART1,UART_IT_RXIEN);
		 data=UART_ReceiveData(UART1);
  	 while(!UART_GetFlagStatus(UART4,UART_FLAG_RXAVL)){UART_SendData(UART4,data);} 
		 printf("%x\r\n",data);
	}
}
