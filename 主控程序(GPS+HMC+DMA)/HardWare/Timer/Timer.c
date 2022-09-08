#include "Timer.h"
#include "SETTING.H"
void TIM2_Init(int arr,int psc){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_TimeBaseStruct.TIM_Period = arr; //������ֵ                          
	TIM_TimeBaseStruct.TIM_Prescaler = (psc-1);//ʱ�ӷ�Ƶ                          
	TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;//ʱ��Ԥ��Ƶ              
	TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;//����ģʽ         
	//TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;                            
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
 
 
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx		
}