#include "Timer.h"
#include "SETTING.H"
void TIM2_Init(int arr,int psc){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_TimeBaseStruct.TIM_Period = arr; //触发阈值                          
	TIM_TimeBaseStruct.TIM_Prescaler = (psc-1);//时钟分频                          
	TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;//时钟预分频              
	TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;//计数模式         
	//TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;                            
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
 
 
	TIM_Cmd(TIM2, ENABLE);  //使能TIMx		
}