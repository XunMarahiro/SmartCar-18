#include "PWM.H"
#include "HAL_device.h"
#include "HAL_conf.h"

void TIM1_PWM(int arr,int psc){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_Config;
	GPIO_Config.GPIO_Mode=GPIO_Mode_AF_PP;	
	GPIO_Config.GPIO_Pin=GPIO_Pin_7;
	GPIO_Config.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Config);

	
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_6);//CH3
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_6);//CH2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_6);//CH1 上桥
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_TimeBaseStruct.TIM_Period = arr; //触发阈值                          
	TIM_TimeBaseStruct.TIM_Prescaler = (psc-1);//时钟分频                          
	TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;//时钟预分频              
	TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;//计数模式         
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;                            
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);
	
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;//输出模式
	TIM_OCInitStruct.TIM_Pulse=arr/2;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;

	TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);  
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1,ENABLE);  
}

