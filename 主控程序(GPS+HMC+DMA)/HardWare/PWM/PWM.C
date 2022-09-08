#include "PWM.H"
#include "SETTING.H"

void PWM_GPIO_Init(){
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_2);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}	

void PWM_Init(int arr,int psc){
	PWM_GPIO_Init();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_TimeBaseStruct.TIM_Period = arr; //触发阈值                          
	TIM_TimeBaseStruct.TIM_Prescaler = (psc-1);//时钟分频                          
	TIM_TimeBaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;//时钟预分频              
	TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;//计数模式         
	//TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;                            
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);
	
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM2;//输出模式
	TIM_OCInitStruct.TIM_Pulse=32;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_Low;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	
	TIM_OC1Init(TIM3, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);  
	TIM_Cmd(TIM3,ENABLE);  
}