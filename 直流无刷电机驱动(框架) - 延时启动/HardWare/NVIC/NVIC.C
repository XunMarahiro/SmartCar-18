#include "NIVC.H"
#include "HAL_device.h"
#include "HAL_conf.h"
void nvic_Init(){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_Config;
	GPIO_Config.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Config.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_Init(GPIOA,&GPIO_Config);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource3);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource4);
	
	EXTI_InitTypeDef EXTI_conf;
	EXTI_conf.EXTI_Line=EXTI_Line1;
	EXTI_conf.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_conf.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
	EXTI_conf.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_conf);
	
	EXTI_conf.EXTI_Line=EXTI_Line3;
	EXTI_Init(&EXTI_conf);
	
	EXTI_conf.EXTI_Line=EXTI_Line4;
	EXTI_Init(&EXTI_conf);
	
	NVIC_InitTypeDef NVIC_conf;
	NVIC_conf.NVIC_IRQChannel=EXTI0_1_IRQn;
	NVIC_conf.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
	NVIC_conf.NVIC_IRQChannelPriority = 1; //IRQ 通道被使能
	NVIC_Init(&NVIC_conf); //④初始化 NVIC 寄存器
	
	NVIC_conf.NVIC_IRQChannel=EXTI2_3_IRQn;
	NVIC_conf.NVIC_IRQChannelPriority = 1; //IRQ 通道被使能
	NVIC_Init(&NVIC_conf); //④初始化 NVIC 寄存器
	
	NVIC_conf.NVIC_IRQChannel=EXTI4_15_IRQn;
	NVIC_conf.NVIC_IRQChannelPriority = 1; //IRQ 通道被使能
	NVIC_Init(&NVIC_conf); //④初始化 NVIC 寄存器
	
}