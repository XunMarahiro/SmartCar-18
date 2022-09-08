#include "HAL_device.h"
#include "HAL_conf.h"
#include "USART.H"
#include "NIVC.H"
#include "math.h"
#include "PWM.H"
#include "RCC.H"
#include "ADC.H"
//Reference voltage, the unit is: V
#define REFVOLATGE 5
int Volt=20,target=20;//15
int Flag=6;
int S1=1,S2=1,S3=1;
//CH1_H PB8 A桥上
//CH2_H PB5	B桥上
//CH3_H PB7	C桥上
//CH1_L	PB6	A桥下
//CH2_L	PB3	B桥下
//CH3_L	PB4	C桥下
void Shut_Down(){
TIM1->CCER=0x0000;
TIM1->EGR = TIM_EventSource_COM;
}
void Exange(){
	if(Flag==1){TIM1->CCER=OUT1;TIM1->EGR = TIM_EventSource_COM;}
	if(Flag==2){TIM1->CCER=OUT2;TIM1->EGR = TIM_EventSource_COM;}
	if(Flag==3){TIM1->CCER=OUT3;TIM1->EGR = TIM_EventSource_COM;}
	if(Flag==4){TIM1->CCER=OUT4;TIM1->EGR = TIM_EventSource_COM;}
	if(Flag==5){TIM1->CCER=OUT5;TIM1->EGR = TIM_EventSource_COM;}
	if(Flag==6){TIM1->CCER=OUT6;TIM1->EGR = TIM_EventSource_COM;Flag=0;}
	Flag++;
}
void Volt_Set(int a){
	TIM1->CCR1=a;
	TIM1->CCR2=a;
	TIM1->CCR3=a;
}
void delay(int t){
	while(t--){
	for(int a=0;a<100;a++);
	};
}
void Check_Position(){
	int t=200;
	Volt_Set(20);
	while(t--){
	TIM1->CCER=OUT6;TIM1->EGR = TIM_EventSource_COM;
		delay(50);
	TIM1->CCER=OUT1;TIM1->EGR = TIM_EventSource_COM;
		delay(50);
	}
	Shut_Down();
	Volt_Set(Volt);
}
int main(){
	TIM1_PWM(100,96);	
	Check_Position();
	delay(10000);
	Volt_Set(Volt);
	nvic_Init();
	while(1){
		delay(10000);
		if(Volt<target){Volt++;Volt_Set(Volt);}
		if(Volt>target){Volt--;Volt_Set(Volt);}
	}
}
void noop(){
}

void EXTI0_1_IRQHandler(){	//A相反电动势 OUT3 OUT6
Exange();	//EXTI_ClearITPendingBit(EXTI_Line1);
EXTI->PR=EXTI_Line1;//清除中断标志位
	
	
	

	
}
void EXTI2_3_IRQHandler(){	//B相反电动势 OUT2 OUT5
Exange();//EXTI_ClearITPendingBit(EXTI_Line3);
EXTI->PR=EXTI_Line3;	
		




}
void EXTI4_15_IRQHandler(){	//C相反电动势 OUT1 OUT4
Exange();//	EXTI_ClearITPendingBit(EXTI_Line4);
EXTI->PR=EXTI_Line4;	
	

	


}