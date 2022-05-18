#include "motor.h"
#include "PWM.H"
#include "HAL_device.h"
#include "HAL_conf.h"


void delay(int i){
	while(i--){
	for(int c=100;c--;);
		}
}

void Stop(){
	TIM1->CCR1=0;
	TIM3->CCR1=100;
	TIM1->CCR2=0;
	TIM3->CCR2=100;
	TIM1->CCR3=0;
	TIM3->CCR3=100;
}
void Shut_Down(){
	TIM1->CCR1=0;
	TIM3->CCR1=0;
	TIM1->CCR2=0;
	TIM3->CCR2=0;
	TIM1->CCR3=0;
	TIM3->CCR3=0;
}
void Start_Init(){
		Shut_Down();
		TIM3->CCR2=100;
		for(int t=500;t>0;t--){
			TIM1->CCR1=10;
			TIM1->CCR3=10;
			delay(40);
			TIM1->CCR1=0;
			TIM1->CCR3=0;
			delay(40);
		}
		Stop();
		delay(10);
		Shut_Down();
}