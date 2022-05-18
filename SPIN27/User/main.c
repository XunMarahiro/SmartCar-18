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
int t;
int Volt=15;	
int target=15;
int Command;
void Shut_Down(){
	TIM1->CCR1=0;
	TIM3->CCR1=0;
	TIM1->CCR2=0;
	TIM3->CCR2=0;
	TIM1->CCR3=0;
	TIM3->CCR3=0;
}
void delay(int i){
	while(i--);
}

void Check_Command(){
	if(Command>100){Shut_Down();}
	if(Command<100){target=Command;}
}
void Speed_Update(){
	TIM3->CCR2=Volt;
	TIM1->CCR2=Volt;
}
int main(){
	Init_USART(115200);
	TIM1_PWM(100,48);
	TIM3_PWM(100,48);
	Shut_Down();
	TIM3->CCR2=50;
	TIM1->CCR2=50;
	while(1){
	delay(5000);
		if(target>Volt&&t==500){Volt++;Speed_Update();t=0;}
		if(target<Volt&&t==500){Volt--;Speed_Update();t=0;}
		t++;
		Check_Command();
	}
}

void UART1_IRQHandler(){
	if(UART_GetITStatus( UART1, UART_IT_RXIEN) ){ 
			UART_ClearITPendingBit(UART1, UART_IT_RXIEN);
		Command=UART_ReceiveData(UART1);
	} 	
}
