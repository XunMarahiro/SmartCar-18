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
int Volt=30;	
int target=15;
int Command;
float val;
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
	TIM3->CCR2=100;
	TIM1->CCR2=Volt;
}
int main(){
	ADC_User_Init();
	
	TIM3_PWM(100,9600);
	TIM1_PWM(100,9600);
	Init_USART(115200);
	Shut_Down();
	TIM1->CCR1=Volt;
	TIM3->CCR3=100;
	while(1){
//		delay(5000);
//		if(target>Volt&&t==500){Volt++;Speed_Update();t=0;}
//		if(target<Volt&&t==500){Volt--;Speed_Update();t=0;}
//		t++;
//		Check_Command();
		printf("V---%f\r\n",Get_ADC_vale());
		if((val=Get_ADC_vale())>20){
			Shut_Down();
			printf("V---%f\r\n",val);
			while(1);
		}
	}
}

void UART1_IRQHandler(){
	if(UART_GetITStatus( UART1, UART_IT_RXIEN) ){ 
			UART_ClearITPendingBit(UART1, UART_IT_RXIEN);
		Command=UART_ReceiveData(UART1);
	} 	
}
