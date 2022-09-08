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

void Speed_Update(){
	TIM1->CCR3=Volt;
}
int main(){
	Init_USART(115200);
	TIM1_PWM(100,960);
	Shut_Down();
	TIM1->CCR3=30;
	while(1){
	delay(50000);
		if(target>Volt){Volt++;}
		if(target<Volt){Volt--;}
		Speed_Update();
	}
}

void UART1_IRQHandler(){
	if(UART_GetITStatus( UART1, UART_IT_RXIEN) ){ 
			UART_ClearITPendingBit(UART1, UART_IT_RXIEN);
			target=UART_ReceiveData(UART1);
			printf("Volt=%x\r\n",Volt);
			printf("CCR1=%x\r\n",TIM1->CCR1);
	} 	
}
