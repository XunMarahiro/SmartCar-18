#include "HAL_device.h"
#include "HAL_conf.h"
#include "USART.H"
#include "GPS.H"
#include "PWM.H"
//Reference voltage, the unit is: V
#define REFVOLATGE 5
int main(){
Init_USART1();
Init_USART2();
PWM_Init(100,200);
	while(1){
	printf("Success\r\n");
	}
}
