#include "SETTING.H"
#include "target.h"

#include "USART.H"
#include "IIC.H"
#include "PWM.H"
#include "Timer.h"

#include "HMC5983.H"
#include "GPS.H"

double jiaodu;

extern double lng_val;
extern double lat_val;

extern double way_val;
extern double target;
extern int daohangcanshu;

void Init_For_All_Device(){
USART1_Init(115200);//与电脑通信  PA9 PA10

USART4_Init(115200);//与电机驱动通信 PC10 PC11 
PWM_Init(100,3000);// 舵机控制信号 PA6
HMC5983_Init();
GPS_Init();
UART_SendData(UART4,0x0A);//电机启动
TIM2_Init(500,6000);	
USART3_Init(115200);//与GPS通信  PB10 PB11

printf("Init_Success\r\n");
}


void Main_Loop(){
}

void TIM2_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx更新中断标志 
		way_val=HMC5983_Read();
		jiaodu=target-way_val;
		printf("转向角度:%f°\r\n当前方向:%f\r\n目标角度:%f\r\n\r\n",jiaodu,way_val,target);
		if(jiaodu<-180){jiaodu=jiaodu+360;}
		if(jiaodu>90){daohangcanshu++;}
		if(jiaodu<-90){daohangcanshu++;}// 证明目标位置在现在方向后方
		jiaodu=jiaodu/90*8+24;//直接 变成角度修正
		if(((int)jiaodu<=32)&&((int)jiaodu>=16)){
		TIM3->CCR1=jiaodu	;//进行角度修改 
		printf("转向角度(占空比):%f%\r\n\r\n",jiaodu);
		}
}
}