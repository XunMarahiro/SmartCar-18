#include "SETTING.H"
#include "target.h"

#include "USART.H"
#include "IIC.H"
#include "PWM.H"
#include "Timer.h"
#include "DMA.h"
#include "vector.H"
#include "HMC5983.H"
#include "GPS.H"
#include "MPU6050.h"
#include "PID.H"

double jiaodu,way_val,zhankongbi=32;

extern double lng_val;
extern double lat_val;

extern double target;
extern int daohangcanshu,Point_Sum;

extern double Base_way;

void Init_For_All_Device(){
USART1_Init(115200);//与电脑通信  PA9 PA10
//USART4_Init(115200);//与电机驱动通信 PC10 PC11 
//PWM_Init(100,3000);// 舵机控制信号 PA6
USART3_Init(115200);//与GPS通信  PB10 PB11
//DELAY_Init();

UART3_DMA_Init();
GPS_Init();//必须放在DMA初始化后
//MPU6050_Init();	
//HMC5983_Init(); 


//UART_SendData(UART4,0x0A);//电机启动
//TIM2_Init(100,6000);
//Base_way=HMC5983_Read();
//printf("Init_Success\r\n");

}

void Main_Loop(){

}

double StraightWay(double Exchange){
		zhankongbi=PID(Exchange);
		if(zhankongbi>50){zhankongbi=50;}if(zhankongbi<24){zhankongbi=24;}//转向限幅
		TIM3->CCR1=zhankongbi	;//进行角度修改 
		printf("zhankongbi=%d\r\n更改角度:%f\r\n",TIM3->CCR1,Exchange);
		printf("jizhunxian=%f\r\njiaodu=%f\r\n",Base_way,way_val);
}

void TIM2_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 
		way_val=HMC5983_Read();
		printf("目标方向:%f\r\n当前方向:%f\r\n",Base_way,way_val);
		StraightWay(Find_Exchange_Way(way_val,Base_way));
				
		}
}

void UART1_IRQHandler(void)   //TIM3中断
{
	int data;
	if(UART_GetITStatus(UART1,UART_IT_RXIEN)!= RESET){
		 UART_ClearITPendingBit(UART1,UART_IT_RXIEN);
		 data=UART_ReceiveData(UART1);
		 if(data<100){UART_SendData(UART4,data);}
		 else{UART_SendData(UART4,0x00);}
		 printf("%x\r\n",data);
	}
}
