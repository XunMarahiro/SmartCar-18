#include "USART.H"
#include "HAL_device.h"
#include "HAL_conf.h"

typedef struct __FILE FILE;

void Init_USART(int bundrate){
	
	NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_InitStruct.NVIC_IRQChannel = UART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
	NVIC_Init(& NVIC_InitStruct);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1,ENABLE);
		
	GPIO_InitTypeDef GPIO_Config;
	GPIO_Config.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Config.GPIO_Pin=GPIO_Pin_9;
	
	GPIO_Config.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Config);
		
	GPIO_Config.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Config.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOA,&GPIO_Config);
		
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);		
		
	UART_ClearITPendingBit(  UART1, UART_IT_RXIEN);
	UART_ITConfig( UART1, UART_IT_RXIEN, ENABLE );
		
	UART_InitTypeDef UART_Config;
	UART_Config.UART_BaudRate=bundrate;
	UART_Config.UART_HardwareFlowControl=UART_HardwareFlowControl_None;
	UART_Config.UART_Mode=UART_Mode_Rx|UART_Mode_Tx;
	UART_Config.UART_Parity=UART_Parity_No;
	UART_Config.UART_StopBits=UART_StopBits_1;
	UART_Config.UART_WordLength=UART_WordLength_8b;
	UART_Init(UART1,&UART_Config);
	UART_Cmd(UART1,ENABLE);
}



#if defined(__GNUC__)
#define PUTCHAR_PROTOTYPE s32 __io_putchar(s32 ch)
#else
#define PUTCHAR_PROTOTYPE s32 fputc(s32 ch, FILE *f)
#endif

#if defined(__ICCARM__)
PUTCHAR_PROTOTYPE {
	while((UART1->CSR & UART_CSR_TXC) == 0); //The loop is sent until it is finished
	UART1->TDR = (ch & (u16)0x00FF);
	return ch;
}
#else
s32 fputc(s32 ch, FILE* f)
{
	while((UART1->CSR & UART_CSR_TXC) == 0); //The loop is sent until it is finished
	UART1->TDR = (ch & (u16)0x00FF);
	return ch;
}

#endif
