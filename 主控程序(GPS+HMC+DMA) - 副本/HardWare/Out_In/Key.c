#include "key.h"
#include "SETTING.H"
extern double lng_val,lat_val;
#define Key
void Key_Init(){
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOE, ENABLE);
	//UART1_TX   GPIOA.9
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOC, ENABLE);
	//UART1_TX   GPIOA.9
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_RESET);
	GPIO_WriteBit(GPIOE,GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOE,GPIO_Pin_14,Bit_RESET);
}
void RED(){
	GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_SET);
	GPIO_WriteBit(GPIOE,GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOE,GPIO_Pin_14,Bit_RESET);
}
void GREEN(){
	GPIO_WriteBit(GPIOE,GPIO_Pin_12,Bit_SET);
	GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_RESET);
	GPIO_WriteBit(GPIOE,GPIO_Pin_14,Bit_RESET);
}
void BLUE(){
	GPIO_WriteBit(GPIOE,GPIO_Pin_14,Bit_SET);
	GPIO_WriteBit(GPIOE,GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_RESET);
}
void Key_Scan(){
	if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)){
		BLUE();
		while(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0));
		RED();
		printf("%f,%f,\r\n",lat_val,lng_val);
	}
}