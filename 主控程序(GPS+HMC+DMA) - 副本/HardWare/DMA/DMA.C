#include "DMA.H"
#include "SETTING.H"
extern u8 Data_Buf[1024];
void UART3_DMA_Init(){
	RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA1, ENABLE);
	DMA_InitTypeDef DMA_InitStructure;
	/* Configure DMA Stream */
	DMA_InitStructure.DMA_PeripheralBaseAddr =0x40004804;
	DMA_InitStructure.DMA_MemoryBaseAddr =&Data_Buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = (uint32_t)512;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc =DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;         
	DMA_InitStructure.DMA_M2M=DMA_M2M_Disable;
	DMA_InitStructure.DMA_Auto_reload=DMA_Auto_Reload_Enable;
	DMA_Init(DMA1_ch3, &DMA_InitStructure);
	
	UART_DMACmd(UART3, UART_DMAReq_EN, ENABLE);
	
	DMA_ITConfig(DMA1_ch3,DMA_IT_TC,ENABLE);	//使能DMA1传输中断	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
	DMA_Cmd(DMA1_ch3,ENABLE); //使能DMA1通道1
}