#include "ADC.H"
#include "HAL_device.h"
#include "HAL_conf.h"

void ADC_User_Init(){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	GPIO_InitTypeDef GPIO_Config;
	GPIO_Config.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Config.GPIO_Pin=GPIO_Pin_0;
	GPIO_Config.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Config);
	
	
	ADC_InitTypeDef ADC_Config;
	ADC_Config.ADC_Resolution=ADC_Resolution_12b;
	ADC_Config.ADC_PRESCARE=ADC_PCLK2_PRESCARE_2;
	ADC_Config.ADC_Mode=ADC_Mode_Continuous_Scan;
	ADC_Config.ADC_ExternalTrigConv=ADC_ExternalTrigShiftTime_64_Cycles;
	ADC_Config.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_Init(ADC1,&ADC_Config);
	ADC_Cmd(ADC1,ENABLE);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0, 0, ADC_SampleTime_239_5Cycles);
}
int Get_ADC_vale(){
	   u16 puiADData;
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);                                     //Software start conversion
    while(ADC_GetFlagStatus(ADC1, ADC_IT_EOC) == 0);
    ADC_ClearFlag(ADC1, ADC_IT_EOC);
    puiADData = ADC_GetConversionValue(ADC1);
    return puiADData/4096*ADC_Gain;
}