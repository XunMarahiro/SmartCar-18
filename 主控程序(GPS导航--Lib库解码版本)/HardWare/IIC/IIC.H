#ifndef _IIC_H
#define _IIC_H

#include "SETTING.H"
#define Pin_SCL_H  GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_SET);
#define	Pin_SCL_L	 GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_RESET);
#define	Pin_SDA_H	 GPIO_WriteBit(GPIOB,GPIO_Pin_7,Bit_SET);
#define	Pin_SDA_L  GPIO_WriteBit(GPIOB,GPIO_Pin_7,Bit_RESET);
#define Read_SDA_Pin GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)
#define	I2C2_Delay1us DELAY_Us(10)

bool I2C_Write_REG(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t REG_data);
void IIC_GPIO_Init();
uint8_t I2C2_Read_REG(uint8_t SlaveAddress,uint8_t REG_Address);
bool I2C2_Read_NByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t* buf, uint8_t len);
#endif