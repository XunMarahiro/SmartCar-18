#ifndef _IIC_H
#define _IIC_H

#include "SETTING.H"
#define Pin_SCL_H  GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_SET);
#define	Pin_SCL_L	 GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_RESET);
#define	Pin_SDA_H	 GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_SET);
#define	Pin_SDA_L  GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_RESET);
#define Read_SDA_Pin GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11)
#define	I2C2_Delay1us DELAY_Us(10)

bool I2C_Write_REG(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t REG_data);
void IIC_GPIO_Init();
uint8_t I2C2_Read_REG(uint8_t SlaveAddress,uint8_t REG_Address);
bool I2C2_Read_NByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t* buf, uint8_t len);


#endif
