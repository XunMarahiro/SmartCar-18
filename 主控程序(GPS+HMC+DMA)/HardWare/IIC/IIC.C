#include "IIC.H"

void IIC_GPIO_Init(){
	  GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD,ENABLE);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
}

bool I2C2_Start(void)
{
    Pin_SCL_H; // 拉高时钟线
    Pin_SDA_H; // 拉高信号线
    I2C2_Delay1us;
    if(!Read_SDA_Pin)		return false;
    Pin_SDA_L;
    I2C2_Delay1us;
    Pin_SDA_L;
    I2C2_Delay1us;
    return true;
}
 
// 发送IIC停止信号
bool I2C2_Stop(void)
{
    Pin_SCL_H;
    Pin_SDA_L;
    I2C2_Delay1us;
    if(Read_SDA_Pin)	return false;
    Pin_SDA_H;
    I2C2_Delay1us;
    if(!Read_SDA_Pin) return false;
    Pin_SDA_H;
    I2C2_Delay1us;	
    return true;
}
 
// IIC发送ACK信号
void I2C2_Ack(void)
{
    Pin_SCL_L;
    I2C2_Delay1us;
    Pin_SDA_L;	
    Pin_SCL_H;
    I2C2_Delay1us;
    Pin_SCL_L;
    Pin_SDA_H;
    I2C2_Delay1us;
}
 
// IIC不发送ACK信号
void I2C2_NAck(void)
{
    Pin_SCL_L;
    I2C2_Delay1us;	
    Pin_SDA_H;
    Pin_SCL_H;
    I2C2_Delay1us;
    Pin_SCL_L;
    I2C2_Delay1us;
}
 
// IIC等待ACK信号
uint8_t I2C2_Wait_Ack(void)
{
    Pin_SCL_L;
    I2C2_Delay1us;	
    Pin_SDA_H;
    Pin_SCL_H;
    I2C2_Delay1us;	
    if(Read_SDA_Pin)
    {
	Pin_SCL_L;
	I2C2_Delay1us;
	return false;
    }
    Pin_SCL_L;
    I2C2_Delay1us;
    return true;
}
 
// IIC发送一个字节
void I2C2_Send_Byte(uint8_t txd)
{
    for(uint8_t i=0; i<8; i++)
    {
	Pin_SCL_L;
	I2C2_Delay1us;
	if(txd & 0x80){
	    Pin_SDA_H;}
	else{
	    Pin_SDA_L;}
	    txd <<= 1;
	    Pin_SCL_H;
	    I2C2_Delay1us;
    }
}

 
// IIC读取一个字节
uint8_t	I2C2_Read_Byte(void)
{
    uint8_t rxd = 0;
    for(uint8_t i=0; i<8; i++)
    {
	rxd <<= 1;
	Pin_SCL_L;
	I2C2_Delay1us;
	Pin_SCL_H;	
	I2C2_Delay1us;		
	if(Read_SDA_Pin)
	{
	    rxd |= 0x01;
	}
    }
    return rxd;
}
// 向从机指定地址写数据
bool I2C_Write_REG(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t REG_data)
{
    if(!I2C2_Start())		return false;
    I2C2_Send_Byte(SlaveAddress);
    if(!I2C2_Wait_Ack()) { I2C2_Stop();	return false;	}
    I2C2_Send_Byte(REG_Address);
    if(!I2C2_Wait_Ack()) { I2C2_Stop();	return false;	}
    I2C2_Send_Byte(REG_data);
    if(!I2C2_Wait_Ack()) { I2C2_Stop(); return false;	}
    if(!I2C2_Stop()) return false;
    return true;
}
 
// 从设备中读取数据
uint8_t I2C2_Read_REG(uint8_t SlaveAddress,uint8_t REG_Address)
{
    uint8_t data;
    if(!I2C2_Start())	return false;
    I2C2_Send_Byte(SlaveAddress);
    if(!I2C2_Wait_Ack()) { I2C2_Stop();	return false;	}
    I2C2_Send_Byte(REG_Address);
    if(!I2C2_Wait_Ack()) { I2C2_Stop();	return false;	}
    if(!I2C2_Start())	return false;
    I2C2_Send_Byte(SlaveAddress + 1);
    if(!I2C2_Wait_Ack()) { I2C2_Stop();	return false;	}
    data = I2C2_Read_Byte();
    I2C2_NAck();
    if(!I2C2_Stop())	return false;	
    return data;
}
 
// 连续写N个字节
bool I2C2_Write_NByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t* buf, uint8_t len)
{
    if(!I2C2_Start())return false;
    I2C2_Send_Byte(SlaveAddress);  //发送设备地址+写信号
    if(!I2C2_Wait_Ack()){I2C2_Stop(); return false;}
    I2C2_Send_Byte(REG_Address);   
    if(!I2C2_Wait_Ack()){I2C2_Stop(); return false;}
    for(uint16_t i=0; i<len; i++)
    {
        I2C2_Send_Byte(buf[i]);
	if(i<len-1)
	{
            if(!I2C2_Wait_Ack()){I2C2_Stop(); return false;}
	}
    }
    I2C2_Stop();
    return true;
}
 
// 连续读N个字节
bool I2C2_Read_NByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t* buf, uint8_t len)
{
    if(!I2C2_Start())return false;
    I2C2_Send_Byte(SlaveAddress);  //发送设备地址+写信号
    if(!I2C2_Wait_Ack()){I2C2_Stop(); return false;}
    I2C2_Send_Byte(REG_Address);   
    if(!I2C2_Wait_Ack()){I2C2_Stop(); return false;}
    if(!I2C2_Start())return false;
    I2C2_Send_Byte(SlaveAddress | 1); // 读操作
    if(!I2C2_Wait_Ack()){I2C2_Stop(); return false;}
    for(uint16_t i=0; i<len; i++)
    {
        buf[i] = I2C2_Read_Byte();
	if(i<len-1)
	{
	    I2C2_Ack();
	}
    }
    I2C2_NAck();
    I2C2_Stop();
    return true;
}
 
 // 检查设备地址
bool I2C2_CheckDevice(uint8_t SlaveAddress)
{
    if(!I2C2_Start())	return false;
    I2C2_Send_Byte(SlaveAddress);
    if(!I2C2_Wait_Ack())
    {
	I2C2_Stop();
	return false;		
    }
    if(!I2C2_Stop())	return false;	
    return true;	
}


