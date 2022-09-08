#ifndef _MPU6050_H
#define _MPU6050_H
#include "SETTING.H"
#include "USART.H"
#include "IIC.H"


#define MPU6050 0xD0

#define ACC_X_H 0x3B 
#define ACC_X_L	0x3C
#define ACC_Y_H	0x3D
#define ACC_Y_L	0x3E
#define ACC_Z_H	0x3F	
#define ACC_Z_L	0x40

#define TEMP_H 0x41
#define TEMP_L 0x42

#define GYRO_X_H 0x43
#define GYRO_X_L 0x44
#define GYRO_Y_H 0x45
#define GYRO_Y_L 0x46
#define GYRO_Z_H 0x47
#define GYRO_Z_L 0x48

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define ACCEL_ADDRESS				0xD0
#define GYRO_ADDRESS				0xD0
#define MAG_ADDRESS					0x18
#define INT_PIN_CFG					0x37

//指南针设备地址
#define MAG_ADDR	0xD0
//指南针设备ID寄存器
#define MAG_WIA		0x00
//指南针设备ID
#define MAG_ID		0x48
//启动单次传输
#define MAG_CNTL1 0x0a

#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz
#define	CONFIG				0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

void MPU6050_Init();
int MPU6050_Read(int reg);
double MPU9250_READ_MAG();

#endif 