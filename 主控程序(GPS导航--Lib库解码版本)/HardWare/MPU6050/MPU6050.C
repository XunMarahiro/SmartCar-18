#include "MPU6050.H"

void MPU6050_Init(){
	I2C_Write_REG(MPU6050,PWR_MGMT_1,0x80);//½â³ýÐÝÃß×´Ì¬
	DELAY_Ms(100);
	I2C_Write_REG(MPU6050,PWR_MGMT_1,0x00);//½â³ýÐÝÃß×´Ì¬
	I2C_Write_REG(MPU6050,SMPLRT_DIV,0x07);
	I2C_Write_REG(MPU6050,CONFIG,0x06);
	I2C_Write_REG(MPU6050,GYRO_CONFIG,0x18);
	I2C_Write_REG(MPU6050,ACCEL_CONFIG,0x01);
	
	I2C_Write_REG(MPU6050,PWR_MGMT_1,0x01);
	I2C_Write_REG(MPU6050,PWR_MGMT_1+1,0x00);
}

int MPU6050_Read(int reg){
	int H,L,Data;
	H=I2C2_Read_REG(MPU6050,reg);
	L=I2C2_Read_REG(MPU6050,reg+1);
	Data=(H<<8)+L;
	if(Data<65536 && Data>32768){Data=Data-65536;}
	return Data;
}

void MPU6050_DATA_PRINT(){
		printf("ACC_X_H=%d\r\n",MPU6050_Read(ACC_X_H));
		printf("ACC_Y_H=%d\r\n",MPU6050_Read(ACC_Y_H));
		printf("ACC_Z_H=%d\r\n",MPU6050_Read(ACC_Z_H));
		printf("GYRO_X_H=%d\r\n",MPU6050_Read(GYRO_X_H));
		printf("GYRO_Y_H=%d\r\n",MPU6050_Read(GYRO_Y_H));
		printf("GYRO_Z_H=%d\r\n",MPU6050_Read(GYRO_Z_H));
}