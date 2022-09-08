#include "MPU6050.H"
int magX,magY,magZ;
void MPU6050_Init(){
	IIC_GPIO_Init();
	I2C_Write_REG(MPU6050,PWR_MGMT_1,0x80);//解除休眠状态
	DELAY_Ms(100);
	I2C_Write_REG(MPU6050,PWR_MGMT_1,0x00);//解除休眠状态
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

void MPU9250_READ_MAG()
{ 
	u8 BUF[6];
	I2C_Write_REG(GYRO_ADDRESS,INT_PIN_CFG,0x02);//turn on Bypass Mode 
	I2C2_Delay1us;
	I2C_Write_REG(MAG_ADDRESS,0x0A,0x01);//用来启动单次转换,否则磁力计输出的数据不变
	I2C2_Delay1us;
		 
	BUF[0]=I2C2_Read_REG(MAG_ADDRESS,MAG_XOUT_L);
	BUF[1]=I2C2_Read_REG(MAG_ADDRESS,MAG_XOUT_H);
	magX=(BUF[1]<<8)|BUF[0];
	printf("BUF[0]:%d\r\n",BUF[0]);
	printf("BUF[1]:%d\r\n",BUF[1]);
	BUF[2]=I2C2_Read_REG(MAG_ADDRESS,MAG_YOUT_L);
	BUF[3]=I2C2_Read_REG(MAG_ADDRESS,MAG_YOUT_H);
	magY=(BUF[3]<<8)|BUF[2];
		 
	BUF[4]=I2C2_Read_REG(MAG_ADDRESS,MAG_ZOUT_L);
	BUF[5]=I2C2_Read_REG(MAG_ADDRESS,MAG_ZOUT_H);
	magZ=(BUF[5]<<8)|BUF[4];
	printf("Magx:%d\r\n",magX);
	printf("magY:%d\r\n",magY);
	printf("magZ:%d\r\n",magZ);
}