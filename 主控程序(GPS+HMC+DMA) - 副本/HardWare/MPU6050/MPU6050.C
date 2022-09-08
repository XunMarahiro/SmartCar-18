#include "MPU6050.H"
extern double rectangle(double Y_val,double X_val);
int magX,magY,magZ;
double ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z,pitch,roll;
void MPU6050_Init(){
	IIC_GPIO_Init();
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
//			printf("ACC_X_H=%d\r\n",MPU6050_Read(ACC_X_H));
//		printf("ACC_Y_H=%d\r\n",MPU6050_Read(ACC_Y_H));
//		printf("ACC_Z_H=%d\r\n",MPU6050_Read(ACC_Z_H));
//		printf("GYRO_X_H=%d\r\n",MPU6050_Read(GYRO_X_H));
//		printf("GYRO_Y_H=%d\r\n",MPU6050_Read(GYRO_Y_H));
//		printf("GYRO_Z_H=%d\r\n",MPU6050_Read(GYRO_Z_H));
		ACC_X=(double)MPU6050_Read(ACC_X_H);
		ACC_Y=(double)MPU6050_Read(ACC_Y_H);
		ACC_Z=(double)MPU6050_Read(ACC_Z_H);
		GYRO_X=(double)MPU6050_Read(GYRO_X_H);
		GYRO_Y=(double)MPU6050_Read(GYRO_Y_H);
		GYRO_Z=(double)MPU6050_Read(GYRO_Z_H);
		pitch=rectangle(ACC_Z,ACC_X)+90;
		roll=rectangle(ACC_Z,ACC_Y)+90;
		if(pitch>360){pitch=pitch-360;}
		if(roll>360){roll=roll-360;}
		if(pitch>180){pitch=360-pitch;}
		if(roll>180){roll=360-roll;}
		if(pitch>90){pitch=180-pitch;}
		if(roll>90){roll=180-roll;}
//		printf("pitch:%f\r\n",pitch);
//		printf("roll:%f\r\n",roll);

}

double MPU9250_READ_MAG()
{ 
	u8 BUF[6];
	I2C_Write_REG(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
  DELAY_Ms(10);	
  I2C_Write_REG(MAG_ADDRESS,0x0A,0x01);
  DELAY_Ms(10);
	
	BUF[0]=I2C2_Read_REG(MAG_ADDRESS,MAG_XOUT_L);
	BUF[1]=I2C2_Read_REG(MAG_ADDRESS,MAG_XOUT_H);
	magX=(BUF[1]<<8)|BUF[0];

	BUF[2]=I2C2_Read_REG(MAG_ADDRESS,MAG_YOUT_L);
	BUF[3]=I2C2_Read_REG(MAG_ADDRESS,MAG_YOUT_H);
	magY=(BUF[3]<<8)|BUF[2];
		 
	BUF[4]=I2C2_Read_REG(MAG_ADDRESS,MAG_ZOUT_L);
	BUF[5]=I2C2_Read_REG(MAG_ADDRESS,MAG_ZOUT_H);
	magZ=(BUF[5]<<8)|BUF[4];
	if(magX>32768){magX=magX-65535;}
	if(magY>32768){magY=magY-65535;}
	if(magZ>32768){magZ=magZ-65535;}
	if(magX!=0&&magY!=0&&magZ!=0){
	printf("magX=%d\r\n",magX);
	printf("magY=%d\r\n",magY);
	printf("magZ=%d\r\n",magZ);
	magX=magX*cos(pitch)+magY*sin(roll)*sin(pitch)-magZ*cos(roll)*sin(pitch);
	magY=magY*cos(roll)+magZ*sin(roll);
	return 360-rectangle((double)magY/0.15,(double)magX/0.15);
	}
}