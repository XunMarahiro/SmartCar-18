#include "HMC5983.H"

int HMC_Data[6];
int x,y,z,x1,y1,z1;
int x_Max,x_Min,y_Max,y_Min,z_Max,z_Min;
int x_offset,y_offset,z_offset,x_gain,y_gain,z_gain;
int x_real,y_real,z_real;
double Pi=3.1415926;
double rectangle(){
	double rec;
//	printf("jiaodu=%f\r\n",atan2(y_real,x_real)*180/Pi+180);
	return atan2(y_real,x_real)*180/Pi+180;
}
void Set_Offeset(){
	x_offset=(x_Max+x_Min)/2;
	y_offset=(y_Max+y_Min)/2;
	z_offset=(z_Max+z_Min)/2;
}
void Find_Max_Min(){
		if(x>x_Max){x_Max=x;}
		if(x<x_Min){x_Min=x;}
		if(y>y_Max){y_Max=y;}
		if(y<y_Min){y_Min=y;}
		if(z>z_Max){z_Max=z;}
		if(z<z_Min){z_Min=z;}
}
void Middle_find(){
	if(x_Max-x_Min>y_Max-y_Min){y_gain=(x_Max-x_Min)/(y_Max-y_Min);x_gain=1;}else{x_gain=(y_Max-y_Min)/(x_Max-x_Min);y_gain=1;}
}

void HMC5983_Init(){
	x_gain=1;y_gain=1;z_gain=1;
I2C_Write_REG(HMC5983,0x00,0x68);
I2C_Write_REG(HMC5983,0x01,0x20);
I2C_Write_REG(HMC5983,0x02,0x01);
//Fix();
}
short HMC5983_Read_reg(int reg){
short H,L;
	H=I2C2_Read_REG(HMC5983,reg);
	L=I2C2_Read_REG(HMC5983,reg+1);
	I2C_Write_REG(HMC5983,0x02,0x01);
	return (H<<8)+L;
}
double HMC5983_Read(){
	x1=HMC5983_Read_reg(X_MSB);
	y1=HMC5983_Read_reg(Y_MSB);
	z1=HMC5983_Read_reg(Z_MSB);
	if(x1!=0&&z1!=0&&y1!=0){
	//printf("x=%d\r\n",x);
	//printf("y=%d\r\n",y);
	//printf("z=%d\r\n",z);
		x=x1;
		y=y1;
		z=z1;
		Find_Max_Min();
		Set_Offeset();
		Middle_find();
		x_real=x*x_gain-x_offset;
		y_real=y*y_gain-y_offset;
		z_real=z-z_offset;
		//printf("x_real=%d\r\n",x_real);
		//printf("y_real=%d\r\n",y_real);
		return rectangle();
	}
}