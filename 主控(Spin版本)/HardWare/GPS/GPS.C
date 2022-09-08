#include "GPS.H"
#include "USART.H"
#include "math.h"
#include "HAL_device.h"
#include "HAL_conf.h"
#include "vector.h"

double x_pi = 52.35987755983;  //坐标转换 参数 本程序中 坐标均有 地球坐标系 转换为 火星坐标系进行运作
double pi = 3.1415926535897932384626 ; // π
double a = 6378245.0;  //长半轴
double ee = 0.00669342162296594323;  // 扁率
double target;
char Data_Buf[2048];
char GNRMC[100];

double jingdu,weidu,way,jiaodu,loca[2];//对 经度 维度 方向 转向角度 和一个 中间中继变量 进行 初始化
int flag[30],fg=0,buf[3];  //解码中, . 点位置的存储
int pos,daohangcanshu=0;	


double transformlat(double lng,double lat){
	double ret;
	ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * sqrt(fabs(lng));
	ret += (20.0 * sin(6.0 * lng * pi) + 20.0 *sin(2.0 * lng * pi)) * 2.0 / 3.0;
	ret += (20.0 * sin(lat * pi) + 40.0 *sin(lat / 3.0 * pi)) * 2.0 / 3.0;
	ret += (160.0 * sin(lat / 12.0 * pi) + 320 *sin(lat * pi / 30.0)) * 2.0 / 3.0;
	return ret;
}


double transformlng(double lng,double lat){
	double ret;
	ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng +0.1 * lng * lat + 0.1 * sqrt(fabs(lng));
	ret += (20.0 * sin(6.0 * lng * pi) + 20.0 *sin(2.0 * lng * pi)) * 2.0 / 3.0;
	ret += (20.0 * sin(lng * pi) + 40.0 *sin(lng / 3.0 * pi)) * 2.0 / 3.0;
	ret += (150.0 * sin(lng / 12.0 * pi) + 300.0 *sin(lng / 30.0 * pi)) * 2.0 / 3.0;
	return ret;
}

void wgs84_to_gcj02(double lng,double lat){
double dlat;
double dlng;	
double radlat;
double magic;
double sqrtmagic;
	dlat = transformlat(lng - 105.0, lat - 35.0);
	dlng = transformlng(lng - 105.0, lat - 35.0);
	radlat = lat / 180.0 * pi;
	magic = sin(radlat);
	magic = 1 - ee * magic * magic;
	sqrtmagic = sqrt(magic);
	dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi);
	dlng = (dlng * 180.0) / (a / sqrtmagic * cos(radlat) * pi);
	weidu = lat + dlat;//维度
	jingdu = lng + dlng;//经度
}

double Way_Point[50][2]={
112.41874,34.612924,
112.41874,34.612831,
112.41867,34.612787,
112.418568,34.612751,
112.41845,	34.612716,
112.418316,	34.612694,
112.418171,	34.612672,
112.418048,	34.612672,
112.41793,	34.612738,
112.417833,	34.612835,
112.417769	,34.612981,
112.417721,	34.613149,
112.417672,	34.613317,
112.417629,	34.613471,
112.417613,	34.613608,
112.417683,	34.613679,
112.417769,	34.613736,
112.417946,	34.613762,
112.418128	,34.613793,
112.418278,	34.613824,
112.418375,	34.613842,
112.418477,	34.613815,
112.418541	,34.613727,
112.418595,	34.613586,
112.418643	,34.613423,
112.418692,	34.613259,
112.418702,	34.613171,
112.418718,	34.613083,

};
void GNRMC_Check(char *data){//遍历数组求得 GNRMC数据段 非常耗时 暂时没有 优化方案
int pos1=0,pos2=0;
	pos=1;
	
	for(int num=fg;num>0;num--){
		
	if((data[flag[num]]==0x24)){
		if((data[flag[num]+4]==0x4D)&&(data[flag[num]+5]==0X43)){
		//printf("%d\r\n",flag[num]);
		while(pos<100){
		printf("%c",data[flag[num]+pos]);//debug
		//GNRMC[pos]=data[flag[num]+pos];
		GNRMC[pos]=data[flag[num]+pos]-'0';
		//printf("%c---%d\r\n",data[flag[num]+pos],pos);
		if(data[flag[num]+pos]==0x24){pos=100;}
		if(data[flag[num]+pos]==0x2c){pos1++;}
		if(data[flag[num]+pos]==0x2E){pos2++;}
		if((pos1==8)&&(data[flag[num]+pos]==0x2c)){buf[0]=pos;}//printf("buf0:%d\r\n",pos);}
		if((pos1==9)&&(data[flag[num]+pos]==0x2c)){buf[1]=pos;}//printf("buf1:%d\r\n",pos);}
		if((pos2==5)&&(data[flag[num]+pos]==0x2E)){buf[2]=pos;}//printf("buf2:%d\r\n",pos);}
		pos++;
		}	
		}
	}
}
}
//void Data_solove(){//老版本 数据解码 无法正常解除方向参数
//	weidu=GNRMC[20]*10+GNRMC[21]+(GNRMC[22]*10+GNRMC[23]+GNRMC[25]*0.1+GNRMC[26]*0.01+GNRMC[27]*0.001+GNRMC[28]*0.0001+GNRMC[29]*0.00001)/60;
//	jingdu=GNRMC[33]*100+GNRMC[34]*10+GNRMC[35]+(GNRMC[36]*10+GNRMC[37]+GNRMC[39]*0.1+GNRMC[40]*0.01+GNRMC[41]*0.001+GNRMC[42]*0.0001+GNRMC[43]*0.00001)/60;
//	way=GNRMC[53]*100+GNRMC[54]*10+GNRMC[55]+GNRMC[57]*0.1+GNRMC[58]*0.01;
//	//printf("%f----%f----%f\r\n",weidu,jingdu,way);
//}	

void Data_solove(){//对数据进行解码 解的初始的经纬度数据和 方向数据用于导航
	weidu=GNRMC[20]*10+GNRMC[21]+(GNRMC[22]*10+GNRMC[23]+GNRMC[25]*0.1+GNRMC[26]*0.01+GNRMC[27]*0.001+GNRMC[28]*0.0001+GNRMC[29]*0.00001)/60;
	jingdu=GNRMC[33]*100+GNRMC[34]*10+GNRMC[35]+(GNRMC[36]*10+GNRMC[37]+GNRMC[39]*0.1+GNRMC[40]*0.01+GNRMC[41]*0.001+GNRMC[42]*0.0001+GNRMC[43]*0.00001)/60;
	way=0;
	for(int n=1;n<(buf[2]-buf[0]);n++){
			way=GNRMC[buf[0]+n]*pow(10,buf[2]-buf[0]-n-1)+way;
	}
}
int target_find(){//导航程序主体
	double N_target;
	double W_target;
	double lng,lat;
	printf("test\r\n");
	while(daohangcanshu<=49){//导航停止判断}{
	lng=Way_Point[daohangcanshu++][0];
	lat=Way_Point[daohangcanshu++][1];
	}

	lng=lng-jingdu;
	lat=lat-weidu;
//	printf("lng--%f-jingdu--%f\r\n",lng,jingdu);
//	printf("lat--%f\r\n",lat);
	N_target=xiangchajiao(lng,lat,lng,0);
	W_target=xiangchajiao(lng,lat,0,lat);
//	printf("N_target--%f\r\n",N_target);
//	printf("W_target--%f\r\n",W_target);
	if((N_target<180)&&(W_target<180)){target=360-N_target;}
	if((N_target<90)&&(W_target<90)){target=N_target;}
	if(N_target>W_target){target=N_target;}
	if(N_target<W_target){target=360-N_target;}
	jiaodu=target-way;
	
	if(jiaodu<-180){jiaodu=jiaodu+360;}
	if(jiaodu>90){return 1;}
	if(jiaodu<-90){return 1;}// 证明目标位置在现在方向后方
	jiaodu=(jiaodu+90)/180*8+24;//直接 变成角度修正
	TIM1->CCR1=jiaodu;//进行角度修改 
	printf("角度 占空比:%f\r\n",jiaodu);
	return 0;
	
}
void data_get(){
	fg=1;
	while(!((UART_GetFlagStatus(UART2,UART_FLAG_RXAVL)==SET)&&(UART_ReceiveData(UART2)==0x24)));
	for(int a=0;a<2048;a++){
	while((UART_GetFlagStatus(UART2,UART_FLAG_RXAVL)!=SET));
	Data_Buf[a]=UART_ReceiveData(UART2);
	if((Data_Buf[a]==0x24)&&(a<1300)){flag[fg]=a;fg++;}
	}
}
void GPS_UpDate(){
	data_get();
	GNRMC_Check(Data_Buf);
	Data_solove();
	wgs84_to_gcj02(jingdu,weidu);
	while(target_find()==1);//如果返回值不为{
	if(daohangcanshu>49){//证明导航结束 //此处 进行刹车制动
	TIM3->CCR1=28;//导航方向归零
		UART_SendData(UART1, 0x00);//串口沟通 电机驱动 降速刹车
	}
	
	printf("纬度:%f\r\n精度:%f\r\n朝向:%f\r\n目标方向:%f\r\n",weidu,jingdu,way,target);//debug 
}
