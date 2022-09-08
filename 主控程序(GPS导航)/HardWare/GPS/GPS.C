#include "GPS.H"
#include "USART.H"
#include "math.h"
#include "vector.h"
double x_pi = 52.35987755983;  //坐标转换 参数 本程序中 坐标均有 地球坐标系 转换为 火星坐标系进行运作
double pi = 3.1415926535897932384626 ; // π
double a = 6378245.0;  //长半轴
double ee = 0.00669342162296594323;  // 扁率
double target;
char Data_Buf[2048];
char GNRMC[100];
double History_Point[100][2];
double jingdu,weidu,way,jiaodu,loca[2];//对 经度 维度 方向 转向角度 和一个 中间中继变量 进行 初始化
int flag[30],fg=0,buf[3];  //解码中, . 点位置的存储
int pos,daohangcanshu=1,Point_Sum=14;	
int History_Index=0;

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
112.39795887,34.58948150,
112.39808286,34.58942847,
112.39817915,34.58933676,
112.39823794,34.58926836,
112.39830806,34.58917665,
112.39836225,34.58911470,
112.39840736,34.58905118,
112.39845642,34.58897619,
112.39850285,34.58888841,
112.39856201,34.58879031,
112.39862324,34.58871573,
112.39871442,34.58862173,
112.39881064,34.58853410,
112.39885533,34.58846321,
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
//	way=0;
//	for(int n=1;n<(buf[2]-buf[0]);n++){
//			way=GNRMC[buf[0]+n]*pow(10,buf[2]-buf[0]-n-1)+way;
//	}
}
double Find_target(double O_Lng,double O_Lat,double N_Lng,double N_Lat){
	double N_target,W_target;
	double lng,lat;
	lng=N_Lng-O_Lng;
	lat=N_Lat-O_Lat;
	printf("lng  %f\r\n",lng);
	printf("lat  %f\r\n",lat);
	N_target=xiangchajiao(lng,lat,abs(lng),0);
	printf("N_target  %f\r\n",N_target);
	W_target=xiangchajiao(lng,lat,0,abs(lat));
	printf("W_target  %f\r\n",W_target);
	if((N_target<90)){if(W_target<90){target=N_target;}
										else{target=360-N_target;}}
	else{if(W_target<90){target=90+W_target;}
			else{target=180+N_target;}}
	return target;
}
double Find_Way(double lng,double lat ){
	History_Point[History_Index++][0]=lng;
	History_Point[History_Index][1]=lat;
	if(History_Index>2){
	way=Find_target(History_Point[History_Index][0],History_Point[History_Index][1],History_Point[History_Index-1][0],History_Point[History_Index-1][1]);
	}
	else{way=0;}
	return way;
}
int target_find(){//导航程序主体
	double N_target;
	double W_target;
	double lng,lat;
	lng=Way_Point[daohangcanshu][0];
	lat=Way_Point[daohangcanshu][1];
	printf("路径点:%d\r\n",daohangcanshu);
	lng=lng-jingdu;
	lat=lat-weidu;
	target=Find_target(jingdu,weidu,lng,lat);
	way=Find_Way(lng,lat);
//	N_target=xiangchajiao(lng,lat,lng,0);
//	W_target=xiangchajiao(lng,lat,0,lat);
//	if((N_target<180)&&(W_target<180)){target=360-N_target;}
//	if((N_target<90)&&(W_target<90)){target=N_target;}
//	if(N_target>W_target){target=N_target;}
//	if(N_target<W_target){target=360-N_target;}
	jiaodu=target-way;
	if(jiaodu<-180){jiaodu=jiaodu+360;}
	if(jiaodu>90){daohangcanshu++; return 1;}
	if(jiaodu<-90){daohangcanshu++;return 1;}// 证明目标位置在现在方向后方
	jiaodu=jiaodu/90*8+24;//直接 变成角度修正
	TIM3->CCR1=jiaodu	;//进行角度修改 
	printf("角度 占空比:%f\r\n",jiaodu);
	return 0;
	
}
void data_get(){
	fg=1;
	while(!((UART_GetFlagStatus(UART3,UART_FLAG_RXAVL)==SET)&&(UART_ReceiveData(UART3)==0x24)));
	for(int a=0;a<2048;a++){
	while((UART_GetFlagStatus(UART3,UART_FLAG_RXAVL)!=SET));
	Data_Buf[a]=UART_ReceiveData(UART3);
	if((Data_Buf[a]==0x24)&&(a<1300)){flag[fg]=a;fg++;}
	}
}
void GPS_UpDate(){
	data_get();
	GNRMC_Check(Data_Buf);
	Data_solove();
	wgs84_to_gcj02(jingdu,weidu);
	printf("纬度:%f\r\n经度:%f\r\n朝向:%f\r\n目标方向:%f\r\n角度:%f\r\n",weidu,jingdu,way,target,jiaodu);//debug 
	target_find();//如果返回值不为{
	if(daohangcanshu>=Point_Sum){//证明导航结束 //此处 进行刹车制动
	TIM3->CCR1=24;//导航方向归零
	UART_SendData(UART4, 0x00);//串口沟通 电机驱动 降速刹车
	printf("End\r\n");
	while(1);
	}
	
}
