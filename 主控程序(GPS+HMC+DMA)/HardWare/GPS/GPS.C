#include "GPS.H"
#include "USART.H"
#include "math.h"
#include "vector.h"
#include "HMC5983.H"
double x_pi = 52.35987755983;  //坐标转换 参数 本程序中 坐标均有 地球坐标系 转换为 火星坐标系进行运作
double pi = 3.1415926535897932384626 ; // π
double a = 6378245.0;  //长半轴
double ee = 0.00669342162296594323;  // 扁率
double Base_way;
double target,lng_val,lat_val;
u8 Data_Buf[512];
int daohangcanshu=1,Point_Sum=11;	
nmea_msg GNRMC;
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
	lat_val = lat + dlat;//维度
	lng_val = lng + dlng;//经度
}

double Way_Point[50][2]={
112.40512 ,	34.587423,
112.405066,	34.587498,
112.404991, 34.5876  ,
112.404916,	34.587684,
112.404846,	34.587785,
112.404765,	34.587883,
112.40468	, 34.587975,
112.404594,	34.588077,
112.404508,	34.588192,
112.404417,	34.588311,
112.40432 ,	34.588421,

};
//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号                             
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{               
    u8 *p=buf;
    while(cx)
    {        
        if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
        if(*buf==',')cx--;
        buf++;
    }
    return buf-p;   //返回差值，
}
//m^n函数
//返回值:m^n次方.
u32 NMEA_Pow(u8 m,u8 n)
{
    u32 result=1;    
    while(n--)result*=m;    
    return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(u8 *buf,u8*dx)
{
    u8 *p=buf;
    u32 ires=0,fres=0;
    u8 ilen=0,flen=0,i;
    u8 mask=0;
    int res;
    while(1) //得到整数和小数的长度
    {
        if(*p=='-'){mask|=0X02;p++;}//是负数
        if(*p==','||(*p=='*'))break;//遇到结束了
        if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
        else if(*p>'9'||(*p<'0'))   //有非法字符
        {   
            ilen=0;
            flen=0;
            break;
        }   
        if(mask&0X01)flen++;
        else ilen++;
        p++;
    }
    if(mask&0X02)buf++; //去掉负号
    for(i=0;i<ilen;i++) //得到整数部分数据
    {  
        ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
    }
    if(flen>5)flen=5;   //最多取5位小数
    *dx=flen;           //小数点位数
    for(i=0;i<flen;i++) //得到小数部分数据
    {  
        fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
    } 
    res=ires*NMEA_Pow(10,flen)+fres;
    if(mask&0X02)res=-res;         
    return res;
}   
	
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;           
    u8 posx;     
    u32 temp;      
    float rs;  

    p1=(u8*)strstr((const char *)buf,"GNR");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
		printf("P1内容:%s\r\n",p1);
    posx=NMEA_Comma_Pos(p1,1);                              //得到UTC时间
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);     //得到UTC时间,去掉ms
        gpsx->utc.hour=temp/10000;
        gpsx->utc.min=(temp/100)%100;
        gpsx->utc.sec=temp%100;      
    }   
    posx=NMEA_Comma_Pos(p1,3);                              //得到纬度
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx);          
        gpsx->latitude=temp/NMEA_Pow(10,dx+2);  //得到°
        rs=temp%NMEA_Pow(10,dx+2);              //得到'        
        gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
    }
    posx=NMEA_Comma_Pos(p1,4);                              //南纬还是北纬 
    if(posx!=0XFF)gpsx->nshemi=*(p1+posx);                   
    posx=NMEA_Comma_Pos(p1,5);                              //得到经度
    if(posx!=0XFF)
    {                                                 
        temp=NMEA_Str2num(p1+posx,&dx);          
        gpsx->longitude=temp/NMEA_Pow(10,dx+2); //得到°
        rs=temp%NMEA_Pow(10,dx+2);              //得到'        
        gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
    }
    posx=NMEA_Comma_Pos(p1,6);                              //东经还是西经
    if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);       
		posx=NMEA_Comma_Pos(p1,8);															//得到航向角
		if(posx!=0XFF)gpsx->direction=(double)NMEA_Str2num(p1+posx,&dx);
		if(dx==1){gpsx->direction=gpsx->direction/10;}
		if(dx==2){gpsx->direction=gpsx->direction/100;}
    posx=NMEA_Comma_Pos(p1,9);                              //得到UTC日期
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx);                     //得到UTC日期
        gpsx->utc.date=temp/10000;
        gpsx->utc.month=(temp/100)%100;
        gpsx->utc.year=2000+temp%100;        
    } 
		printf("(%d,%d)\r\n",gpsx->longitude,gpsx->latitude);
}
//		posx=NMEA_Comma_Pos(p1,8);															//得到航向角
//		if(posx!=0XFF)gpsx->direction=(double)NMEA_Str2num(p1+posx,&dx)*pow(10,-dx);

double Find_target(double O_Lng,double O_Lat,double N_Lng,double N_Lat){
	double N_target,W_target;
	double lng,lat;
	lng=(N_Lng-O_Lng);
	lat=(N_Lat-O_Lat);
//	printf("lng  %f\r\n",lng);
//	printf("lat  %f\r\n",lat);
	N_target=xiangchajiao(lng,lat,fabs(lng),0);//x轴
//	printf("N_target  %f\r\n",N_target);
	W_target=xiangchajiao(lng,lat,0,fabs(lat));//y轴
//	printf("W_target  %f\r\n",W_target);
	if((N_target<90)){if(W_target<90){target=W_target;}
										else{target=N_target+90;}}
	else{if(W_target<90){target=360-W_target;}
			else{target=360-W_target;}}
	return target;
}

int target_find(){//导航程序主体
	double N_target;
	double W_target;
	double lng,lat,True_N;
	lng=Way_Point[daohangcanshu][0];
	lat=Way_Point[daohangcanshu][1];
//	True_N=360;//Find_target(lng_val,lat_val,86.4,166.3);
	target=Find_target(lng_val,lat_val,lng,lat);
//	if(True_N>180){target=target+360-True_N;}
//	if(True_N<180){target=target-True_N;}
	if(target>360){target=target-360;}
	if(target<0){target=target+360;}
	if(target<270&&target>90){return 1;}
//	printf("当前位置:(%f,%f)\r\n目标位置:(%f,%f)\r\n两者方向:%f\r\n",lng_val,lat_val,lng,lat,target);
	return 0;
}

void Test(){
		
}


int Clear_GGA[11]={0xF1,0xD9,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F};
int Clear_GSA[11]={0xF1,0xD9,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13};
int Clear_GSV[11]={0xF1,0xD9,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17};
int Clear_TXT[11]={0xF1,0xD9,0x06,0x01,0x03,0x00,0xF0,0x40,0x00,0x3A,0x8F};
int Clear_ANT[11]={0xF1,0xD9,0x06,0x01,0x03,0x00,0xF0,0x20,0x00,0x1A,0x4F};
int Clear_ZDA[11]={0xF1,0xD9,0x06,0x01,0x03,0x00,0xF0,0x07,0x00,0x01,0x1D};
int Clear_RMC[11]={0xF1,0xD9,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19};
int Open_RMC[11]={0xF1,0xD9,0x06,0x01,0x03,0x00,0xF0,0x05,0x01,0x00,0x1A};
int Set_Fre[28]={0xF1,0xD9,0x06,0x42,0x14,0x00,0x00,0x0A,0x05,0x00,0x64,0x00,0x00,0x00 
						  ,0x60,0xEA,0x00,0x00,0xD0,0x07,0x00,0x00,0xC8,0x00,0x00,0x00,0xB8,0xED};
int RST[9]={0xF1,0xD9,0x06,0x40,0x01,0x00,0x01,0x48,0x22};

void GPS_Send(int t,int *p){
	for(int a=t;a>0;a--){
		while (UART_GetFlagStatus(UART3,UART_FLAG_TXEMPTY)!=SET);
		UART_SendData(UART3,*p);
		printf("%c",*p);
		p++;
	}
}

void GPS_Init(){
	GPS_Send(9,RST);
	GPS_Send(11,Clear_GGA);
	GPS_Send(11,Clear_GSA);
	GPS_Send(11,Clear_GSV);
	GPS_Send(11,Clear_TXT);
	GPS_Send(11,Clear_ANT);
	GPS_Send(11,Clear_ZDA);
//	GPS_Send(11,Clear_RMC);
	GPS_Send(28,Set_Fre);
}

void DMA1_Channel3_IRQHandler(void)
{
		printf("成功进入DMA中断\r\n");
    if(DMA_GetFlagStatus(DMA1_FLAG_TC3) != RESET)
    {
		DMA_ClearITPendingBit(DMA1_IT_TC3);
		DMA_Cmd(DMA1_ch3,DISABLE); //使能DMA1通道1		
		NMEA_GNRMC_Analysis(&GNRMC,&Data_Buf);
		for(int a=0;a<512;a++){
			if(Data_Buf[a]==0x00){Data_Buf[a]='$';}
		}
		DMA_Cmd(DMA1_ch3,ENABLE); //使能DMA1通道1
		
		wgs84_to_gcj02((double)(GNRMC.longitude)/100000,(double)(GNRMC.latitude)/100000);
		while(target_find())
			
			
		if(daohangcanshu>Point_Sum){
		UART_SendData(UART4,0X00);//刹车
		}
		
		}
		
}