#include "pid.h"

/*******************************

参数整定找最佳，从小到大顺序查
先是比例后积分，最后再把微分加
曲线振荡很频繁，比例度盘要放大
曲线漂浮绕大湾，比例度盘往小扳
曲线偏离回复慢，积分时间往下降
曲线波动周期长，积分时间再加长
曲线振荡频率快，先把微分降下来
动差大来波动慢，微分时间应加长
理想曲线两个波，前高后低4比1
一看二调多分析，调节质量不会低

********************************/
#define Kp 30
#define Ki 0	
#define Kd 2
double Now_Error,Sum_Error,Last_Error,Hst_Error;

double P_Caculate(){
	double val;
	val=Now_Error*Kp;
	return val;
}
double I_Caculate(){
	double val;
	val=Sum_Error*Ki;
	return val;
}
double D_Caculate(){
	double val;
	val=Kd*Hst_Error;
	return val;
}


double PID(double New_Error){
	double val;
	Now_Error=New_Error;
	Sum_Error=Sum_Error+Now_Error;
	if(Sum_Error>100){Sum_Error=100;}
	if(Sum_Error<-100){Sum_Error=-100;}
	Hst_Error=Last_Error-Now_Error;
	val=P_Caculate()+I_Caculate()+D_Caculate();
	//printf("val=%f\r\n",val);
	Last_Error=Now_Error;
	if(val>50){val=50;}
	if(val<15){val=15;}
	return val;
}










