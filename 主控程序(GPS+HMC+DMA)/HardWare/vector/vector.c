#include "vector.h"
double P = 3.1415926535897932384626 ; // π

double xiangchajiao(double x1,double y1,double x2,double y2){
	return acos((x1*x2+y1*y2)/(sqrt(x1*x1+y1*y1)*sqrt(x2*x2+y2*y2)))*180/P;
}
double Way_Limite(double Way){
	if(Way>360){Way=Way-360;}
	if(Way<0){Way=Way+360;}
	return Way;
}
double Find_Exchange_Way(double Now_Way,double Target_Way){//以当前方向为基准进行导航
	double val;
	val=Way_Limite(Target_Way-Now_Way);
	if(val>180&&val<360){val=val-360;}
	return val;
}