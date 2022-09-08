#include "vector.h"
double P = 3.1415926535897932384626 ; // ¦Ð
double xiangchajiao(double x1,double y1,double x2,double y2){
	return acos((x1*x2+y1*y2)/(sqrt(x1*x1+y1*y1)*sqrt(x2*x2+y2*y2)))*180/P;
}