#include "SETTING.H"
#include "target.h"

int t=65535;



void Get_Yihuo(char *p){
	char Sum=*p;
	for(int a=9;a>0;a--){
		printf("%c=%x--%x\r\n",*p,*p,Sum);
		p++;
		Sum=*p^Sum;		
	}
}


int main(){
	Init_For_All_Device();
	while(1){
	Main_Loop();
	}
}