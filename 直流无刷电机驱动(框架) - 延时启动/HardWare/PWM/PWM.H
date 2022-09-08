#ifndef PWM_H
#define PWM_H
void TIM1_PWM(int arr,int psc);
void TIM3_PWM(int arr,int psc);

#define CC1E  (0x0001) 		  //PB8  AH
#define CC1NE	(0x0004)	  //PB6  AL
#define CC2E	(CC1E<<4)			//PB5  BH
#define CC2NE	(CC1NE<<4)		//PB3	 CL
#define CC3E	(CC1E<<8)	  //PB7	 CH
#define CC3NE	(CC1NE<<8)	//PB4  BL
//#define CC4E	CC1E<<12
//#define CC4NE	CC1NE<<12

#define OUT1	CC1E+CC3NE // AH+BL
#define OUT2	CC1E+CC2NE //AH+CL
#define OUT3	CC2E+CC2NE //BH+CL
#define OUT4	CC1NE+CC2E //AL+BH
#define OUT5	CC1NE+CC3E //AL+CH
#define OUT6	CC3NE+CC3E //BL+CH

#endif