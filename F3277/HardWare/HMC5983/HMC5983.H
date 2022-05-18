#ifndef _HMC5983_H
#define _HMC5983_H

#include "SETTING.H"
#include "IIC.H"
#include "USART.H"

#define HMC5983 0x3C
#define CRA 0x00
#define CRB 0x01
#define conf 0x02
#define mode2 0x00	
#define Set 0x70
#define Gain 0xA0
#define Data 0x06

#define X_MSB 0x03
#define X_LSB 0x04
#define Z_MSB 0x05
#define Z_LSB 0x06
#define Y_MSB 0x07
#define Y_LSB 0x08

void HMC5983_Init();
double HMC5983_Read();
#endif 