#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.H"
#include "stdio.h"
#include "time.h"

#define MASK(x) (1 << (x))

#define PTB3 3
#define PTB2 2

#define PTB1 1
#define PTB0 0
#define PTA12 12

#define FW 11
#define RV 9
#define LT 13
#define RT 15

#define TPMMOD 3750

void initMotorPWM(void);
	
void initMotor(void);
void fw(void);
void rv(void);
void leftForward(void);
void rightForward(void);
void leftReverse(void);
void rightReverse(void);
void offMotors(void);
void leftSharp(void);
void rightSharp(void);
void rightCircle(void);

