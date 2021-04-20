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

#define TRIGPIN 5
#define ECHOPIN 6

#define SELFDRIVINGLEFT 370

#define FORWARD 0x11
#define REVERSE 0x13
#define LEFTFORWARD 0x15
#define RIGHTFORWARD 0x17
#define ROTATERIGHT 0x1B
#define ROTATELEFT 0x19
#define LEFTREVERSE 0x1D
#define RIGHTREVERSE 0x1F
#define SELFDRIVING 0x51
#define OFFMOTORS 0x10


#define ENABLE_MASK(x) (x&0x01)


#define MODE_MASK(x) (x&0xF0)

#define FW 11
#define RV 9
#define LT 13
#define RT 15


#define TPMMOD 0x0EA6

void initMotorPWM(void);
	
void initMotor(void);
void fw(int ms);
void rv(int ms);
void leftForward(int ms);
void rightForward(int ms);
void leftReverse(int ms);
void rightReverse(int ms);
void offMotors(void);
void leftSharp(int ms);
void rightSharp(int ms);
void rightCircle(int ms);

void initUltraSound(void);
void onTrigPin(void);
void offTrigPin(void);
void delay(volatile uint32_t nof);
void sensor(void);

void selfDriving(void);
