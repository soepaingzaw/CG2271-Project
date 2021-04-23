#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.H"
#include "stdio.h"
#include "time.h"

#define DELAYFORSTATIONARYRED 500
#define DELAYFORLEDPERIOD 1000
#define DELAYFORREMAININGTIME 750
#define TENMICROSECONDS 0x30


#define MOVING_MASK(x) (x&0x01)
#define MASK(x) (1 << (x))

#define MSG_COUNT 1

#define TRIGPIN 5
#define ECHOPIN 6

#define SELFDRIVING 0x51

#define OFF 0x00
#define CONNECT 0x20
#define ENDCHALLENGE 0x22
#define FORWARD 0x11
#define REVERSE 0x13
#define LEFTFORWARD 0x15
#define RIGHTFORWARD 0x17
#define ROTATERIGHT 0x1B
#define ROTATELEFT 0x19
#define LEFTREVERSE 0x1D
#define RIGHTREVERSE 0x1F
#define RIGHTCIRCLE 0x51
#define STOP 0x10

#define DELAYFORSELFDRIVINGLEFT 370
#define DELAYFORSTOPSTATE 500

void initUltraSound(void);
void onTrigPin(void);
void offTrigPin(void);
void delay(volatile uint32_t nof);

void stopState(osMessageQueueId_t,osMessageQueueId_t, osMessageQueueId_t,osMessageQueueId_t,int8_t);
void leftState(osMessageQueueId_t,osMessageQueueId_t, osMessageQueueId_t,osMessageQueueId_t,int8_t);
void circleState(osMessageQueueId_t,osMessageQueueId_t, osMessageQueueId_t,osMessageQueueId_t,uint8_t,uint8_t);
void forwardState(osMessageQueueId_t,osMessageQueueId_t, osMessageQueueId_t,osMessageQueueId_t,int8_t);

