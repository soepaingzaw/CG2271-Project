#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.H"
#include "stdio.h"
#include "time.h"

#define MASK(x) (1 << (x))
//Below is TPM1 CH1
#define PTD0 0
#define NUM_NOTES 30

void delay(volatile uint32_t nof);

void initAudio(void);
void initAudioPWM(void);

void maryHadALittleLamb(void);
void bluetoothConnected(void);
void finishRun(void);
