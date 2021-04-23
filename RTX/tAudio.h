#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.H"
#include "stdio.h"
#include "time.h"

#define MASK(x) (1 << (x))

#define PTD0 0
#define NUM_NOTES_SONG 30
#define NUM_NOTES_START 3
#define NUM_NOTES_END 8

#define COUNTERFREQUENCY 375000
#define STARTDELAY 120
#define SONGDELAY 100

#define INITIALAUDIODELAY 1000

#define NOTE_REST 0
#define NOTE_C 262
#define NOTE_D 294
#define NOTE_E 330
#define NOTE_G 392

#define END 0x22

#define SHORTPAUSE 109
#define MEDPAUSE 164
#define LONGPAUSE 327

#define ENDDELAY 218

void delay(volatile uint32_t nof);
void initAudio(void);
void initAudioPWM(void);
void maryHadALittleLamb(uint8_t *);
void bluetoothConnected(void);
void finishRun(uint8_t *);
