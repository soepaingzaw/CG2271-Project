#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
#define RED_LED 5 //PORTD 5
#define GREEN_LEDA 1 // PORTA
#define GREEN_LEDB 2 // PORTA
#define GREEN_LEDC 4 // PORTD
#define GREEN_LEDD 12 // PORTA
#define GREEN_LEDE 4 // PORTA
#define GREEN_LEDF 5 // PORTA
#define GREEN_LEDG 8 // PORTC
#define GREEN_LEDH 9 // PORTC
#define GREEN_LEDI 7 // PORTC
#define GREEN_LEDJ 0 // PORTC
#define MASK(x) (1 << (x))
#define WAITINGDELAY 1000
#define RED_MOVE_DELAY 500
#define RED_STOP_DELAY 250
#define GREEN_DELAY 100
#define CONNECTED_DELAY 250
 
void initLED(void); 
void onGreen(void);
void offGreen(void);
void movingGreen(void) ;
void greenLEDBluetoothOn(void);
void movingRed(void);
void stationaryRed(void);

