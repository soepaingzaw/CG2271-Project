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
 #define RED_MOVE_DELAY 500
 #define RED_STOP_DELAY 250
 #define GREEN_DELAY 100
 
 unsigned int moving = 0;
 
 void InitGPIO(void){
	// Enable Clock to PORTA and PORTC and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK));
	// Configure MUX settings to make all 3 pins GPIO
	PORTD->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[RED_LED] |= PORT_PCR_MUX(1);
	PTD->PDDR |= MASK(RED_LED);
	 
	PORTA->PCR[GREEN_LEDA] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LEDA] |= PORT_PCR_MUX(1);
	PTA->PDDR |= MASK(GREEN_LEDA);
	PORTA->PCR[GREEN_LEDB] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LEDB] |= PORT_PCR_MUX(1);
	PTA->PDDR |= MASK(GREEN_LEDB);
	PORTD->PCR[GREEN_LEDC] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LEDC] |= PORT_PCR_MUX(1);
	 PTD->PDDR |= MASK(GREEN_LEDC);
	PORTA->PCR[GREEN_LEDD] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LEDD] |= PORT_PCR_MUX(1);
	PTA->PDDR |= MASK(GREEN_LEDD);
	PORTA->PCR[GREEN_LEDE] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LEDE] |= PORT_PCR_MUX(1);
	PTA->PDDR |= MASK(GREEN_LEDE);
	PORTA->PCR[GREEN_LEDF] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LEDF] |= PORT_PCR_MUX(1);
	PTA->PDDR |= MASK(GREEN_LEDF);
	PORTC->PCR[GREEN_LEDG] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LEDG] |= PORT_PCR_MUX(1);
	PTC->PDDR |= MASK(GREEN_LEDG);
	PORTC->PCR[GREEN_LEDH] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LEDH] |= PORT_PCR_MUX(1);
	PTC->PDDR |= MASK(GREEN_LEDH);
	PORTC->PCR[GREEN_LEDI] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LEDI] |= PORT_PCR_MUX(1);
	PTC->PDDR |= MASK(GREEN_LEDI);
	PORTC->PCR[GREEN_LEDJ] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LEDJ] |= PORT_PCR_MUX(1);
	PTC->PDDR |= MASK(GREEN_LEDJ);
}
 
void onGreen(void) {
	PTA->PSOR |= MASK(GREEN_LEDA);
	PTA->PSOR |= MASK(GREEN_LEDB);
	PTD->PSOR |= MASK(GREEN_LEDC);
	PTA->PSOR |= MASK(GREEN_LEDD);
	PTA->PSOR |= MASK(GREEN_LEDE);
	PTA->PSOR |= MASK(GREEN_LEDF);
	PTC->PSOR |= MASK(GREEN_LEDG);
	PTC->PSOR |= MASK(GREEN_LEDH);
	PTC->PSOR |= MASK(GREEN_LEDI);
	PTC->PSOR |= MASK(GREEN_LEDJ);
}

void offGreen(void) {
	PTA->PCOR |= MASK(GREEN_LEDA);
	PTA->PCOR |= MASK(GREEN_LEDB);
	PTD->PCOR |= MASK(GREEN_LEDC);
	PTA->PCOR |= MASK(GREEN_LEDD);
	PTA->PCOR |= MASK(GREEN_LEDE);
	PTA->PCOR |= MASK(GREEN_LEDF);
	PTC->PCOR |= MASK(GREEN_LEDG);
	PTC->PCOR |= MASK(GREEN_LEDH);
	PTC->PCOR |= MASK(GREEN_LEDI);
	PTC->PCOR |= MASK(GREEN_LEDJ);
}

void green_led (void *argument) {
 
  // ...
  for (;;) {
		if (moving == 1) {
			offGreen();
			PTA->PSOR |= MASK(GREEN_LEDA);
			osDelay(GREEN_DELAY);
			PTA->PCOR |= MASK(GREEN_LEDA);
			PTA->PSOR |= MASK(GREEN_LEDB);
			osDelay(GREEN_DELAY);
			PTA->PCOR |= MASK(GREEN_LEDB);
			PTD->PSOR |= MASK(GREEN_LEDC);
			osDelay(GREEN_DELAY);
			PTD->PCOR |= MASK(GREEN_LEDC);
			PTA->PSOR |= MASK(GREEN_LEDD);
			osDelay(GREEN_DELAY);
			PTA->PCOR |= MASK(GREEN_LEDD);
			PTA->PSOR |= MASK(GREEN_LEDE);
			osDelay(GREEN_DELAY);
			PTA->PCOR |= MASK(GREEN_LEDE);
			PTA->PSOR |= MASK(GREEN_LEDF);
			osDelay(GREEN_DELAY);
			PTA->PCOR |= MASK(GREEN_LEDF);
			PTC->PSOR |= MASK(GREEN_LEDG);
			osDelay(GREEN_DELAY);
			PTC->PCOR |= MASK(GREEN_LEDG);
			PTC->PSOR |= MASK(GREEN_LEDH);
			osDelay(GREEN_DELAY);
			PTC->PCOR |= MASK(GREEN_LEDH);
			PTC->PSOR |= MASK(GREEN_LEDI);
			osDelay(GREEN_DELAY);
			PTC->PCOR |= MASK(GREEN_LEDI);
			PTC->PSOR |= MASK(GREEN_LEDJ);
			osDelay(GREEN_DELAY);
		} else {
			onGreen();
			osDelay(GREEN_DELAY);
			onGreen();
			osDelay(GREEN_DELAY);
		}
	}
}

void red_led (void *argument) {
	
	for (;;) {
		if (moving == 1) {
			PTD->PSOR |= MASK(RED_LED);
			osDelay(RED_MOVE_DELAY);
			PTD->PCOR |= MASK(RED_LED);
			osDelay(RED_MOVE_DELAY);
		} else {
			PTD->PSOR |= MASK(RED_LED);
			osDelay(RED_STOP_DELAY);
			PTD->PCOR |= MASK(RED_LED);
			osDelay(RED_STOP_DELAY);
		}
	}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
  // ...
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(green_led, NULL, NULL);    // Create application main thread
  osThreadNew(red_led, NULL, NULL); 
	osKernelStart();                      // Start thread execution
  for (;;) {}
}
