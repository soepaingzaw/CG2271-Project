#include "tLED.h"
 
 /*INITIALISE ALL LED COMPONENTS */
 void initLED(void){
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
 
 /* TURN ON GREEN LED */
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

/* TURN OFF GREEN LED */
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

/* GREEN LED IN RUNNING MODE, ONE LED AT A TIME */
void movingGreen(void) {
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

}

//GREEN LED FLASH TWICE UPON SUCCESSFUL
//CONNECTION WITH BLUETOOTH
void greenLEDBluetoothOn(void){
	onGreen();
	osDelay(CONNECTED_DELAY);
	offGreen();
	osDelay(CONNECTED_DELAY);	
	onGreen();
	osDelay(CONNECTED_DELAY);
	offGreen();
	osDelay(CONNECTED_DELAY);	
	onGreen();
	  
}

/* RED LED BLINKS 500ms ON 500ms OFF WHEN MOVING*/
void movingRed(void){
	PTD->PSOR |= MASK(RED_LED);
	osDelay(RED_MOVE_DELAY);
	PTD->PCOR |= MASK(RED_LED);
	osDelay(RED_MOVE_DELAY);

}

/* RED LED BLINKS 250ms ON 250ms OFF WHEN STATIONARY*/
void stationaryRed(void){
	PTD->PSOR |= MASK(RED_LED);
	osDelay(RED_STOP_DELAY);
	PTD->PCOR |= MASK(RED_LED);
	osDelay(RED_STOP_DELAY);

}



