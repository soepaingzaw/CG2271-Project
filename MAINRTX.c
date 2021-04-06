/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.H"
#include "stdio.h"
#include "time.h"
#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK(x) (1 << (x))

#define UART_RX_PORTE23 23
#define BAUD_RATE 9600

#define DPIN6 6

osMutexId_t myMutex;
osSemaphoreId_t mySem;
osSemaphoreId_t blueSem;

volatile uint8_t rx_data = 0x00;

void initGPIO(void)
{
// Enable Clock to PORTB and PORTD
SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
// Configure MUX settings to make all 3 pins GPIO
PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
// Set Data Direction Registers for PortB and PortD
PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
PTD->PDDR |= MASK(BLUE_LED);
}

void initUART2(uint32_t baud_rate){
	
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE-> PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE-> PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	bus_clock = DEFAULT_SYSTEM_CLOCK/2;
	divisor = bus_clock / (baud_rate*16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	UART2->C2 |= (UART_C2_RE_MASK);
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_RIE_MASK;

}

void initSwitch(void) {

  PORTD->PCR[DPIN6] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[DPIN6] |= (PORT_PCR_MUX(1)|PORT_PCR_PS_MASK|
	PORT_PCR_PE_MASK|PORT_PCR_IRQC(0x0a));
// Set Data Direction Registers for PortB and PortD

  PTD->PDDR &= ~MASK(DPIN6);
	
	NVIC_SetPriority(PORTD_IRQn,0);
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
	

	
}

void delay(volatile uint32_t nof) {
		while (nof!=0) {
			//__ASM("NOP");
			nof--;
		}
		
}

void redOn(void) {
		PTB->PCOR = MASK(RED_LED);
		
		//PTB->PSOR = MASK(GREEN_LED);
		
		//PTD->PSOR = MASK(BLUE_LED);

}

void redOff(void) {
	PTB->PSOR = MASK(RED_LED);

}

void greenOn(void) {
		//PTB->PSOR = MASK(RED_LED);
		
		PTB->PCOR = MASK(GREEN_LED);
		
		//PTD->PSOR = MASK(BLUE_LED);

}

void greenOff(void) {
	PTB->PSOR = MASK(GREEN_LED);
}

void blueOn(void) {
		PTB->PSOR = MASK(RED_LED);
		
		PTB->PSOR = MASK(GREEN_LED);
		
		PTD->PCOR = MASK(BLUE_LED);

}

void blueOff(void) {
		
		
		PTD->PSOR = MASK(BLUE_LED);

}

void offRGB(void) {
		PTB->PSOR = MASK(RED_LED);
		
		PTB->PSOR = MASK(GREEN_LED);
		PTD->PSOR = MASK(BLUE_LED);
}

void UART2_IRQHandler(void) {
	
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	//delay(0x100000);
	if (UART2->S1 & UART_S1_RDRF_MASK) {
	// received a character
		rx_data = UART2->D;// remove transmit functionality
		
		if(rx_data == 0x01) {
			osSemaphoreRelease(mySem);
		}
	
		else if(rx_data ==0x03) {
			osSemaphoreRelease(blueSem);
		}

	
		
	} 
	
	PORTD->ISFR = 0xffffffff;
	
}




void PORTD_IRQHandler() {
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	delay(0x100000);
	osSemaphoreRelease(mySem);
	
	PORTD->ISFR |= MASK(DPIN6);
	
	
}
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void led_red_thread (void *argument) {
 
  // ...
  for (;;) {
		//osMutexAcquire(myMutex, osWaitForever);
		osSemaphoreAcquire(mySem,osWaitForever);
	    redOn();
		  osDelay(1000);
		  //delay(0x200000);
			redOff();
			osDelay(1000);
		  //delay(0x200000);
		//osMutexRelease(myMutex);
		//osSemaphoreRelease(mySem);
	}
}

void led_green_thread(void *argument) {
	
	for(;;) {
		//osMutexAcquire(myMutex, osWaitForever);
		osSemaphoreAcquire(mySem,osWaitForever);
		greenOn();
		osDelay(1000);
		//delay(0x200000);
		greenOff();
		osDelay(1000);
		//delay(0x200000);
		//osMutexRelease(myMutex);
		//osSemaphoreRelease(mySem);
		
	}
}

void led_blue_thread(void *argument) {
	
	for(;;) {
		//osMutexAcquire(myMutex, osWaitForever);
		osSemaphoreAcquire(blueSem,osWaitForever);
		blueOn();
		osDelay(1000);
		//delay(0x200000);
		blueOff();
		osDelay(1000);
		//delay(0x200000);
		//osMutexRelease(myMutex);
		//osSemaphoreRelease(mySem);
		
	}
}
 
int main (void) {
 
  SystemCoreClockUpdate();
	
		initUART2(BAUD_RATE);
		initGPIO();
		initSwitch();

		offRGB();
		
  osKernelInitialize();       
  //myMutex = osMutexNew(NULL);
		mySem = osSemaphoreNew(1,0,NULL);
		blueSem = osSemaphoreNew(1,0,NULL); //semaphore fo blue thread

		osThreadNew(led_green_thread, NULL, NULL);
		osThreadNew(led_red_thread, NULL, NULL);
	  osThreadNew(led_blue_thread, NULL, NULL);
	

  osKernelStart();      
  for (;;) {}
}
