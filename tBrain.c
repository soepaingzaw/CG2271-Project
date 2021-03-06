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

#define MSG_COUNT 1

osThreadId_t t_Brain, t_Audio , red_Thread,green_Thread, t_MotorControl; 

osMessageQueueId_t audioQ,brainQ,redQ,greenQ,motorQ;

osEventFlagsId_t led_flag;

volatile uint8_t rx_data = 0x00;

typedef struct {
	uint8_t cmd;
	uint8_t data;
	
} packet;

/*
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
*/

void initClockGate() {
	// Enable Clock Gating for PORTB and PORTE
	SIM->SCGC5 |= (SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTE_MASK) | (SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTD_MASK);
	// Enable Clock Gating for UART2
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
		//Enable clock gating for Timer0, Timer 1 and Timer 2
	SIM->SCGC6 = (SIM_SCGC6_TPM0_MASK) | (SIM_SCGC6_TPM1_MASK) | (SIM_SCGC6_TPM2_MASK);
}

void initUART2(uint32_t baud_rate){
	
	uint32_t divisor, bus_clock;

	
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
		//osMessageQueuePut(tBrain,&rx_data,NULL,0);
		
		
		if(rx_data == 0x21){
			//osMessageQueuePut(audioQ, &rx_data, NULL, 0);
			bluetoothConnected();
		}
	
	} 
	
	PORTD->ISFR = 0xffffffff;
	
}




void PORTD_IRQHandler() {
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	delay(0x100000);
	//osSemaphoreRelease(mySem);
	
	PORTD->ISFR |= MASK(DPIN6);
	
	
}



void greenThread() {
	uint8_t rx_g;	
	
	for(;;) {
		
		osMessageQueueGet(greenQ,&rx_g,NULL,osWaitForever);
		if(rx_g==0x21){
			onGreen();
			osDelay(500);
			offGreen();
			osDelay(500);
		}
	}
	
	
}
/*
void redThread() {
	uint8_t rx_r = rx_data;	
	
	for(;;) {
		
		osMessageQueueGet(greenQ,&rx_r,NULL,osWaitForever);
		if(rx_r==0x21){
			onGreen();
			osDelay(500);
			offGreen();
			osDelay(500);
		}
	}
	
	
}
*/

void tAudio() {
	
	uint8_t rx_a;
	
	
	for(;;){
		osMessageQueueGet(audioQ,&rx_a,NULL,osWaitForever);
		if(rx_a==0x21) {
			bluetoothConnected();
			rx_a==0x25;
			osMessageQueuePut(audioQ, &rx_a, NULL, 0);	
		}
		else if(rx_a==0x23) {
			finishRun();
		}
		else if(rx_a==0x25){
			maryHadALittleLamb();
		}
		
		
	}
	
}

void tBrain(void *argument) {
	
	//uint8_t rx_b = rx_data;
	uint8_t rx_b = 0x00;
	
  for(;;) {
		//osMessageQueueGet(audioQ,&rx_b,NULL,osWaitForever);
		//osMessageQueuePut(motorQ, &rx_b, NULL, 0);
		//osMessageQueuePut(redQ, &rx_b, NULL, 0); 
		//osDelay(10);
		//osMessageQueuePut(greenQ, &rx_b, NULL, 0); 
		//osDelay(10);
	
			osMessageQueuePut(audioQ, &rx_b, NULL, 0);	
	}			
	

}


int main (void) {
 
    SystemCoreClockUpdate();
	
		
	  initClockGate();
	  initUART2(BAUD_RATE);
	  initAudio();
	  initPWM();
		initLED();
		//initSwitch();

		offRGB();
		
    osKernelInitialize();   

	

	
	
	
		osThreadNew(tBrain, NULL, NULL);
	  brainQ = osMessageQueueNew(MSG_COUNT,sizeof(packet),NULL);
	
	  osThreadNew(tAudio,NULL,NULL);
	  audioQ = osMessageQueueNew(MSG_COUNT,sizeof(packet),NULL);
	  //startAudioQ = osMessageQueueNew(MSG_COUNT,sizeof(packet),NULL);
	
	/*
		osThreadNew(redThread, NULL, NULL);
		redQ = osMessageQueueNew(MSG_COUNT,sizeof(packet),NULL);
		*/
		//osThreadNew(greenThread, NULL, NULL);	
		//greenQ = osMessageQueueNew(MSG_COUNT,sizeof(packet),NULL);
/*
		osThreadNew(tMotorControl, NULL, NULL);
	  motorQ = osMessageQueueNew(MSG_COUNT, sizeof(uint8_t), NULL);
	*/
	
	

	
	
	

  osKernelStart();      
  for (;;) {}
}
