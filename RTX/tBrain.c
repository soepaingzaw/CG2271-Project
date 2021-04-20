#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.H"
#include "stdio.h"
#include "time.h"

#include "tMotor.h"
#include "tAudio.h"
#include "tLED.h"

#define START 0x21
#define END 0x23
#define RESET 0x25
#define CLEARFLAGS 0xFFFFFFFF

#define MASK(x) (1 << (x))

#define UART_RX_PORTE23 23
#define BAUD_RATE 9600

#define MSG_COUNT 1

osThreadId_t t_Brain, t_Audio, red_Thread, green_Thread, t_MotorControl; 
osMessageQueueId_t audioQ,brainQ,redQ,greenQ,motorQ;

volatile int flag = 1; 

uint8_t rx_data = 0x00;//disable all tasks initially

volatile int audioFlag = 0;


void initClockGate() {
	// Enable Clock Gating for GPIO
	SIM->SCGC5 |= (SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTE_MASK) | (SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTD_MASK)|(SIM_SCGC5_PORTC_MASK);
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

void UART2_IRQHandler(void) {
	
	NVIC_ClearPendingIRQ(UART2_IRQn);

	if (UART2->S1 & UART_S1_RDRF_MASK) {
	  // received a character
		rx_data = UART2->D;

		if(rx_data == START){
			bluetoothConnected();
			audioFlag = 1;
		}
		//press end challenge to play end music
		//end stop playing song
		else if(rx_data == END) {			
			offMotors();
			audioFlag = 0;
			finishRun();
		}
		//reset flag to enable self driving mode to run again
		else if(rx_data== RESET){
			flag = 1;
		}
	} 
	PORTE->ISFR = CLEARFLAGS;
}

void tMotorControl(void *argument) {
	
	uint8_t rx_m;	
	
	for(;;) {
		osMessageQueueGet(motorQ,&rx_m,NULL,osWaitForever);
		if(rx_m==FORWARD){
			fw(1);
		}
		else if(rx_m==REVERSE){
			rv(1);
		}
		else if(rx_m==LEFTFORWARD) {
			leftForward(1);		
		}
		else if(rx_m==RIGHTFORWARD) {
		
			rightForward(1);
		}
		else if(rx_m==ROTATERIGHT) {	
			rightSharp(1);
		}
		else if(rx_m==ROTATELEFT) {		
			leftSharp(1);
		}
		else if(rx_m==LEFTREVERSE){
			leftReverse(1);
		}
		else if(rx_m==RIGHTREVERSE){
				rightReverse(1);
		}
		
		else if(rx_m==SELFDRIVING) {
				if(flag){
					selfDriving();
					flag=0;
				}
		}		
		else if(rx_m==OFFMOTORS) {
			offMotors();
		}	
	}
}


void greenThread(void *argument) {
	uint8_t rx_g;	
	
	for(;;) {		
		osMessageQueueGet(greenQ,&rx_g,NULL,osWaitForever);
		if(rx_g==START){
			//everytime connected turn motor off
			greenLEDBluetoothOn();
			rx_data=OFFMOTORS;
		}
		else if(ENABLE_MASK(rx_g)){
				offGreen();
				movingGreen();
		}
		else{
			onGreen();	
		}
	}
}

void redThread(void *argument) {
	uint8_t rx_r;	
	
	for(;;) {
		
		osMessageQueueGet(redQ,&rx_r,NULL,osWaitForever);
		if(ENABLE_MASK(rx_r)){		
				movingRed();
		}
		else{
			stationaryRed();	
		}
	}	
}

void tAudio(void *argument) {
	
	uint8_t rx_a;
	
	int internalFlag = 1;
	
	for(;;){
		osMessageQueueGet(audioQ,&rx_a,NULL,osWaitForever);

		if(audioFlag){
			//1 second delay before song starts
			if(internalFlag) {
				osDelay(2000);
				internalFlag = 0;
			}
			maryHadALittleLamb();
		}	
	}
}

void tBrain(void *argument) {
	
  for(;;) {
		  osMessageQueuePut(motorQ, &rx_data, NULL, 0);
		  osMessageQueuePut(redQ, &rx_data, NULL, 0); 	
		  osMessageQueuePut(greenQ, &rx_data, NULL, 0); 
			osMessageQueuePut(audioQ, &rx_data, NULL, 0);	
		  osDelay(10);
	}			
}


int main (void) {
 
    SystemCoreClockUpdate();
	
	  initClockGate();
	  initUART2(BAUD_RATE);
	  initAudio();
	  initAudioPWM();
		initLED();
	  initMotor();
	  initMotorPWM();
	  initUltraSound();
		
    osKernelInitialize();   
	
		osThreadNew(tBrain, NULL, NULL);
	  brainQ = osMessageQueueNew(MSG_COUNT,sizeof(uint8_t),NULL);
	
	  osThreadNew(tAudio,NULL,NULL);
	  audioQ = osMessageQueueNew(MSG_COUNT,sizeof(uint8_t),NULL);
	
		osThreadNew(redThread, NULL, NULL);
		redQ = osMessageQueueNew(MSG_COUNT,sizeof(uint8_t),NULL);

		osThreadNew(greenThread, NULL, NULL);	
		greenQ = osMessageQueueNew(MSG_COUNT,sizeof(uint8_t),NULL);
				
		osThreadNew(tMotorControl, NULL, NULL);
	  motorQ = osMessageQueueNew(MSG_COUNT, sizeof(uint8_t), NULL);

  osKernelStart();      
  for (;;) {}
}
