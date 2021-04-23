#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.H"
#include "stdio.h"
#include "time.h"

#include "tMotor.h"
#include "tAudio.h"
#include "tLED.h"
#include "tBrain.h"

#define CLEARFLAGS 0xFFFFFFFF

#define MASK(x) (1 << (x))

#define UART_RX_PORTE23 23
#define BAUD_RATE 9600

osMessageQueueId_t audioQ,brainQ,redQ,greenQ,motorQ;
osThreadId_t t_Brain, t_Audio, red_Thread, green_Thread, t_MotorControl; 

//Disable all operations initially
uint8_t rx_data = OFF;

void initClockGate() {
	// Enable Clock Gating for all the different ports
	SIM->SCGC5 |= (SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTE_MASK) | (SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTD_MASK)|(SIM_SCGC5_PORTC_MASK);
	// Enable Clock Gating for UART2
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
		//Enable clock gating for Timer0, Timer 1 and Timer 2
	SIM->SCGC6 = (SIM_SCGC6_TPM0_MASK) | (SIM_SCGC6_TPM1_MASK) | (SIM_SCGC6_TPM2_MASK);
}

/* INITIALISE UART2 */
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

/*SERIAL_ISR COMPONENT, USES IRQ HANDLER FOR UART*/

void UART2_IRQHandler(void) {
	
	NVIC_ClearPendingIRQ(UART2_IRQn);

	if (UART2->S1 & UART_S1_RDRF_MASK) {
		rx_data = UART2->D;	
	} 
	PORTE->ISFR = CLEARFLAGS;
}

/* tMOTORCONTROL THREAD FOR MOVEMENT*/
void tMotorControl(void *argument) {
	
	uint8_t rx_m;	
	
	for(;;) {
		osMessageQueueGet(motorQ,&rx_m,NULL,osWaitForever);
		if(rx_m==FORWARD){
			fw();
		}
		else if(rx_m==REVERSE){
			rv();
		}
		else if(rx_m==LEFTFORWARD) {
			leftForward();		
		}
		else if(rx_m==RIGHTFORWARD) {	
			rightForward();
		}
		else if(rx_m==ROTATERIGHT) {	
			rightSharp();
		}
		else if(rx_m==ROTATELEFT) {		
			leftSharp();
		}
		else if(rx_m==LEFTREVERSE){
			leftReverse();
		}
		else if(rx_m==RIGHTREVERSE){
			rightReverse();
		}
		else if(rx_m==RIGHTCIRCLE) {
			rightCircle();		
		}		
		else if(rx_m==STOP||rx_m==ENDCHALLENGE) {
			offMotors();
		}	
	}
}

/* PART OF tLED CONTROLS GREEN LEDS*/
void greenThread(void *argument) {
	uint8_t rx_g;	
	
	for(;;) {		
		osMessageQueueGet(greenQ,&rx_g,NULL,osWaitForever);
		if(rx_g==CONNECT){
			greenLEDBluetoothOn();
			osDelay(WAITINGDELAY);
		}
		else if(MOVING_MASK(rx_g)){
			offGreen();
			movingGreen();
		}
		else{
			onGreen();	
		}
	}
}

/* PART OF tLED CONTROLS RED LEDS*/
void redThread(void *argument) {
	uint8_t rx_r;	
	
	for(;;) {
		
		osMessageQueueGet(redQ,&rx_r,NULL,osWaitForever);
		if(MOVING_MASK(rx_r)){		
				movingRed();
		}
		else{
			stationaryRed();
		}
	}	
}

/* AUDIO THREAD*/
void tAudio(void *argument) {
	
	uint8_t rx_a;
	
	for(;;){
		osMessageQueueGet(audioQ,&rx_a,NULL,osWaitForever);
		if(rx_a == CONNECT){		
			bluetoothConnected();
		  osDelay(INITIALAUDIODELAY);
		}
		
		else if(rx_a == ENDCHALLENGE){
			finishRun(&rx_data);
		}
		
		else {
		  maryHadALittleLamb(&rx_data);//Passes address of rx_data for early termination
	  }

	}
}

/* MAIN CONTROL THREAD */
void tBrain(void *argument) {
	
  for(;;) {

		if(rx_data==SELFDRIVING){
			forwardState(motorQ,audioQ,redQ,greenQ,FORWARD);			
			stopState(motorQ,audioQ,redQ,greenQ,STOP);	
			leftState(motorQ,audioQ,redQ,greenQ,ROTATELEFT);				
			stopState(motorQ,audioQ,redQ,greenQ,STOP);				
			circleState(motorQ,audioQ,redQ,greenQ,RIGHTCIRCLE,STOP);		
			stopState(motorQ,audioQ,redQ,greenQ,STOP);			
			leftState(motorQ,audioQ,redQ,greenQ,ROTATELEFT);		
			stopState(motorQ,audioQ,redQ,greenQ,STOP);						
      forwardState(motorQ,audioQ,redQ,greenQ,FORWARD);				
			stopState(motorQ,audioQ,redQ,greenQ,STOP);			
		}
		else if(rx_data==CONNECT){
			osMessageQueuePut(greenQ, &rx_data, NULL, 0); 
			osMessageQueuePut(audioQ, &rx_data, NULL, 0);	
		  osMessageQueuePut(redQ, &rx_data, NULL, 0); 	
			osDelay(DELAYFORSTATIONARYRED);
		  osMessageQueuePut(redQ, &rx_data, NULL, 0); 	
			osDelay(DELAYFORSTATIONARYRED);		  
		  osMessageQueuePut(redQ, &rx_data, NULL, 0); 	
			osDelay(DELAYFORSTATIONARYRED);
		  osMessageQueuePut(redQ, &rx_data, NULL, 0); 	
			osDelay(DELAYFORSTATIONARYRED);		
			rx_data = STOP;			
			
		}
		else if(rx_data!=OFF){
			osMessageQueuePut(motorQ, &rx_data, NULL, 0);
		  osMessageQueuePut(redQ, &rx_data, NULL, 0); 	
		  osMessageQueuePut(greenQ, &rx_data, NULL, 0); 
			osMessageQueuePut(audioQ, &rx_data, NULL, 0);	
		}
	}			
}

/* MAIN FUNCTION */
int main (void) {
		//Initialisation of all sub systems
    SystemCoreClockUpdate();
		
	  initClockGate();
	  initUART2(BAUD_RATE);
	  initAudio();
	  initAudioPWM();
		initLED();
	  initMotor();
	  initMotorPWM();
	  initUltraSound();
		
		//Initialisation of all threads and message queues
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

