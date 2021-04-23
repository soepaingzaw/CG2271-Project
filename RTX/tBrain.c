#include "tBrain.h"

/*INITALISE ULTRASOUND COMPONENT*/
void initUltraSound(void){
	
	PORTC->PCR[TRIGPIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[TRIGPIN] |= PORT_PCR_MUX(1);
	PORTC->PCR[ECHOPIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[ECHOPIN] |= PORT_PCR_MUX(1);
	
	PTC->PDDR |= MASK(TRIGPIN);
	PTC->PDDR &= ~MASK(ECHOPIN);
}

/* to turn on trig pin */
void onTrigPin(void) {
	PTC->PSOR = MASK(TRIGPIN);
	
}

/* to turn off trig pin */
void offTrigPin(void) {
	PTC->PCOR = MASK(TRIGPIN);
	
}

// To appropriate delay to set trig pin to high
// in order to activate ultrasound sensor
void delay(volatile uint32_t nof) {
	while (nof!=0) {
		nof--;
	}	
}

/* tbrain commands robot to be in a STOP state*/
void stopState(osMessageQueueId_t motor,osMessageQueueId_t audio, osMessageQueueId_t red,osMessageQueueId_t green,int8_t data){
	 
	osMessageQueuePut(motor, &data, NULL, 0);
	osMessageQueuePut(red, &data, NULL, 0); 	
	osMessageQueuePut(green, &data, NULL, 0); 
	osMessageQueuePut(audio, &data, NULL, 0);	
	osDelay(DELAYFORSTOPSTATE);
}

/* tbrain commands robot to be in a ROTATELEFT state*/
void leftState(osMessageQueueId_t motor,osMessageQueueId_t audio, osMessageQueueId_t red,osMessageQueueId_t green,int8_t data){
	
	osMessageQueuePut(motor, &data, NULL, 0);
	osMessageQueuePut(red, &data, NULL, 0); 	
	osMessageQueuePut(green, &data, NULL, 0); 
	osMessageQueuePut(audio, &data, NULL, 0);			
	osDelay(DELAYFORSELFDRIVINGLEFT);
}

/* tbrain commands robot to be in a RIGHTCIRCLE state*/
void circleState(osMessageQueueId_t motor,osMessageQueueId_t audio, osMessageQueueId_t red,osMessageQueueId_t green,uint8_t data, uint8_t off){
	
	osMessageQueuePut(motor, &data, NULL, 0);
	osMessageQueuePut(audio, &data, NULL, 0);	

	for(int i=0;i<3;i++){
		osMessageQueuePut(red, &data, NULL, 0); 	
		osMessageQueuePut(green, &data, NULL, 0);
		osDelay(DELAYFORLEDPERIOD);
	}
	
	osMessageQueuePut(red, &off, NULL, 0); 	
	osMessageQueuePut(green,  &off, NULL, 0); 
	osDelay(DELAYFORREMAININGTIME);

}

/* tbrain commands robot to be in a FORWARD state*/
void forwardState(osMessageQueueId_t motor,osMessageQueueId_t audio, osMessageQueueId_t red,osMessageQueueId_t green,int8_t data){
	uint32_t count;
  double distance;
	double microseconds;
	
	while(1){
		
		count = 0;
		
		osMessageQueuePut(motor, &data, NULL, 0);
		osMessageQueuePut(red, &data, NULL, 0); 	
		osMessageQueuePut(green, &data, NULL, 0); 		
		osMessageQueuePut(audio, &data, NULL, 0);

		//set trigpin of HCSR04 Ultrasonic sensor to high
		onTrigPin();
		//delay for 10 microseconds before turning trig pin off
		delay(TENMICROSECONDS);	
		offTrigPin();
		
	  //wait for Echo pin of HCSR04 to be 1
		while(!(PTC->PDIR & MASK(ECHOPIN)));
		
		//while EchoPin is high, keep incrementing counter 
		while(PTC->PDIR & MASK(ECHOPIN)){
			count++;	
		}	
		//when Echopin returns to low again, stop incrementing		
		//count increments at 4.8Mhz=>4.8cycles/us ->24.0/5 
		microseconds = (count * 5)/24.0;
		
		//distance in cm => 2 * distance = time * (speed of sound)
		distance =  microseconds * 0.034 / 2;
		
		//calibrated stopping distance so that robot does not hit cone
		if(distance<15.5){
			break;
		}
	}	
}

