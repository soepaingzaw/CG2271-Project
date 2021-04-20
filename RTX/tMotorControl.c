#include "tMotor.h"


void initMotorPWM(void) {
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;	
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK|TPM_SC_CMOD_MASK|TPM_SC_PS_MASK);
	TPM1->SC |= (TPM_SC_CMOD(1)|TPM_SC_PS(7));
	
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK|TPM_SC_CMOD_MASK|TPM_SC_PS_MASK);
	TPM2->SC |= (TPM_SC_CMOD(1)|TPM_SC_PS(7));

	
	TPM1_C1SC  &= ~(TPM_CnSC_ELSA_MASK |TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK |TPM_CnSC_MSB_MASK);
	TPM1_C0SC  &= ~(TPM_CnSC_ELSA_MASK |TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK |TPM_CnSC_MSB_MASK);
	
	TPM2_C1SC  &= ~(TPM_CnSC_ELSA_MASK |TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK |TPM_CnSC_MSB_MASK);
	TPM2_C0SC  &= ~(TPM_CnSC_ELSA_MASK |TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK |TPM_CnSC_MSB_MASK);
	
	
	
	TPM1_C1SC |=(TPM_CnSC_ELSB(1)|TPM_CnSC_MSB(1));    
	TPM1_C0SC |=(TPM_CnSC_ELSB(1)|TPM_CnSC_MSB(1)); //added line
	
	TPM2_C1SC |=(TPM_CnSC_ELSB(1)|TPM_CnSC_MSB(1));    
	TPM2_C0SC |=(TPM_CnSC_ELSB(1)|TPM_CnSC_MSB(1)); //added line
	
}

//set modulo value 48000000/128 = 375000, 375000Hz/100Hz = 3750	
void initMotor(void) {
	
	//initilaize motors
  PORTB->PCR[PTB1] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB1] |= PORT_PCR_MUX(3);//TPM1CH1
	PORTB->PCR[PTB0] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB0] |= PORT_PCR_MUX(3);//TPM1CH0
	
	PORTB->PCR[PTB3] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB3] |= PORT_PCR_MUX(3);//TPM2CH1
	PORTB->PCR[PTB2] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB2] |= PORT_PCR_MUX(3);//TPM2CH0
	
  PTB->PDDR |= (MASK(PTB1));
	PTB->PDDR |= (MASK(PTB0));
	
	PTB->PDDR |= (MASK(PTB3));
	PTB->PDDR |= (MASK(PTB2));

}



void fw(int ms) {
	TPM1->MOD = TPMMOD;
	TPM2->MOD = TPMMOD;
	

	TPM1_C0V = TPMMOD;
	TPM2_C1V = TPMMOD;
	
	TPM2_C0V = 0;
	TPM1_C1V = 0;
	
	osDelay(ms);

}


void rv(int ms) {


	TPM1->MOD = TPMMOD;
	TPM2->MOD = TPMMOD;
	
	TPM1_C1V = TPMMOD;
	TPM2_C0V = TPMMOD;
	
	TPM1_C0V = 0;
	TPM2_C1V = 0;


  osDelay(ms);
	
}


void leftForward(int ms) {
	TPM1->MOD = TPMMOD;
	TPM2->MOD = TPMMOD;
		
	TPM1_C1V = 0;
	TPM2_C0V = 0;
	
	TPM1_C0V = TPMMOD;
	TPM2_C1V = 0;

	osDelay(ms);

	
}

void rightForward(int ms) {
	TPM1->MOD = TPMMOD;		
	TPM2->MOD = TPMMOD;
	
	TPM1_C1V = 0;
	TPM2_C0V = 0;
	
	TPM1_C0V = 0;
	TPM2_C1V = TPMMOD;

	osDelay(ms);

}

void leftReverse(int ms){
	TPM1->MOD = TPMMOD;
	TPM2->MOD = TPMMOD;
		
	TPM1_C1V = TPMMOD;
	TPM2_C0V = 0;
	
	TPM1_C0V = 0;
	TPM2_C1V = 0;

	osDelay(ms);
	
}

void rightReverse(int ms){
	
	TPM1->MOD = TPMMOD;		
	TPM2->MOD = TPMMOD;
	
	TPM1_C1V = 0;
	TPM2_C0V = TPMMOD;
	
	TPM1_C0V = 0;
	TPM2_C1V = 0;

	osDelay(ms);
	
}


void offMotors(void) {
	TPM1->MOD = 0;
	TPM2->MOD = 0;
	
	
	TPM1_C0V = 0;
	TPM1_C1V = 0;
	TPM2_C0V = 0;
	TPM2_C1V = 0;
}

void leftSharp(int ms){
		TPM1->MOD = TPMMOD;		
	  TPM2->MOD = TPMMOD;

		TPM1_C1V = 0;
		TPM2_C0V = TPMMOD;
		
		TPM1_C0V = TPMMOD;
		TPM2_C1V = 0;

		osDelay(ms);

}

void rightSharp(int ms) {
	TPM1->MOD = TPMMOD;
	TPM2->MOD = TPMMOD;
		
	TPM1_C1V = TPMMOD;
	TPM2_C0V = 0;
	
	TPM1_C0V = 0;
	TPM2_C1V = TPMMOD;
	
	osDelay(ms);

}

void rightCircle(int ms) {
	TPM1->MOD = TPMMOD;		
	TPM2->MOD = TPMMOD;
	
	TPM1_C1V = TPMMOD/2;
	TPM2_C0V = 0;
	
	TPM1_C0V = TPMMOD;
	TPM2_C1V = TPMMOD;

	osDelay(ms);

}



void initUltraSound(void){
	
	PORTC->PCR[TRIGPIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[TRIGPIN] |= PORT_PCR_MUX(1);
	PORTC->PCR[ECHOPIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[ECHOPIN] |= PORT_PCR_MUX(1);
	
	PTC->PDDR |= MASK(TRIGPIN);
	PTC->PDDR &= ~MASK(ECHOPIN);
}

void onTrigPin(void) {
	PTC->PSOR = MASK(TRIGPIN);
	
}

void offTrigPin(void) {
	PTC->PCOR = MASK(TRIGPIN);
	
}

void delay(volatile uint32_t nof) {
		while (nof!=0) {
			nof--;
		}	
}

uint32_t count=0;
double distance=0;

void sensor(void){

	double microseconds;
	
	
	while(1){
		
		count = 0;

		//set trigpin of HCSR04 to high
		onTrigPin();
		//delay for 10 microseconds before turning trig pin off
		delay(0x30);	
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
		
		//distance in cm
		distance =  microseconds * 0.034 / 2;
		
		//calibrated stopping distance so that robot does not hit cone
		if(distance<15.5){
			offMotors();
			osDelay(500);
			break;
		}
	}	
}

void selfDriving(void){
		
	  fw(1);
		sensor();
	

		leftSharp(SELFDRIVINGLEFT);
	
		offMotors();
		osDelay(1000);
	
	  //Loop around cone the time taken is calibrated
	  rightCircle(3750);
	
		offMotors();
		osDelay(1000);
		
		leftSharp(375);
		offMotors();
		osDelay(1000);
		
		fw(1);
		sensor();	
}






