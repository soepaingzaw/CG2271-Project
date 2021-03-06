#include "MKL25Z4.H"
#include "stdio.h"
#include "time.h"

#define MASK(x) (1 << (x))
//Below is TPM1 CH1
#define PTD0 0
#define NUM_NOTES 30
/*
#define FREQ_CHANGE(x) (375000/x);
(((uint32_t)(((uint32_t)(x))<<TPM_SC_PS_SHIFT))&TPM_SC_PS_MASK)
*/	 
							 

void InitGPIO(void)
{
// Enable Clock to PORTB and PORTD
SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
// Configure MUX settings to make all 3 pins GPIO
PORTD->PCR[PTD0] &= ~PORT_PCR_MUX_MASK;
PORTD->PCR[PTD0] |= PORT_PCR_MUX(4);

// Set Data Direction Registers for  PortD
PTD->PDDR |= (MASK(PTD0));
}

void InitPWM(void) {
	
	SIM->SCGC6 |= (SIM_SCGC6_TPM0_MASK);
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK|TPM_SC_CMOD_MASK|TPM_SC_PS_MASK);
	TPM0->SC |= (TPM_SC_CMOD(1)|TPM_SC_PS(7));
	

	
	TPM0_C0SC  &= ~(TPM_CnSC_ELSA_MASK |TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK |TPM_CnSC_MSB_MASK);
	
	TPM0_C0SC |=(TPM_CnSC_ELSB(1)|TPM_CnSC_MSB(1));    
	
}


void delay(volatile uint32_t nof) {
		while (nof!=0) {
	
			nof--;
		}
		//delay(0x200000);//1 second delay
}

void maryHadALittleLamb(void) {
	
	    int temp=375000;
	
	    int mary[30] = {330,294,262,294,330,330,330,  
											0,294,294,294,0,330,392,392,0,
											330,294,262,294,330,330,330,  
											0,294,294,330,294,262,0};	
	
			for(int i=0;i<NUM_NOTES;i++) {
	
			TPM0->MOD = temp/mary[i];
			TPM0_C0V = (temp/(mary[i]))/2;
			delay(0x100000);
			TPM0->MOD = 0;
			TPM0_C0V = 0;		
			delay(0x100000);

		}

}


int main (void) {
	SystemCoreClockUpdate();
	InitGPIO();

	InitPWM();
	
	while(1){

		maryHadALittleLamb();

	}
	
}