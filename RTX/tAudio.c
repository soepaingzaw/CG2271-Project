#include "tAudio.h"

void initAudio(void) {
	PORTD->PCR[PTD0] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[PTD0] |= PORT_PCR_MUX(4);
// Set Data Direction Registers for  PortD
  PTD->PDDR |= (MASK(PTD0));
}

void initAudioPWM(void) {
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK|TPM_SC_CMOD_MASK|TPM_SC_PS_MASK);
	TPM0->SC |= (TPM_SC_CMOD(1)|TPM_SC_PS(7));
	

	TPM0_C0SC  &= ~(TPM_CnSC_ELSA_MASK |TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK |TPM_CnSC_MSB_MASK);	
	TPM0_C0SC |=(TPM_CnSC_ELSB(1)|TPM_CnSC_MSB(1));    
	
	TPM0->MOD = 0;
	TPM0_C0V = 0;	
}


void maryHadALittleLamb() {
	
	    int temp=375000;
	
	    int mary[30] = {330,294,262,294,330,330,330,  
											0,294,294,294,0,330,330,330,0,
											330,294,262,294,330,330,330,  
											0,294,294,330,294,262,0};	
	
			for(int i=0;i<NUM_NOTES;i++) {
	
			TPM0->MOD = temp/mary[i];
			TPM0_C0V = (temp/(mary[i]))/2;
			osDelay(100);
				
			TPM0->MOD = 0;
			TPM0_C0V = 0;		
			osDelay(100);

		}

}

void bluetoothConnected(void) {
			int temp =375000;
			int start[3] = {330,294,262};	
			
			for(int i=0;i<3;i++){
				TPM0->MOD = temp/start[i];
				TPM0_C0V = (temp/(start[i]))/2;
				delay(0x80000);
				TPM0->MOD = 0;
				TPM0_C0V = 0;		
				delay(0x80000);
			}	
}

void finishRun(void) {
	int temp = 375000;
	int finish[8] = {262,262,262,330,392,0,330,392};	
	uint32_t  time[8] = {0x80000,0x80000,0x80000,0x80000,0xC0000,0x80000,0x80000,0x180000};
	
	for(int i=0;i<8;i++){
			TPM0->MOD = temp/finish[i];
			TPM0_C0V = (temp/(finish[i]))/2;
			delay(time[i]);
			TPM0->MOD = 0;
			TPM0_C0V = 0;		
			delay(0x100000);
					
	}
	delay(0xFFFFFFFF);
	
}

