#include "tAudio.h"

/* INITIALISE AUDIO COMPONENTS */
void initAudio(void) {
	PORTD->PCR[PTD0] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[PTD0] |= PORT_PCR_MUX(4);

  PTD->PDDR |= (MASK(PTD0));
}

/* INITIALISE COUNTERS FOR AUDIO COMPONENTS */
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

/* NURSERY RHYME MARY HAD A LITTLE LAMB */
void maryHadALittleLamb(uint8_t *rx_a) {
	
	int mary[NUM_NOTES_SONG] = {NOTE_E,NOTE_D,NOTE_C,NOTE_D,NOTE_E,NOTE_E,NOTE_E,  
										NOTE_REST,NOTE_D,NOTE_D,NOTE_D,NOTE_REST,NOTE_E,NOTE_E,NOTE_E,NOTE_REST,
										NOTE_E,NOTE_D,NOTE_C,NOTE_D,NOTE_E,NOTE_E,NOTE_E,  
										NOTE_REST,NOTE_D,NOTE_D,NOTE_E,NOTE_D,NOTE_C,NOTE_REST};	

	for(int i=0;i<NUM_NOTES_SONG&&(*rx_a)!=END;i++) {

		TPM0->MOD = COUNTERFREQUENCY/mary[i];
		TPM0_C0V = (COUNTERFREQUENCY/(mary[i]))/2;
		osDelay(SONGDELAY);
			
		TPM0->MOD = 0;
		TPM0_C0V = 0;		
		osDelay(SONGDELAY);
	}
}


//Plays Mi Re Do (Do Re Mi in reverse) tune 
//upon successful connection with bluetooth
void bluetoothConnected(void) {

	int start[NUM_NOTES_START] = {NOTE_E,NOTE_D,NOTE_C};	
	
	for(int i=0;i<NUM_NOTES_START;i++){
		TPM0->MOD = COUNTERFREQUENCY/start[i];
		TPM0_C0V = (COUNTERFREQUENCY/(start[i]))/2;
		osDelay(STARTDELAY);
		TPM0->MOD = 0;
		TPM0_C0V = 0;		
		osDelay(STARTDELAY);
	}	
}

/*Plays an old school Nintendo tune when the player finishes a challenge */
void finishRun(uint8_t *rx_a) {

	int finish[NUM_NOTES_END] = {NOTE_C,NOTE_C,NOTE_C,NOTE_E,NOTE_G,NOTE_REST,NOTE_E,NOTE_G};	
	uint32_t  time[NUM_NOTES_END] = {SHORTPAUSE,SHORTPAUSE,SHORTPAUSE,SHORTPAUSE,MEDPAUSE ,SHORTPAUSE,SHORTPAUSE,LONGPAUSE};
	
	for(int i=0;i<NUM_NOTES_END && (*rx_a)==END;i++){
		TPM0->MOD = COUNTERFREQUENCY/finish[i];
		TPM0_C0V = (COUNTERFREQUENCY/(finish[i]))/2;
		osDelay(time[i]);
		TPM0->MOD = 0;
		TPM0_C0V = 0;		
		osDelay(ENDDELAY);		
	}
}

