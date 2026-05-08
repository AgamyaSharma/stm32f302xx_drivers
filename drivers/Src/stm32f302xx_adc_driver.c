
#include "stm32f302xx.h"
#include "stm32f302xx_adc_driver.h"


void ADC_PeriCloclControl(ADC_RegDef_t *pADCx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		ADC_PCLK_ENABLE();
	}else{
		ADC_PCLK_DISABLE();
	}
}

void ADC_Innit(ADC_Handle_t *pADCHandle){
	pADCHandle->pADCx->CR &= ~(0x3 << 28);
	pADCHandle->pADCx->CR |= (1 << 28);
	for(volatile int i=1; i<=1000;i++){
	    	__asm("nop");

	}
	pADCHandle->pADCx->CR &= ~(1 << 30);
	switch(pADCHandle->ADC_CONFIG.ADC_Calibration){
		case ADC_CAL_DIF:
			pADCHandle->pADCx->CR |= (1 << 30);
			break;
		case ADC_CAL_SINGLE:
			break;
	}

	pADCHandle->pADCx->CR |= (1 << 31);
	while(!(pADCHandle->pADCx->ISR & 1));

	pADCHandle->pADCx->CFGR &= ~(1 << 5);
	switch(pADCHandle->ADC_CONFIG.ADC_DataAlign){
		case ADC_DATA_LEFT_ALIGN:
			pADCHandle->pADCx->CFGR |= (1 << 5);
			break;
		case ADC_DATA_RIGHT_ALIGN:
			break;
	}

	pADCHandle->pADCx->CFGR &= ~(0x3 << 3);
	switch(pADCHandle->ADC_CONFIG.ADC_DataRes){
		case ADC_DATA_RES_12BIT:
			break;
		case ADC_DATA_RES_10BIT:
			pADCHandle->pADCx->CFGR |= (1 << 3);
			break;
		case ADC_DATA_RES_8BIT:
			pADCHandle->pADCx->CFGR |= (2 << 3);
			break;
		case ADC_DATA_RES_6BIT:
			pADCHandle->pADCx->CFGR |= (3 << 3);
			break;
	}

	pADCHandle->pADCx->CFGR &= ~(1 << 12);
	switch(pADCHandle->ADC_CONFIG.ADC_OVRhandle){
		case ADC_OVRMODE_ENABLE:
			pADCHandle->pADCx->CFGR |= (1 << 12);
			break;
		case ADC_OVRMODE_DISABLE:
			break;
	}

	pADCHandle->pADCx->CFGR &= ~((1 << 16) | (1 << 13));
	switch (pADCHandle->ADC_CONFIG.ADC_ConversionMode){
		case ADC_MODE_SINGLE_CONVERSION:
			break;
		case ADC_MODE_CONT_CONVERSION:
			pADCHandle->pADCx->CFGR |= (1 << 13);
			break;
		case ADC_MODE_DISCON_CONVERSION:
				pADCHandle->pADCx->CFGR |= (1 << 16);
				if(pADCHandle->ADC_CONFIG.Discon_ChunkSize <= 8){
					pADCHandle->pADCx->CFGR |= (pADCHandle->ADC_CONFIG.Discon_ChunkSize << 17);
				}
			break;
	}

	pADCHandle->pADCx->SQR[0] &= ~(0xF);
	pADCHandle->pADCx->SQR[0] |= ((pADCHandle->ADC_CONFIG.SequenceLen) - 1);

	for(int i = 0; i <= ((pADCHandle->ADC_CONFIG.SequenceLen) - 1); i ++){

		uint8_t temp1;
		uint8_t temp2;
		uint8_t currentChannel = pADCHandle->ADC_CONFIG.pSequence[i].Channel;
		uint8_t currentSampleTime = pADCHandle->ADC_CONFIG.pSequence[i].SampleTime;


			if(currentChannel <= 9){
				temp1 = (currentChannel*3);
				pADCHandle->pADCx->SMPR[0] &= ~(0x7 << temp1);
				pADCHandle->pADCx->SMPR[0] |= (currentSampleTime << temp1);
			}else if(currentChannel > 9){
				temp1 = (currentChannel - 10)*3;
				pADCHandle->pADCx->SMPR[0] &= ~(0x7 << temp1);
				pADCHandle->pADCx->SMPR[0] |= (currentSampleTime << temp1);
			}


		temp1 = (i + 1)/5;
		temp2 = (((i + 1) % 5)*6);
		pADCHandle->pADCx->SQR[temp1] |= temp2;
	}

	pADCHandle->pADCx->CFGR |= (pADCHandle ->ADC_CONFIG.ADC_ExtTrig_Edge << 10);
	pADCHandle->pADCx->CFGR |= (pADCHandle ->ADC_CONFIG.ADC_ExtTrig_Source << 6);
}
void ADC_Deinnit(ADC_Handle_t *pADCHandle){
	 ADC_REG_RESET();
	 ADC_PCLK_DISABLE();
}
