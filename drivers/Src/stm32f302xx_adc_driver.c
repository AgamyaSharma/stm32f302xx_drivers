
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
	if(pADCHandle->ADC_CONFIG.ADC_Calibration == ADC_CAL_DIF){
		pADCHandle->pADCx->CR &= ~(1 << 30);
		pADCHandle->pADCx->CR |= (1 << 30);
	}else{
		pADCHandle->pADCx->CR &= ~(1 << 30);
	}

	pADCHandle->pADCx->CR |= (1 << 31);
	while(!(pADCHandle->pADCx->ISR & 1));

	if(pADCHandle->ADC_CONFIG.ADC_DataAlign == ADC_DATA_LEFT_ALIGN){
		pADCHandle->pADCx->CFGR |= (1 << 5);
	}else{
		pADCHandle->pADCx->CFGR &= ~(1 << 5);
	}

	if(pADCHandle->ADC_CONFIG.ADC_DataRes == ADC_DATA_RES_12BIT	){
		pADCHandle->pADCx->CFGR &= ~(0x3 << 3);
	}else if(pADCHandle->ADC_CONFIG.ADC_DataRes == ADC_DATA_RES_10BIT){
		pADCHandle->pADCx->CFGR &= ~(0x3 << 3);
		pADCHandle->pADCx->CFGR |= (1 << 3);
	}else if(pADCHandle->ADC_CONFIG.ADC_DataRes == ADC_DATA_RES_8BIT){
		pADCHandle->pADCx->CFGR &= ~(0x3 << 3);
		pADCHandle->pADCx->CFGR |= (2 << 3);
	}else if(pADCHandle->ADC_CONFIG.ADC_DataRes == ADC_DATA_RES_6BIT){
		pADCHandle->pADCx->CFGR &= ~(0x3 << 3);
		pADCHandle->pADCx->CFGR |= (3 << 3);
	}

	if(pADCHandle->ADC_CONFIG.ADC_OVRhandle == ADC_OVRMODE_ENABLE){
		pADCHandle->pADCx->CFGR &= ~(1 << 12);
		pADCHandle->pADCx->CFGR |= (1 << 12);
	}else{
		pADCHandle->pADCx->CFGR &= ~(1 << 12);
	}


}
void ADC_Deinnit(ADC_Handle_t *pADCHandle){
	 ADC_REG_RESET();
	 ADC_PCLK_DISABLE();
}
