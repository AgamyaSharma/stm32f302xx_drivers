
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
	pADCHandle->pADCx->CR &= ~(0xB << 28);
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

	pADCHandle->pADCx->CR |= (1 << 30);
	while(!(pADCHandle->pADCx->ISR & 1));

}
void ADC_Deinnit(ADC_Handle_t *pADCHandle){
	 ADC_REG_RESET();
	 ADC_PCLK_DISABLE();
}
