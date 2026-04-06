#include <stdint.h>
#include "stm32f302xx.h"
#include "stm32f302xx_usart_driver.h"

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_ENABLE();
		}else if(pUSARTx == USART2){
			USART2_PCLK_ENABLE();
		}else if(pUSARTx == USART3){
			USART3_PCLK_ENABLE();

		}
	}else if(EnorDi == DISABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_DISABLE();
		}else if(pUSARTx == USART2){
			USART2_PCLK_DISABLE();
		}else if(pUSARTx == USART3){
			USART3_PCLK_DISABLE();
		}
	}
}

void USART_Deinnit(USART_RegDef_t *pUSARTx){
	if(pUSARTx == USART1){
		USART1_REG_RESET();
		USART1_PCLK_DISABLE();
	}else if(pUSARTx == USART2){
		USART2_REG_RESET();
		USART2_PCLK_DISABLE();
	}else if(pUSARTx == USART3){
		USART3_REG_RESET();
		USART3_PCLK_DISABLE();
	}

}

void USART_Innit(USART_RegDef_t *pUSARTHandle){

}
