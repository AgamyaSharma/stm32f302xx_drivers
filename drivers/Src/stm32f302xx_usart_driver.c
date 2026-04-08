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

void USART_Innit(USART_Handle_t *pUSARTHandle){
	if(!IS_USART_MODE(pUSARTHandle->USART_Config.USART_Mode)){
		return;
	}
	if(!IS_USART_WL_BITS(pUSARTHandle->USART_Config.USART_WordLength)){
		return;
	}
	if(!IS_USART_CLK_MODE(pUSARTHandle->USART_Config.USART_ClkControl)){
		return;
	}
	if(!IS_USART_CPHA(pUSARTHandle->USART_Config.USART_CPHA)){
		return;
	}
	if(!IS_USART_CPOL(pUSARTHandle->USART_Config.USART_CPOL)){
		return;
	}
	if(!IS_USART_SP(pUSARTHandle->USART_Config.USART_StopBits)){
		return;
	}
	if(!IS_USART_PARITY(pUSARTHandle->USART_Config.USART_ParityControl)){
		return;
	}

	uint32_t temp;
	temp = pUSARTHandle->USART_Config.USART_WordLength;
	if(temp == USART_WL_7BIT){
		pUSARTHandle->pUSARTx->CR1 |= (1 << 28);
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << 12);

	}else if(temp == USART_WL_8BIT){
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << 28);
	    pUSARTHandle->pUSARTx->CR1 &= ~(1 << 12);
	}else if(temp == USART_WL_9BIT){
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << 28);
	    pUSARTHandle->pUSARTx->CR1 &= (1 << 12);
	}

	temp = pUSARTHandle->USART_Config.USART_BaudRate;
	pUSARTHandle->pUSARTx->BRR = temp;

	temp = pUSARTHandle->USART_Config.USART_ClkControl;
	if(temp == USART_CLK_ENABLE){
		if(pUSARTHandle->USART_Config.USART_CPHA == USART_CPHA_HIGH){
			pUSARTHandle->pUSARTx->CR2 |= (1 << 9);
		}else{
			pUSARTHandle->pUSARTx->CR2 &= ~(1 << 9);
		}

		if(pUSARTHandle->USART_Config.USART_CPOL == USART_CPOL_HIGH){
			pUSARTHandle->pUSARTx->CR2 |= (1 << 10);
		}else{
			pUSARTHandle->pUSARTx->CR2 &= ~(1 << 10);
		}
		pUSARTHandle->pUSARTx->CR2 |= (1 << 11);
	} else{
		pUSARTHandle->pUSARTx->CR2 &= ~(1 << 11);
	}

	temp = pUSARTHandle->USART_Config.USART_ParityControl;
	if(temp != RESET){
		pUSARTHandle->pUSARTx->CR1 |= (1 << 10);
		if(temp == USART_PARITY_EVEN){
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << 9);
		}else{
			pUSARTHandle->pUSARTx->CR1 |= (1 << 9);
		}
	}

	temp = pUSARTHandle->USART_Config.USART_StopBits;
	if(temp == USART_SP_1){
		pUSARTHandle->pUSARTx->CR2 &= ~(0x3 << 12);

	}else if(temp == USART_SP_0_5){
		pUSARTHandle->pUSARTx->CR2 &= ~(0x3 << 12);
		pUSARTHandle->pUSARTx->CR2 |= (1 << 12);
	}else if(temp == USART_SP_2){
		pUSARTHandle->pUSARTx->CR2 &= ~(0x3 << 12);
		pUSARTHandle->pUSARTx->CR2 |= (0x2 << 12);
	}else if(temp == USART_SP_1_5){
		pUSARTHandle->pUSARTx->CR2 &= ~(0x3 << 12);
		pUSARTHandle->pUSARTx->CR2 |= (0x3 << 12);
	}

	temp = pUSARTHandle->USART_Config.USART_HwFlowControl;
	if(temp == USART_HWFLOW_RTSE){
		pUSARTHandle->pUSARTx->CR3 &= ~(1 << 8);
		pUSARTHandle->pUSARTx->CR3 |= (1 << 8);
	}else if(temp == USART_HWFLOW_CTSE){
		pUSARTHandle->pUSARTx->CR3 &= ~(1 << 9);
		pUSARTHandle->pUSARTx->CR3 |= (1 << 9);
	}

	temp = pUSARTHandle->USART_Config.USART_OverSampling;
	if(temp == USART_OVER_8){
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << 15);
	}else if(pUSARTHandle->pUSARTx->CR1 == USART_OVER_16){
		pUSARTHandle->pUSARTx->CR1 |= (1 << 15);
	}

	temp = pUSARTHandle->USART_Config.USART_Mode;
	if(temp == USART_MODE_TX){
		pUSARTHandle->pUSARTx->CR1 |= (1 << 3);
	}else if(temp == USART_MODE_RX){
		pUSARTHandle->pUSARTx->CR1 |= (1 << 2);
	}else if(USART_MODE_DUPLEX){
		pUSARTHandle->pUSARTx->CR1 |= (1 << 2);
		pUSARTHandle->pUSARTx->CR1 |= (1 << 3);
	}
}


