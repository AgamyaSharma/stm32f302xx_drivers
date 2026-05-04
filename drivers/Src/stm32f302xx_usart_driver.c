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
	    pUSARTHandle->pUSARTx->CR1 |= (1 << 12);
	}

	temp = pUSARTHandle->USART_Config.USART_BaudRate;

	pUSARTHandle->pUSARTx->BRR = ((8000000 + (temp/2)) / temp);   // TODO: A COMPLETE FUNCTION FOR GETTING THE CLOCK INFO FROM RCC ALSO LOOK INTO AN RCC DRIVER

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
		pUSARTHandle->pUSARTx->CR1 |= (1 << 15);
	}else if(temp == USART_OVER_16){
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << 15);
	}

	temp = pUSARTHandle->USART_Config.USART_Mode;
	if(temp == USART_MODE_TX){
		pUSARTHandle->pUSARTx->CR1 |= (1 << 3);
	}else if(temp == USART_MODE_RX){
		pUSARTHandle->pUSARTx->CR1 |= (1 << 2);
	}else if(temp == USART_MODE_DUPLEX){
		pUSARTHandle->pUSARTx->CR1 |= (1 << 2);
		pUSARTHandle->pUSARTx->CR1 |= (1 << 3);
	}

	pUSARTHandle->pUSARTx->CR1 |= (1);
}

uint8_t USART_GetStatusFlag(USART_RegDef_t*pUSARTx, uint8_t FlagName){

	 if(pUSARTx->ISR & FlagName){
		 return FLAG_SET;
	 }else{
		 return FLAG_RESET;
	 }
}
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		while(!(USART_GetStatusFlag(pUSARTHandle->pUSARTx, USART_TXE_FLAG)));
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WL_7BIT){
			(*((uint8_t*)&pUSARTHandle->pUSARTx->TDR)) = (*((uint8_t*)pTxBuffer) & (0x7F) ) ;
			pTxBuffer = (((uint8_t*)pTxBuffer) + 1);
			Len--;
		}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WL_8BIT){
			(*((uint8_t*)&pUSARTHandle->pUSARTx->TDR)) = (*((uint8_t*)pTxBuffer)) ;
			pTxBuffer = (((uint8_t*)pTxBuffer) + 1);
			Len--;
		}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WL_9BIT){
			(*((uint16_t*)&pUSARTHandle->pUSARTx->TDR)) = ((*((uint16_t*)pTxBuffer) & (0x1FF))) ;
			pTxBuffer = (((uint16_t*) pTxBuffer) + 1);
			Len--;
			Len--;
	}
  }
}

void USART_RecieveveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0){
			while(!(USART_GetStatusFlag(pUSARTHandle->pUSARTx, USART_RXE_FLAG)));
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WL_7BIT){
				(*((uint8_t*)pRxBuffer)  ) = (*((uint8_t*)&pUSARTHandle->pUSARTx->RDR)& (0x7F)) ;
				pRxBuffer = (((uint8_t*)pRxBuffer) + 1);
				Len--;
			}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WL_8BIT){
				 (*((uint8_t*)pRxBuffer)) = (*((uint8_t*)&pUSARTHandle->pUSARTx->RDR));
				pRxBuffer = (((uint8_t*)pRxBuffer) + 1);
				Len--;
			}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WL_9BIT){
				  (*((uint16_t*)pRxBuffer)) = ((*((uint16_t*)&pUSARTHandle->pUSARTx->RDR) & (0x1FF))) ;
				  pRxBuffer = (((uint16_t*)pRxBuffer) + 1);
				Len--;
				Len--;
		}
	  }
}

void USART_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi== ENABLE){
		if(IRQNumber <=31){

			NVIC->ISER[0] |= (1<< IRQNumber);

		}else if((IRQNumber > 31) && (IRQNumber <=63)){

			NVIC->ISER[1] |= (1<< IRQNumber % 32);

		}else if((IRQNumber > 63) && (IRQNumber <=95)){

			NVIC->ISER[2] |= (1<< (IRQNumber % 64));

		}

	}else {
		if(IRQNumber <=31){

			NVIC->ICER[0] |= (1<<IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber <=63){

			NVIC->ICER[1] |= (1<<(IRQNumber % 32));
		}else if(IRQNumber > 63 && IRQNumber <=95){

			NVIC->ICER[2] |= (1<<(IRQNumber % 64));

		}

	}

}

void USART_PriorityConfig(uint8_t IRQPriority,uint8_t IRQNumber){
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprxSection = (IRQNumber % 4);
	uint8_t shiftAmount =((8*iprxSection) + 4);
	NVIC->IPR[iprx] |= (IRQPriority << (shiftAmount));
}

uint8_t USART_Buffer_Push(USART_Buffer_t *pBuffer, uint8_t *tempData){
	uint32_t NextHead = ((pBuffer->Head +1) & BUFFER_MASK);

	if(NextHead == (pBuffer->Tail)){
		pBuffer->Tail = ((pBuffer->Tail +1) & BUFFER_MASK);
		pBuffer->Buffer[pBuffer->Head] = *tempData;
		pBuffer->Head = NextHead;


	}else{
		pBuffer->Buffer[pBuffer->Head] = *tempData;
		pBuffer->Head = NextHead;
	}
	return 1;
}

uint8_t USART_Buffer_Pop(USART_Buffer_t *pBuffer, uint8_t *pdata){
	if((pBuffer->Head) == (pBuffer->Tail)){
		return 0; //BUFFER EMPTY
	}
	*pdata = pBuffer->Buffer[pBuffer->Tail];
	pBuffer->Tail = ((pBuffer->Tail +1) & BUFFER_MASK);
	return 1;
}

void USART_SendDataIt(USART_Handle_t *pUSARTHandle, uint8_t *pDataBuffer, uint32_t Len){
	uint8_t state= pUSARTHandle->TxState;
	if(state != USART_BUSY_IN_TX){
		pUSARTHandle->TxState = USART_BUSY_IN_TX;
		for(int i=0;i<=Len-1;i++){
			uint8_t data = pDataBuffer[i];
			USART_Buffer_Push( &(pUSARTHandle->TxBuffer), &data);
		}
	pUSARTHandle->pUSARTx->CR1 |= (1 << 7);

	}
}


void USART_RecieveDataIt(USART_Handle_t *pUSARTHandle){
	uint8_t state = pUSARTHandle->RxState;
	if(state != USART_BUSY_IN_RX){
		pUSARTHandle->RxState = USART_BUSY_IN_RX;
		pUSARTHandle->pUSARTx->CR1 |= (1 << 5);
	}else{
		return;
	}
}

void USART_IRQHandle(USART_Handle_t *pUSARTHandle ){
	uint8_t temp1;
	uint8_t temp2;
	temp1 = pUSARTHandle->pUSARTx->CR1 & (1 << 7);
	temp2 = pUSARTHandle->pUSARTx->ISR & (1 << 7);
	if (temp1 && temp2){
		usart_txe_interrupt_handle(pUSARTHandle);
	}

	temp1 = pUSARTHandle->pUSARTx->CR1 & (1 << 5);
	temp2 = pUSARTHandle->pUSARTx->ISR & (1 << 5);
	if(temp1 && temp2){
		usart_rxne_interrupt_handle(pUSARTHandle);
	}
}

static void usart_txe_interrupt_handle(USART_Handle_t *pUSARTHandle){
	uint8_t temp1;
	uint8_t status = USART_Buffer_Pop( &(pUSARTHandle->TxBuffer), &temp1);
	if(status){
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WL_7BIT){
		(*((uint8_t*)&pUSARTHandle->pUSARTx->TDR)) = ((temp1) & (0x7F));

		}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WL_8BIT){
		(*((uint8_t*)&pUSARTHandle->pUSARTx->TDR)) = (temp1);
		}
	}else{
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << 7);
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
	}
}

static void usart_rxne_interrupt_handle(USART_Handle_t *pUSARTHandle)
{
	uint8_t temp1;
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WL_7BIT){
			temp1= (*((uint8_t*)&pUSARTHandle->pUSARTx->RDR)& (0x7F));
			USART_Buffer_Push(&(pUSARTHandle->RxBuffer), &temp1);
	}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WL_8BIT){
		    temp1 = (*((uint8_t*)&pUSARTHandle->pUSARTx->RDR));
		    USART_Buffer_Push(&(pUSARTHandle->RxBuffer), &temp1);
	}
}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t APPEv){

}
