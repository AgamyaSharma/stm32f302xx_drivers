#include <stdio.h>
#include <stdint.h>
#include "stm32f302xx.h"
#include "stm32f302xx_dma_driver.h"

void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		RCC->AHBENR |= (1 << 0);
	}else if(EnOrDi == DISABLE){
		RCC->AHBENR &= ~(1 << 0);
	}

}

void DMA_StartTransfer(DMA_RegDef_t *pDMAx ,uint8_t Channelx){
	pDMAx->CH_REG[(Channelx - 1)].CCR |= (1 << 0);
}

void DMA_StopTransfer(DMA_RegDef_t *pDMAx ,uint8_t Channelx){
	pDMAx->CH_REG[(Channelx - 1)].CCR &= ~(1 << 0);
}

void DMA_Innit(DMA_Handle_t *pDMAHandle){
	uint8_t ChannelNo = pDMAHandle->DMA_Config.DMA_Channel;
	pDMAHandle->pDMAx->CH_REG[ChannelNo- 1].CPAR = (0x00000000);
	pDMAHandle->pDMAx->CH_REG[ChannelNo- 1].CPAR |= pDMAHandle->DMA_Config.DMA_PeriAddr;

	pDMAHandle->pDMAx->CH_REG[ChannelNo- 1].CMAR = (0x00000000);
	pDMAHandle->pDMAx->CH_REG[ChannelNo- 1].CMAR |= pDMAHandle->DMA_Config.DMA_MemAddr;

	pDMAHandle->pDMAx->CH_REG[ChannelNo- 1].CNDTR &= ~(0x00007FFF);
	pDMAHandle->pDMAx->CH_REG[ChannelNo- 1].CNDTR |= pDMAHandle->DMA_Config.DMA_SequenceLen;

	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR &= ~(1 << 7);
	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR |= (pDMAHandle->DMA_Config.DMA_MINC << 7);

	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR &= ~(1 << 6);
	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR |= (pDMAHandle->DMA_Config.DMA_PINC << 6);

	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR &= ~(1 << 5);
	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR |= (pDMAHandle->DMA_Config.DMA_CIRC << 5);

	uint8_t temp2 = (pDMAHandle->DMA_Config.DMA_TransferMode);
	if((temp2 == DMA_MODE_MEM2PERI) ||(temp2 == DMA_MODE_PERI2MEM)){

		pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR &= ~(1 << 4);
		pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR |= (pDMAHandle->DMA_Config.DMA_TransferMode << 4);

	}else if((temp2 == DMA_MODE_MEM2MEM)){
		pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR |= (pDMAHandle->DMA_Config.DMA_TransferMode << 14);
	}

	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR &= ~(11 << 8);
	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR |= (pDMAHandle->DMA_Config.DMA_Psize << 8);

	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR &= ~(11 << 10);
	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR |= (pDMAHandle->DMA_Config.DMA_Msize << 10);

	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR &= ~(11 << 12);
	pDMAHandle->pDMAx->CH_REG[ChannelNo - 1].CCR |= (pDMAHandle->DMA_Config.DMA_ChannelPriority);

}


