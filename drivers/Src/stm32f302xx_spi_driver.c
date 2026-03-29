#include <stdint.h>
#include "stm32f302xx.h"
#include "stm32f302xx_spi_driver.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI2){
			SPI2_PCLK_ENABLE();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_ENABLE();

		}
	}else{
		if(pSPIx == SPI2){
			SPI2_PCLK_DISABLE();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DISABLE();
		}
	}
}

void SPI_Innit(SPI_Handle_t *pSPIHandle){
	if(!IS_SPI_DEVICE_MODE(pSPIHandle->SPI_Config.SPI_DeviceMode)){
		return;
	}

	if(!IS_SPI_BUS_CONFIG(pSPIHandle->SPI_Config.SPI_BusConfig)){
			return;
		}

	if(!IS_SPI_SSM(pSPIHandle->SPI_Config.SPI_ISM)){
			return;
		}

	if(!IS_SPI_SCLK_SPEED(pSPIHandle->SPI_Config.SPI_Speed)){
			return;
		}

	if(!IS_SPI_DFF(pSPIHandle->SPI_Config.SPI_DFF)){
			return;
		}

	if(!IS_SPI_CPOL(pSPIHandle->SPI_Config.SPI_CPOL)){
			return;
		}

	if(!IS_SPI_CPHA(pSPIHandle->SPI_Config.SPI_CPHA)){
			return;
		}

	uint32_t temp= 0;

	temp = (pSPIHandle->SPI_Config.SPI_DeviceMode)<<2;
	pSPIHandle->pSPIx->CR1 |= temp;

	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD){

		pSPIHandle->pSPIx->CR1 &= ~((1 << 15));
	} else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		pSPIHandle->pSPIx->CR1 |= (1 << 15);
		pSPIHandle->pSPIx->CR1 |= (1<<14);

	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX){
		pSPIHandle->pSPIx->CR1 &= ~((1 << 15));
		pSPIHandle->pSPIx->CR1 |= (1 << 10);

	}

	 pSPIHandle->pSPIx->CR1 &= ~(0x7 << 3);
	 pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_Config.SPI_Speed << 3);

	 if((pSPIHandle->SPI_Config.SPI_ISM) = SPI_SSM_EN){
		 temp = (pSPIHandle->SPI_Config.SPI_ISM << 9);
		 pSPIHandle->pSPIx->CR1 |= temp;

		 pSPIHandle->pSPIx->CR1 |= (1<<8);
	 }

	 pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPI_Config.SPI_CPHA;
	 pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPI_Config.SPI_CPOL;

	 if(pSPIHandle->SPI_Config.SPI_DFF == SPI_DFF_8BITS){
		 pSPIHandle->pSPIx->CR2 &= ~(0xF << 8);
		 pSPIHandle->pSPIx->CR2 |= (0x7 << 8);
		 pSPIHandle->pSPIx->CR2 &= ~(1 << 12);
	 }else {
		 pSPIHandle->pSPIx->CR2 &= ~(0xF << 8);
		 pSPIHandle->pSPIx->CR2 |= (0xF << 8);
		 pSPIHandle->pSPIx->CR2 |= (1 << 12);
	 }

}


void SPI_DeInnit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI2){
		 SPI2_REG_RESET();
	}else if(pSPIx == SPI3){
		 SPI3_REG_RESET();
	}

}

uint8_t SPI_GetStatusFlag(SPI_RegDef_t*pSPIx, uint8_t FlagName){
	if(pSPIx->SR & FlagName ){
		return FLAG_SET;
	}else {
		return FLAG_RESET;
	}
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while( Len > 0){
		while((SPI_GetStatusFlag(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)){
			uint32_t CRRead = pSPIx->CR2;
			uint8_t temp = ((uint8_t)(((CRRead >> 8))) & 0xF);
			if(temp == 0xF){
				pSPIx->DR = (*(uint16_t*)pTxBuffer);
				Len--;
				Len--;
				pTxBuffer = (((uint16_t*) pTxBuffer) + 1);
				pTxBuffer = (((uint16_t*) pTxBuffer) + 1);
			}else{
				pSPIx->DR = (*(uint8_t*)pTxBuffer);
				Len--;
				pTxBuffer = ((uint8_t*) pTxBuffer) + 1;
			}
		}
	}

}

void SPI_receiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while( Len > 0){
			while((SPI_GetStatusFlag(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)){
				uint32_t CRRead = pSPIx->CR2;
				uint8_t temp = ((uint8_t)(((CRRead >> 8))) & 0xF);
				if(temp == 0xF){
				  (*(uint16_t*)pTxBuffer) = pSPIx->DR;
					Len--;
					Len--;
					pTxBuffer = (((uint16_t*) pTxBuffer) + 1);
					pTxBuffer = (((uint16_t*) pTxBuffer) + 1);
				}else{
					pSPIx->DR = (*(uint8_t*)pTxBuffer);
					Len--;
					pTxBuffer = ((uint8_t*) pTxBuffer) + 1;
				}
			}
		}

	}

}


