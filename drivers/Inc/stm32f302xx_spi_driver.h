

#ifndef INC_STM32F302XX_SPI_DRIVER_H_
#define INC_STM32F302XX_SPI_DRIVER_H_
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_ISM;
	uint8_t SPI_Speed;
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
}SPI_Handle_t;
#endif /* INC_STM32F302XX_SPI_DRIVER_H_ */


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


void SPI_Innit(SPI_Handle_t *pSPIHandle);


void SPI_DeInnit(SPI_RegDef_t *pSPIx);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);


void SPI_IRQHandle(SPI_Handle *pSPIHandle );
