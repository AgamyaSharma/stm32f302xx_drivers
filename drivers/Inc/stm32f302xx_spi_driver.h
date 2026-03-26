

#ifndef INC_STM32F302XX_SPI_DRIVER_H_
#define INC_STM32F302XX_SPI_DRIVER_H_
#include <stdint.h>
#include "stm32f302xx.h"


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

#define SPI_DEVICE_MODE_MASTER					1
#define SPI_DEVICE_MODE_SLAVE					0

#define SPI_BUS_CONFIG_FD						1
#define SPI_BUS_CONFIG_HD						2
#define SPI_BUS_CONFIG_SIMPLEX_TX				3
#define SPI_BUS_CONFIG_SIMPLEX_RX				4

#define SPI_SCLK_SPEED_DIV2						0
#define SPI_SCLK_SPEED_DIV4						1
#define SPI_SCLK_SPEED_DIV8						2
#define SPI_SCLK_SPEED_DIV16					3
#define SPI_SCLK_SPEED_DIV32					4
#define SPI_SCLK_SPEED_DIV64					5
#define SPI_SCLK_SPEED_DIV128					6
#define SPI_SCLK_SPEED_DIV256					7

#define SPI_DFF_8BITS							0
#define SPI_DFF_16BITS							1


#define SPI_CPOL_HIGH							1
#define SPI_CPOL_LOW							0

#define SPI_CPHA_HIGH							1
#define SPI_CPHA_LOW							0

#define SPI_SSM_EN							    1
#define SPI_SSM_DI							    0

#define IS_SPI_DEVICE_MODE(MODE)                    ((MODE) <= SPI_DEVICE_MODE_SLAVE)
#define IS_SPI_BUS_CONFIG(BUSCONFIG)				((BUSCONFIG) <= SPI_BUS_CONFIG_SIMPLEX_RX)
#define IS_SPI_SCLK_SPEED(SPEED)					((SPEED) <= SPI_SCLK_SPEED_DIV256)
#define IS_SPI_DFF(DFF)								((DFF) <= SPI_DFF_16BITS)
#define IS_SPI_CPOL(CPOL)							((CPOL) <= SPI_CPOL_HIGH)
#define IS_SPI_CPHA(CPHA)							((CPHA) <= SPI_CPHA_HIGH)
#define IS_SPI_SSM(SSM)								((SSM) <= SPI_SSM_EN)





void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


void SPI_Innit(SPI_Handle_t *pSPIHandle);


void SPI_DeInnit(SPI_RegDef_t *pSPIx);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);


void SPI_IRQHandle(SPI_Handle_t *pSPIHandle );

#endif /* INC_STM32F302XX_SPI_DRIVER_H_ */


