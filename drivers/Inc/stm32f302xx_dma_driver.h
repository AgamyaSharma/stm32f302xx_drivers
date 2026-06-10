

#ifndef INC_STM32F302XX_DMA_DRIVER_H_
#define INC_STM32F302XX_DMA_DRIVER_H_
#include <stdint.h>
#include "stm32f302xx.h"

typedef struct{
	uint8_t 	DMA_Channel;
	uint32_t	DMA_PeriAddr;
	uint32_t	DMA_MemAddr;
	uint8_t		DMA_SequenceLen;
	uint32_t	DMA_Psize;
	uint32_t	DMA_Msize;
	uint8_t		DMA_TransferMode;
	uint8_t		DMA_PINC;
	uint8_t		DMA_MINC;
	uint8_t		DMA_ChannelPriority;
	uint8_t		DMA_CIRC;
}DMA_Config_t;

typedef struct{
	DMA_RegDef_t *pDMAx;
	DMA_Config_t DMA_Config;
}DMA_Handle_t;

void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t EnOrDi);
void DMA_Innit(DMA_Handle_t *pDMAHandle);
void DMA_Deinnit(DMA_RegDef_t *pDMAx);
void DMA_StartTransfer(DMA_RegDef_t *pDMAx ,uint8_t Channelx);
void DMA_StopTransfer(DMA_RegDef_t *pDMAx, uint8_t Channelx);

#endif /* INC_STM32F302XX_DMA_DRIVER_H_ */


#define	PINC_ENABLED			ENABLE
#define PINC_DISABLED    		DISABLE

#define CIRC_ENABLED			ENABLE
#define CIRC_DISABLED			DISABLE

#define DMA_MODE_MEM2MEM		2
#define DMA_MODE_MEM2PERI		1
#define DMA_MODE_PERI2MEM		0

#define MSIZE_8BITS				0
#define MSIZE_16BITS			1
#define MSIZE_32BITS			2

#define PSIZE_8BITS				0
#define PSIZE_16BITS			1
#define PSIZE_32BITS			2





