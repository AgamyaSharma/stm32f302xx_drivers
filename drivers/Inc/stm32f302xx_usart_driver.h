

#ifndef INC_STM32F302XX_USART_DRIVER_H_
#define INC_STM32F302XX_USART_DRIVER_H_

#include <stdint.h>
#include "stm32f302xx.h"

#define BUFFER_SIZE							64
#define BUFFER_MASK							(BUFFER_SIZE - 1)

typedef struct{
	uint8_t  USART_Mode;
	uint8_t  USART_WordLength;
	uint32_t USART_BaudRate;
	uint8_t  USART_ClkControl;
	uint8_t  USART_CPHA;
	uint8_t  USART_CPOL;
	uint8_t  USART_OverSampling;
	uint8_t  USART_StopBits;
	uint8_t  USART_ParityControl;
	uint8_t  USART_HwFlowControl;
}USART_Config_t;

typedef struct{
	volatile uint8_t Buffer[BUFFER_SIZE];
	volatile uint32_t Head;
	volatile uint32_t Tail;
}USART_Buffer_t;

typedef struct{
	USART_RegDef_t    *pUSARTx;
	USART_Config_t 	  USART_Config;
	USART_Buffer_t    RxBuffer;
	USART_Buffer_t    TxBuffer;
	uint8_t 		  TxState;
	uint8_t			  RxState;
}USART_Handle_t;




#define USART1_PCLK_ENABLE()		(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 18))

#define USART1_PCLK_DISABLE()		(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 18))

#define USART1_REG_RESET()			do{((RCC->APB2RSTR) |= (1 << 14)); ((RCC->APB2RSTR) &= ~(1 << 14));}while(0)
#define USART2_REG_RESET()			do{((RCC->APB1RSTR) |= (1 << 17)); ((RCC->APB1RSTR) &= ~(1 << 14));}while(0)
#define USART3_REG_RESET()			do{((RCC->APB1RSTR) |= (1 << 18)); ((RCC->APB1RSTR) &= ~(1 << 14));}while(0)

#define USART_MODE_TX				1
#define USART_MODE_RX				2
#define USART_MODE_DUPLEX			3

#define USART_WL_7BIT				1
#define USART_WL_8BIT				2
#define USART_WL_9BIT				3

#define USART_CLK_ENABLE			1
#define USART_CLK_DISABLE			0

#define USART_CPHA_LOW				0
#define USART_CPHA_HIGH				1

#define USART_CPOL_LOW				0
#define USART_CPOL_HIGH				1

#define USART_SP_1					1
#define USART_SP_0_5				2
#define USART_SP_2					3
#define USART_SP_1_5				4

#define USART_PARITY_EVEN			1
#define USART_PARITY_ODD			2

#define USART_HWFLOW_RTSE			1
#define USART_HWFLOW_CTSE			2

#define USART_OVER_8				0
#define USART_OVER_16				1

#define USART_BUSY_IN_TX			1
#define USART_BUSY_IN_RX			2

#define IS_USART_MODE(MODE)				(MODE <= USART_MODE_DUPLEX)
#define IS_USART_WL_BITS(BITS)			(BITS <= USART_WL_9BIT)
#define IS_USART_CLK_MODE(CLK)			(CLK <= USART_CLK_ENABLE)
#define IS_USART_CPHA(CPHA)				(CPHA <= USART_CPHA_HIGH)
#define IS_USART_CPOL(CPOL)				(CPOL <= USART_CPOL_HIGH)
#define IS_USART_SP(SP)					(SP <= USART_SP_1_5)
#define IS_USART_PARITY(PARITY)			(PARITY <= USART_PARITY_ODD)

#define USART_TXE_FLAG                  (1 << 7)
#define USART_RXE_FLAG                  (1 << 5)

#define USART_EVENT_TX_CMPLT			1
#define USART_EVENT_RX_CMPLT			2





void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

void USART_Deinnit(USART_RegDef_t *pUSARTx);

void USART_Innit(USART_Handle_t *pUSARTHandle);

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);

void USART_RecieveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

void USART_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);

void USART_PriorityConfig(uint8_t IRQPriority,uint8_t IRQNumber);

void USART_IRQHandle(USART_Handle_t *pUSARTHandle );

uint8_t USART_GetStatusFlag(USART_RegDef_t*pSPIx, uint8_t FlagName);

uint8_t USART_Buffer_Push(USART_Buffer_t *pBuffer, uint8_t *tempData);

void USART_RecieveDataIt(USART_Handle_t *pUSARTHandle);

uint8_t USART_Buffer_Pop(USART_Buffer_t *pBuffer, uint8_t *pdata);

static void usart_rxne_interrupt_handle(USART_Handle_t *pUSARTHandle);

static void usart_txe_interrupt_handle(USART_Handle_t *pUSARTHandle);

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t APPEv);


#endif /* INC_STM32F302XX_USART_DRIVER_H_ */
