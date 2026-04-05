

#ifndef INC_STM32F302XX_USART_DRIVER_H_
#define INC_STM32F302XX_USART_DRIVER_H_

#include <stdint.h>
#include "stm32f302xx.h"

typedef struct{
	uint8_t  USART_Mode;
	uint8_t  USART_WordLength;
	uint32_t USART_BaudRate;
	uint8_t  USART_ClkControl;
	uint8_t  USART_CPHA;
	uint8_t  USART_CPOL;
	uint8_t  USART_StopBits;
	uint8_t  USART_ParityControl;
	uint8_t  USART_HwFlowControl;
}USART_Config_t;

typedef struct{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
}USART_Handle_t;




#define USART1_PCLK_ENABLE()		(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 18))

#define USART1_PCLK_DISABLE()		(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 18))

#define USART1_REG_RESET()			do{((RCC->APB2RSTR) |= (1 << 14)); ((RCC->APB2RSTR) &= ~(1 << 14));}while(0)
#define USART2_REG_RESET()			do{((RCC->APB1RSTR) |= (1 << 14)); ((RCC->APB1RSTR) &= ~(1 << 14));}while(0)
#define USART3_REG_RESET()			do{((RCC->APB1RSTR) |= (1 << 14)); ((RCC->APB1RSTR) &= ~(1 << 14));}while(0)

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

void USART_Deinnit(USART_RegDef_t *pUSARTx);

void USART_Innit(USART_RegDef_t *pUSARTHandle);

void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len );

void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len );

void USART_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);

void USART_PriorityConfig(uint8_t IRQPriority,uint8_t IRQNumber);

void USART_IRQHandle(USART_Handle_t *pUSARTHandle );
#endif /* INC_STM32F302XX_USART_DRIVER_H_ */
