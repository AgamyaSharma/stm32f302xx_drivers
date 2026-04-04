

#ifndef INC_STM32F302XX_USART_DRIVER_H_
#define INC_STM32F302XX_USART_DRIVER_H_

#include <stdint.h>
#include "stm32f302xx.h"

typedef struct{

}USART_Config_t;

typedef struct{
	USART_RegDef_t *pUSARTx;
	USART_Config_t SPIConfig;
}USART_Handle_t;


#define USART1						(USART_Reg_Def_t*(USART1_BASE_ADDR))
#define USART2						(USART_Reg_Def_t*(USART2_BASE_ADDR))
#define USART3						(USART_Reg_Def_t*(USART3_BASE_ADDR))

#define USART1_PCLK_ENABLE()		(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 18))

#define USART1_PCLK_DISABLE()		(RCC->APB2ENR &= ~(1 << 14))
#define USART1_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 18))

#define USART1_REG_RESET()			do{((RCC->AP2RSTR) |= (1 << 14)); ((RCC->AP2RSTR) &= ~(1 << 14));}while(0)
#define USART2_REG_RESET()			do{((RCC->AP1RSTR) |= (1 << 14)); ((RCC->AP1RSTR) &= ~(1 << 14));}while(0)
#define USART3_REG_RESET()			do{((RCC->AP1RSTR) |= (1 << 14)); ((RCC->AP1RSTR) &= ~(1 << 14));}while(0)

void USART_PeriClockControl(USART_RegDef_t *pSPIx, uint8_t EnorDi);

void USART_Deinnit(USART_RegDef_t *pUSARTHandle);

void USART_Innit(USART_RegDef_t *pUSARTHandle);

void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len );

void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len );

void USART_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);

void USART_PriorityConfig(uint8_t IRQPriority,uint8_t IRQNumber);

void USART_IRQHandle(SPI_Handle_t *pUSARTHandle );
#endif /* INC_STM32F302XX_USART_DRIVER_H_ */
