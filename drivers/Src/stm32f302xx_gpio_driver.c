
#include "stm32f302xx.h"
#include "stm32f302xx_gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){

			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB){

			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOB){

			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC){

			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD){

			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOF){

			GPIOF_PCLK_EN();
		}

	}else if(EnorDi){
		if(pGPIOx == GPIOA){

			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){

			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){

			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){

			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOF){

			GPIOF_PCLK_DI();
		}
	}
}
void GPIO_Innit(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;

	if(!IS_GPIO_PIN_NUMBER(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ){

		return;
	}

	if(!IS_GPIO_PINMODE(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)){

		return;
	}

	if(!IS_GPIO_OP_SPEED(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed)){

		return;
	}

	if(!IS_GPIO_OP_TYPE(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) ){

		return;
	}

	if(!IS_GPIO_PUPD_CONTROL(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl) ){
		return;
	}
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_PINMODE_ANALOG){

	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->MODER |= temp;
	}else {
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PINMODE_IT_FT){
			EXTI->FTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PINMODE_IT_RT){
			EXTI->RTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

				}

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PINMODE_IT_RFT){
			EXTI->FTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinMode);
				}

		EXTI->IMR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		uint8_t temp1 =((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4);
		uint8_t temp2 =((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4);
		SYSFCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = ((GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx)) << (temp2 * 4));

	}

	temp =((pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) <<  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 <<  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;


    temp =((pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) <<  (2* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl) << (2* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PINMODE_ALTFN){
    	uint32_t temp1,temp2;
    	temp1 = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8);
    	temp2 = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8);
    	(pGPIOHandle->pGPIOx->AFR[temp1]) &= ~(0xF << (4 * temp2));
    	(pGPIOHandle->pGPIOx->AFR[temp1]) |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunctionMode << (4 * temp2));
    }


}
void GPIO_DeInnit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){

		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB){

		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC){

		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD){

		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOF){

		GPIOF_REG_RESET();
	}

}
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
		value = (uint16_t)(pGPIOx->IDR );
		return value;

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){
	if (value == GPIO_PIN_SET){

		pGPIOx->ODR |= (1 << PinNumber);
	}else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1<<PinNumber);

}
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi== Enable){
		if(IRQNumber <=31){

			NVIC->ISER[0] |= (1<< IRQNUMBER);

		}else if(IRQNumber > 31 && IRQNUMBERS <=63){

			NVIC->ISER[1] |= (1<< IRQNUMBER % 32);

		}else if(IRQNumber > 63 && IRQNUMBER <=95){

			NVIC->ISER[2] |= (1<< (IRQNUMBER % 64));

		}

	}else {
		if(IRQNumber <=31){

			NVIC->ICER[0] |= (1<<IRQNumber);
		}else if(IRQNumber > 31 && IRQNUMBERS <=63){

			NVIC->ICER[1] |= (1<<(IRQNumber % 32));
		}else if(IRQNumber > 63 && IRQNUMBER <=95){

			NVIC->ICER[2] |= (1<<(IRQNumber % 64));

		}

	}

}

void GPIO_PriorityConfig(uint8_t IRQPriority,uint8_t IRQNumber){
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprxSection = (IRQNumber % 4);
	uint8_t shiftAmount =((8*iprxSection) + 4);
	NVIC->IPR[iprx] |= (IRQPriority << (shiftAmount));
}
void GPIO_IRQHandle(uint8_t PinNumber){
	if (PinNumber < 32) {
	        // Handle Lines 0 to 31 in PR1
		if (EXTI->PR1 & (1 << PinNumber)) {
			EXTI->PR1 |= (1 << PinNumber);
	        }
	    } else {
	        // Handle Lines 32 and above in PR2
	        // We subtract 32 so that Line 32 becomes bit 0, Line 33 becomes bit 1, etc.
	        if (EXTI->PR2 & (1 << (PinNumber - 32))) {
	            EXTI->PR2 |= (1 << (PinNumber - 32));
	        }
	    }
}
