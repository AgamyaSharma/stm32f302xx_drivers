
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
	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) <= GPIO_PINMODE_ANALOG){
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) <= GPIO_OP_SPEED_HIGH){
		temp =((pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) << (2* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));  //TODO: Add the rest of the innit logic and the documentation for each functions
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp = 0;
	}
	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) <= GPIO_OP_TYPE_OP ){
		temp =((pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) <<  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER |= temp;
        temp = 0;
	}
}
void GPIO_DeInnit(GPIO_RegDef_t *pGPIOx){

}
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

}
void GPIO_ReadFromOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){

}
void GPIO_ReadFromOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){

}
void GPIO_IRQHandle(uint8_t PinNumber){

}
