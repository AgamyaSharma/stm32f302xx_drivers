

#ifndef INC_STM32F302XX_GPIO_DRIVER_H_
#define INC_STM32F302XX_GPIO_DRIVER_H_

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunctionMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

void GPIO_PeriClockControl(void);
void GPIO_Innit(void);
void GPIO_DeInnit(void);
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_ReadFromOutputPin(void);
void GPIO_ReadFromOutputPort(void);
void GPIO_IRQConfig(void);
void GPIO_IRQHandler(void);




#endif /* INC_STM32F302XX_GPIO_DRIVER_H_ */
