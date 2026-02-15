

#ifndef INC_STM32F302XX_GPIO_DRIVER_H_
#define INC_STM32F302XX_GPIO_DRIVER_H_
#include <stdint.h>
#include "stm32f302xx.h"
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

#define GPIO_PIN_NUMBER_0 								0
#define GPIO_PIN_NUMBER_1 								1
#define GPIO_PIN_NUMBER_2 								2
#define GPIO_PIN_NUMBER_3 								3
#define GPIO_PIN_NUMBER_4 								4
#define GPIO_PIN_NUMBER_5 								5
#define GPIO_PIN_NUMBER_6 								6
#define GPIO_PIN_NUMBER_7 								7
#define GPIO_PIN_NUMBER_8 								8
#define GPIO_PIN_NUMBER_9 								9
#define GPIO_PIN_NUMBER_10 								10
#define GPIO_PIN_NUMBER_11 								11
#define GPIO_PIN_NUMBER_12 								12
#define GPIO_PIN_NUMBER_13 								13
#define GPIO_PIN_NUMBER_14 								14
#define GPIO_PIN_NUMBER_15 								15

#define GPIO_PINMODE_INPUT								0
#define GPIO_PINMODE_OUTPUT								1
#define GPIO_PINMODE_ALTFN								2
#define GPIO_PINMODE_ANALOG								3
/*
 * space for interrupts later
 */

#define GPIO_NO_PUPD									0
#define GPIO_PIN_PU    									1
#define GPIO_PIN_PD 									2


#define GPIO_OP_TYPE_PP									0
#define GPIO_OP_TYPE_OP									1

#define GPIO_OP_SPEED_LOW								0
#define GPIO_OP_SPEED_MED								1
#define GPIO_OP_SPEED_HIGH								3

#define GPIO_MAX_PIN_NUMBER								15
#define IS_GPIO_PIN_NUMBER(PIN)							((PIN) <= GPIO_MAX_PIN_NUMBER)

#define IS_GPIO_PINMODE(MODE)							((MODE) <= GPIO_PINMODE_ANALOG)

#define IS_GPIO_PUPD_CONTROL(PUPD)						((PUPD) <= GPIO_PIN_PD)

#define IS_GPIO_OP_TYPE(OPTYPE)							((OPTYPE) <= GPIO_OP_TYPE_OP)

#define IS_GPIO_OP_SPEED(SPEED)							((SPEED) <= GPIO_OP_SPEED_HIGH)




void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);





void GPIO_Innit(GPIO_Handle_t *pGPIOHandle);


void GPIO_DeInnit(GPIO_RegDef_t *pGPIOx);


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);


void GPIO_ReadFromOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);


void GPIO_ReadFromOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);


void GPIO_IRQHandle(uint8_t PinNumber);






#endif /* INC_STM32F302XX_GPIO_DRIVER_H_ */
