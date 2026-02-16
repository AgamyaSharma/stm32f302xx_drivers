#include <stdint.h>

#include "stm32f302xx.h"
#include "stm32f302xx_gpio_driver.h"


int main(void)
{
	GPIO_Handle_t MyLed = {0};
	MyLed.pGPIOx = GPIOB;
	MyLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	MyLed.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_OUTPUT;
	GPIO_PeriClockControl(GPIOB,ENABLE);
	GPIO_Innit(&MyLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOB,GPIO_PIN_NUMBER_13);
		for(int i=0; i<500000; i++){
		}
	}
	for(;;);
}

