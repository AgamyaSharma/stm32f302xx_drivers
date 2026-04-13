#include <stdint.h>
#include "stm32f302xx.h"
#include "stm32f302xx_gpio_driver.h"
#include "stm32f302xx_usart_driver.h"

int main(void)


{

	GPIO_Handle_t MyLed = {0};
	MyLed.pGPIOx = GPIOB;
	MyLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	MyLed.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_OUTPUT;
	GPIO_PeriClockControl(GPIOB,ENABLE);
	GPIO_Innit(&MyLed);
	GPIO_PeriClockControl(GPIOA,ENABLE);
	USART_Handle_t USARTSendData = {0};
	GPIO_Handle_t USARTPins = {0};

	USARTPins.pGPIOx = GPIOB;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed =  GPIO_OP_SPEED_HIGH;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = 0;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunctionMode = 7;

	USARTPins.GPIO_PinConfig.GPIO_PinNumber = 10;
	GPIO_Innit(&USARTPins);
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = 11;
	GPIO_Innit(&USARTPins);
	USART_PeriClockControl(USART3,ENABLE);
	USARTSendData.pUSARTx = USART3;
	USARTSendData.USART_Config.USART_Mode =  USART_MODE_DUPLEX;
	USARTSendData.USART_Config.USART_WordLength = USART_WL_8BIT;
	USARTSendData.USART_Config.USART_OverSampling = USART_OVER_16;
	USARTSendData.USART_Config.USART_StopBits = USART_SP_1;
	USARTSendData.USART_Config.USART_BaudRate = 9600;
	uint8_t tx_buffer[1] = {0x42};
	uint8_t rx_buffer[1] = {0};
	USARTSendData.pTxBuffer = tx_buffer;
	USARTSendData.pRxBuffer = rx_buffer;
	USART_Innit(&USARTSendData);
	while(1){
	USART_SendData(&USARTSendData,1);
	USART_ReceiveData(&USARTSendData,1);
	if(rx_buffer[0] == 0x42){
			GPIO_ToggleOutputPin(GPIOB,GPIO_PIN_NUMBER_13);
			}
	for(int i=0; i<500000; i++){

				}



	}
for(;;);
}
