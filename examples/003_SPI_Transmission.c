#include <stdint.h>
#include "stm32f302xx.h"
#include "stm32f302xx_gpio_driver.h"
#include "stm32f302xx_spi_driver.h"


int main(void)
{
	// GPIO config for SPI Mode
	GPIO_Handle_t SPIPins = {0};
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunctionMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // These are general configs required for SPI Peripherals
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	GPIO_PeriClockControl(GPIOB,ENABLE);
	// clk must be enabled before writing into any register

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	GPIO_Innit(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_14;
	GPIO_Innit(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_15;
	GPIO_Innit(&SPIPins);
	SPI_Handle_t SPITX = {0};

	//SPI config for Transmission
	SPITX.pSPIx = SPI2;
	SPITX.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPITX.SPI_Config.SPI_CPOL = SPI_CPOL_LOW; // To be configured as per the module or the sensor
	SPITX.SPI_Config.SPI_CPHA = SPI_CPHA_LOW; // To be configured as per the module or the sensor
	SPITX.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPITX.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPITX.SPI_Config.SPI_Speed = SPI_SCLK_SPEED_DIV16;
	SPITX.SPI_Config.SPI_ISM = SPI_SSM_EN; //enabled so that the Software pulls high and low in singular master mode
	SPI_PeriClockControl(SPI2,ENABLE); // clk must be enabled before writing into any register
	SPI_Innit(&SPITX);

	uint8_t tx_buffer[5] = {0x42, 0x01, 0xA1, 0xFF, 0x00}; // transmission buffer
	while(1){
		SPI_SendData(SPI2, tx_buffer,5);
		for(int i=0; i<500000; i++){

		}
	}

	for(;;);
}
