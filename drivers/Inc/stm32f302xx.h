/*
 * stm32f302xx.h
 *
 *
 */

#ifndef INC_STM32F302XX_H_
#define INC_STM32F302XX_H_
#include <stdint.h>
/*
 *  base addresses of Flash and SRAM memories
 */
#define FLASH_BASE_ADDR 					0x08000000U /* base address of the FLASH- 64Kb*/
#define SRAM1_BASE_ADDR						0x20000000U /* base address of the SRAM- 16Kb*/
#define SRAM 								SRAM1_BASE_ADDR /*SRAM1 is the main SRAM*/
#define ROM									0x1FFFD800U /*base address of the ROM*/
 /* 0x1FFF F800 - 0x1FFF FFFF [Option bytes], special non volatile memory, use with caution, only for read out protection*/
/*
 * base addresses of peripherals AHBX and APBx buses
 */
#define PERIPERH_BASE_ADDR					0x40000000U
#define APB1PERIPH_BASE_ADDR				PERIPERH_BASE_ADDR
#define APB2PERIPH_BASE_ADDR				0x40010000U
#define AHB1PERIPH_BASE_ADDR				0x40020000U
#define AHB2PERIPH_BASE_ADDR				0x48000000U
#define AHB3PERIPH_BASE_ADDR				0x50000000U

/*
 * base addresses of peripherals which are hanging on AHB1
 */
#define CRC_BASE_ADDR						(AHB1PERIPH_BASE_ADDR + 0x3000U)
#define DMA1_BASE_ADDR						(AHB1PERIPH_BASE_ADDR + 0x0000U)
#define FLASH_INT_BASE_ADDR					(AHB1PERIPH_BASE_ADDR + 0x2000U)
#define RCC_BASE_ADDR						(AHB1PERIPH_BASE_ADDR + 0x1000U)
#define TSC_BASE_ADDR						(AHB1PERIPH_BASE_ADDR + 0x4000U)


/*
 * base addresses of peripherals which are hanging on AHB2
 */
#define GPIOA_BASE_ADDR		   				(AHB2PERIPH_BASE_ADDR + 0x0000U)
#define GPIOB_BASE_ADDR						(AHB2PERIPH_BASE_ADDR + 0x0400U)
#define GPIOC_BASE_ADDR						(AHB2PERIPH_BASE_ADDR + 0x0800U)
#define GPIOD_BASE_ADDR						(AHB2PERIPH_BASE_ADDR + 0x0C00U)
#define GPIOF_BASE_ADDR						(AHB2PERIPH_BASE_ADDR + 0x1400U)
/*
 * base addresses of peripherals which are hanging on AHB3
 */
#define ADC1_BASE_ADDR						(AHB3PERIPH_BASE_ADDR + 0x0000U)

/*
 * base address of peripherals which are hanging on APB1
 */
#define BXCAN_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x6400U)
#define CAN_SRAM_BASE_ADDR					(APB1PERIPH_BASE_ADDR + 0x6000U)
#define DAC1_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x7400U)
#define I2C1_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x5400U)
#define I2C2_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x5800U)
#define I2C3_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x7800U)
#define I2S2_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x3800U)/* shares memory location with SPI2*/
#define I2S3_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x3C00U)/* shares memory location with SPI3*/
#define I2S2EXT_BASE_ADDR					(APB1PERIPH_BASE_ADDR + 0x3400U)
#define I2S3EXT_BASE_ADDR					(APB1PERIPH_BASE_ADDR + 0x4000U)
#define IWDG_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x3000U)
#define PWR_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x7000U)
#define RTC_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x2800U)
#define SPI2_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x3800U)/* shares memory location with I2S2*/
#define SPI3_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x3C00U)/* shares memory location with I2S3*/
#define TIM2_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x0000U)
#define TIM6_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x1000U)
#define USART2_BASE_ADDR					(APB1PERIPH_BASE_ADDR + 0x4400U)
#define USART3_BASE_ADDR					(APB1PERIPH_BASE_ADDR + 0x4800U)
#define USB_FS_BASE_ADDR					(APB1PERIPH_BASE_ADDR + 0x5C00U)
#define USB_BASE_ADDR					    (APB1PERIPH_BASE_ADDR + 0x6000U)
#define WWDG_BASE_ADDR						(APB1PERIPH_BASE_ADDR + 0x2C00U)

/*
 * base address of peripherals which are hanging on APB2
 */
#define COMP_BASE_ADDR                    	(APB2PERIPH_BASE_ADDR + 0x0000U)/* shares memory location with SYSFG*/
#define EXTI_BASE_ADDR 						(APB2PERIPH_BASE_ADDR + 0x0400U)
#define OP_BASE_ADDR                    	(APB2PERIPH_BASE_ADDR + 0x0000U)/* shares memory location with SYSFG*/
#define SYSCFG_BASE_ADDR                    (APB2PERIPH_BASE_ADDR + 0x0000U)
#define USART1_BASE_ADDR  					(APB2PERIPH_BASE_ADDR + 0x3800U)
#define TIM1_BASE_ADDR						(APB2PERIPH_BASE_ADDR + 0x2C00U)
#define TIM15_BASE_ADDR						(APB2PERIPH_BASE_ADDR + 0x4000U)
#define TIM16_BASE_ADDR						(APB2PERIPH_BASE_ADDR + 0x4400U)
#define TIM17_BASE_ADDR						(APB2PERIPH_BASE_ADDR + 0x4800U)
/*
 *
 */
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBRSTR;
	volatile uint32_t CFGR2;
	volatile uint32_t CFGR3;
}RCC_RegDef_t;

#define RCC ((RCC_RegDef_t*)RCC_BASE_ADDR)
/*
 *
 */
typedef struct {
	volatile uint32_t MODER;								/* GPIO General structure */
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
	volatile uint32_t BRR;
}GPIO_RegDef_t;


/*
 *
 */
#define GPIOA 								((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB 								((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC								((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD								((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOF								((GPIO_RegDef_t*)GPIOF_BASE_ADDR)

#define GPIOA_PCLK_EN()						(RCC->AHBENR |= (1<<17))
#define GPIOB_PCLK_EN()						(RCC->AHBENR |= (1<<18))
#define GPIOC_PCLK_EN()						(RCC->AHBENR |= (1<<19))
#define GPIOD_PCLK_EN()						(RCC->AHBENR |= (1<<20))
#define GPIOF_PCLK_EN()						(RCC->AHBENR |= (1<<22))

#define GPIOA_PCLK_DI()						(RCC->AHBENR &= ~(1<<17))
#define GPIOB_PCLK_DI()						(RCC->AHBENR &= ~(1<<18))
#define GPIOC_PCLK_DI()						(RCC->AHBENR &= ~(1<<19))
#define GPIOD_PCLK_DI()						(RCC->AHBENR &= ~(1<<20))
#define GPIOF_PCLK_DI()						(RCC->AHBENR &= ~(1<<22))

#define GPIOA_REG_RESET()					do{(RCC->AHBRSTR |= (1<<17)); (RCC->AHBENR &= ~(1<<17));}while(0)
#define GPIOB_REG_RESET()					do{(RCC->AHBRSTR |= (1<<18)); (RCC->AHBENR &= ~(1<<18));}while(0)
#define GPIOC_REG_RESET()					do{(RCC->AHBRSTR |= (1<<19)); (RCC->AHBENR &= ~(1<<19));}while(0)
#define GPIOD_REG_RESET()					do{(RCC->AHBRSTR |= (1<<20)); (RCC->AHBENR &= ~(1<<20));}while(0)
#define GPIOF_REG_RESET()					do{(RCC->AHBRSTR |= (1<<22)); (RCC->AHBENR &= ~(1<<22));}while(0)





#define ENABLE 								1
#define DISABLE 							0

#define SET 								ENABLE
#define RESET 								DISABLE

#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET
#endif /* INC_STM32F302XX_H_ */
