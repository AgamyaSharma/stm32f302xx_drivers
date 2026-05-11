**STM32 Low-Level Peripheral Driver Library**
**Project Overview**
This repository contains a custom Hardware Abstraction Layer (HAL) for STM32 microcontrollers, developed from the ground up using ARM Cortex-M reference manuals and device datasheets. The project serves to bypass vendor-specific libraries to gain a direct understanding of silicon-level architecture, register-mapped I/O, and hardware-software synchronization.Core ArchitectureThe drivers utilize a structured design pattern to ensure deterministic execution and memory safety. Register Definition MappingPeripherals are mapped using C structures where each member corresponds to a specific hardware register address. To ensure exact alignment with the physical memory map provided in the datasheet, explicit array padding is used to account for reserved memory spaces.Implementation Detail: All hardware registers are qualified as volatile to ensure the compiler fetches the actual hardware state during every access, preventing incorrect optimization during polling or interrupt execution.

typedef struct{
	volatile uint32_t	  CR2;
	volatile uint32_t	  CR3;
	volatile uint32_t	  BRR; 
	volatile uint32_t	  GTPR;
	volatile uint32_t	  RTOR;
	volatile uint32_t	  RQR;
	volatile uint32_t	  ISR;
	volatile uint32_t	  ICR;
	volatile uint32_t	  RDR;
	volatile uint32_t	  TDR;
}USART_RegDef_t;

**Stateful Handle Design**
A "Handle" architecture is employed to support multiple instances of a peripheral. This decouples static hardware configuration (e.g., sample time, resolution) from the dynamic runtime state (e.g., active buffer pointers, transmission flags, or error codes)
 
  typedef struct{
	USART_RegDef_t    *pUSARTx;
	USART_Config_t 	  USART_Config;
	USART_Buffer_t    RxBuffer;
  USART_Buffer_t    TxBuffer;  //Circular Buffer
	uint8_t 		      TxState;   //Circular Biffer
	uint8_t			      RxState;
}USART_Handle_t; 

Non-Blocking Interrupt LogicThe library prioritizes non-blocking execution through Interrupt Service Routines (ISRs). The ISRs manage data extraction and internal state transitions, notifying the application layer through event callbacks once a sequence or data transfer is finalized.Implementation ExamplesThe following snippet demonstrates the high-level API usage, showing how the drivers abstract hardware complexity while maintaining full control over the configuration.
   EXAMPLE CODE SNIPPIT
   ADC_ConfigSequence_t mySequence[3] = {
    {1, ADC_SMP_601_5}, // Channel 1
    {2, ADC_SMP_601_5}, // Channel 2
    {3, ADC_SMP_601_5}  // Channel 3
};

ADC1_Handle.ADC_CONFIG.pSequence = mySequence;
ADC1_Handle.ADC_CONFIG.SequenceLen = 3;
ADC1_Handle.ADC_CONFIG.ADC_ConversionMode = ADC_MODE_CONT_CONVERSION;

ADC_Innit(&ADC1_Handle);

ADC1_Handle.pADCBuffer = SensorData;
ADC_StartConversion(ADC1);
ADC_RecieveDataIT(&ADC1_Handle);
  

**Driver Implementation Status**
**Completed Drivers**
**GPIO:** Mode selection, pull-up/down control, and alternate functions.
**USART:** Asynchronous IT mode with circular buffer.
**SPI:** Master mode configuration and data framing.
**ADC:** Continuous mode scanning and hardware interrupts.

**Currently in Testing**
**I2C:** Hardware state machine control.
**DMA:** Memory-to-peripheral data pipelines.

**Usage**
Include the required peripheral header from the Inc/ directory.
Define and initialize the peripheral configuration and handle structures.
If using interrupt-based drivers, ensure the corresponding IRQ handler is linked in the vector table.

Development Objectives
The development of this library focuses on:
**Architecture Mastery**: Understanding the internal bus matrix and peripheral interconnectivity of the ARM Cortex-M architecture.
**Bypassing Abstraction**: Eliminating vendor HAL overhead to achieve a minimal flash and RAM footprint.
