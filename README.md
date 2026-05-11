# STM32 Low-Level Peripheral Driver Library

## Project Overview
This repository contains a custom Hardware Abstraction Layer (HAL) for STM32 microcontrollers, developed from the ground up using ARM Cortex-M reference manuals and device datasheets. 

The project serves to bypass vendor-specific libraries to gain a direct understanding of:
* Silicon-level architecture
* Register-mapped I/O
* Hardware-software synchronization

---

## Core Architecture

### 1. Register Definition Mapping
Peripherals are mapped using C structures where each member corresponds to a specific hardware register address. To ensure exact alignment, explicit array padding is used to account for reserved memory spaces.

**Implementation Detail:** All hardware registers are qualified as `volatile` to ensure the compiler fetches the actual hardware state during every access.

```c
typedef struct {
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t BRR;
    volatile uint32_t GTPR;
    volatile uint32_t RTOR;
    volatile uint32_t RQR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t RDR;
    volatile uint32_t TDR;
} USART_RegDef_t;

2. Stateful Handle Design
A "Handle" architecture is employed to support multiple instances of a peripheral. This decouples static hardware configuration from the dynamic runtime state.  


typedef struct {
    USART_RegDef_t *pUSARTx;
    USART_Config_t  USART_Config;
    USART_Buffer_t  RxBuffer;
    USART_Buffer_t  TxBuffer;
    uint8_t         TxState;
    uint8_t         RxState;
} USART_Handle_t;

The following snippet demonstrates the high-level API usage for a multi-channel ADC sequence using interrupts.


ADC_ConfigSequence_t mySequence[3] = {
    {1, ADC_SMP_601_5}, 
    {2, ADC_SMP_601_5}, 
    {3, ADC_SMP_601_5} 
};

ADC1_Handle.ADC_CONFIG.pSequence = mySequence;
ADC1_Handle.ADC_CONFIG.SequenceLen = 3;
ADC1_Handle.ADC_CONFIG.ADC_ConversionMode = ADC_MODE_CONT_CONVERSION;

ADC_Innit(&ADC1_Handle);

ADC1_Handle.pADCBuffer = SensorData;
ADC_StartConversion(ADC1);
ADC_RecieveDataIT(&ADC1_Handle);

ADC1_Handle.ADC_CONFIG.pSequence = mySequence;
ADC1_Handle.ADC_CONFIG.SequenceLen = 3;
ADC1_Handle.ADC_CONFIG.ADC_ConversionMode = ADC_MODE_CONT_CONVERSION;

ADC_Innit(&ADC1_Handle);

ADC1_Handle.pADCBuffer = SensorData;
ADC_StartConversion(ADC1);
ADC_RecieveDataIT(&ADC1_Handle);
Driver Implementation Status
 Completed Drivers
GPIO: Mode selection and alternate function mapping.

USART: Asynchronous IT mode with circular buffer.

SPI: Master mode configuration and data framing.

ADC: Continuous mode scanning and hardware interrupts.

🟡 Currently in Testing
I2C: Hardware state machine control.

DMA: Memory-to-peripheral data pipelines.

Development Objectives
Architecture Mastery: Understanding the internal bus matrix of the ARM Cortex-M.
Bypassing Abstraction: Eliminating vendor HAL overhead for minimal footprint.