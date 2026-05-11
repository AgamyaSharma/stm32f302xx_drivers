#include <stdint.h>
#include "stm32f302xx.h"
#include "stm32f302xx_adc_driver.h"
#include "stm32f302xx_gpio_driver.h"

#define SEQUENCE_LENGTH 3

//  Global Handle
ADC_Handle_t ADC1_Handle;

//  (Array of Structs)
ADC_ConfigSequence_t ADC1_Sequence[SEQUENCE_LENGTH];

//  Data Buffer
uint16_t SensorData[SEQUENCE_LENGTH];
volatile uint8_t SequenceCompleteFlag = 0;

void GPIO_AnalogSetup(void) {
	GPIO_Handle_t ADCPins = {0};

	    ADCPins.pGPIOx = GPIOA;

	    // Enable the clock for Port A first
	    GPIO_PeriClockControl(GPIOA, ENABLE);

	    //  Set mode to Analog.
	    ADCPins.GPIO_PinConfig.GPIO_PinMode = GPIO_PINMODE_ANALOG;

	    // CRITICAL: Disable all pull-up / pull-down resistors.
	    ADCPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	    ADCPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_0;
	    GPIO_Init(&ADCPins);

	    // PA1
	    ADCPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_1;
	    GPIO_Init(&ADCPins);

	    // PA2
	    ADCPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_2;
	    GPIO_Init(&ADCPins);
	}


void ADC1_Setup(void) {

    ADC1_Handle.pADCx = ADC1;


    ADC1_Sequence[0].Channel = 1;
    ADC1_Sequence[0].SampleTime = ADC_SMP_601_5;

    ADC1_Sequence[1].Channel = 2;
    ADC1_Sequence[1].SampleTime = ADC_SMP_601_5;

    ADC1_Sequence[2].Channel = 3;
    ADC1_Sequence[2].SampleTime = ADC_SMP_601_5;

    // Configure the Core Settings
    ADC1_Handle.ADC_CONFIG.pSequence = ADC1_Sequence;
    ADC1_Handle.ADC_CONFIG.SequenceLen = SEQUENCE_LENGTH;
    ADC1_Handle.ADC_CONFIG.ADC_DataRes = ADC_DATA_RES_12BIT;
    ADC1_Handle.ADC_CONFIG.ADC_DataAlign = ADC_DATA_RIGHT_ALIGN;
    ADC1_Handle.ADC_CONFIG.ADC_ConversionMode = ADC_MODE_CONT_CONVERSION;
    ADC1_Handle.ADC_CONFIG.ADC_Calibration = ADC_CAL_SINGLE;
    ADC1_Handle.ADC_CONFIG.ADC_OVRhandle = ADC_OVRMODE_ENABLE;
    ADC1_Handle.ADC_CONFIG.ADC_ExtTrig_Source = 0; // Software Trigger
    ADC1_Handle.ADC_CONFIG.ADC_ExtTrig_Edge = ADC_EXTEN_DISABLED;

    // Enable the peripheral clock and initialize
    ADC_PeriClockControl(ADC1, ENABLE);
    ADC_Innit(&ADC1_Handle);
}

int main(void) {
    GPIO_AnalogSetup();
    ADC1_Setup();

    ADC_IRQConfig(18, ENABLE); // 18 is  ADC1_2_IRQn on STM32F3

    ADC1_Handle.pADCBuffer = SensorData;
    ADC1_Handle.RxLen = 0;


    ADC_StartConversion(ADC1);
    ADC_RecieveDataIT(&ADC1_Handle);

    while(1) {
        if (SequenceCompleteFlag == 1) {
            SequenceCompleteFlag = 0;
        }
    }
}


void ADC1_2_IRQHandler(void) {
    ADC_IRQHandle(&ADC1_Handle);
}

void ADC_ApplicationEventCallback(ADC_Handle_t *pADCHandle, uint8_t APPEv) {
    if (APPEv == ADC_EVENT_EOC) {
        if (pADCHandle->RxLen == 0) {
            SequenceCompleteFlag = 1;
        }
    }
}
