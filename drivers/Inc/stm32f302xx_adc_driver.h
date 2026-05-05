#ifndef INC_STM32F302XX_ADC_DRIVER_H_
#define INC_STM32F302XX_ADC_DRIVER_H_
#include <stdio.h>
#include <stdint.h>
#include "stm32f302xx.h"

typedef struct{
	uint8_t					ADC_Calibration;
	uint8_t					ADC_ConversionMode;
	uint8_t					ADC_DataRes;
	ADC_ConfigSequence_t 	*pSequence;
	uint8_t					SequenceLen;
	uint8_t					ADC_OVRhandle;
	uint8_t					ADC_DataAlign;
}ADC_Config_t;

typedef struct{
	uint8_t 	Channel;
	uint8_t		SampleTime;
}ADC_ConfigSequence_t;

typedef struct{
	ADC_RegDef_t 	*pADCx;
	ADC_Config_t 	ADC_CONFIG;
};

#endif /* INC_STM32F302XX_ADC_DRIVER_H_ */
