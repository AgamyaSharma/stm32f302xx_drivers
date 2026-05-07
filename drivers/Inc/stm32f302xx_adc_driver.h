#ifndef INC_STM32F302XX_ADC_DRIVER_H_
#define INC_STM32F302XX_ADC_DRIVER_H_
#include <stdio.h>
#include <stdint.h>
#include "stm32f302xx.h"

typedef struct{
	uint8_t 	Channel;
	uint8_t		SampleTime;
}ADC_ConfigSequence_t;

typedef struct{
	uint8_t					ADC_Calibration;
	uint8_t					ADC_ConversionMode;
	uint8_t					Discon_ChunkSize;
	uint8_t					ADC_DataRes;
	ADC_ConfigSequence_t 	*pSequence;
	uint8_t					SequenceLen;
	uint8_t					ADC_OVRhandle;
	uint8_t					ADC_DataAlign;
	uint8_t					ADC_ExtTrig_Source;
	uint8_t					ADC_ExtTrig_Edge;
}ADC_Config_t;


typedef struct{
	ADC_RegDef_t 	*pADCx;
	ADC_Config_t 	ADC_CONFIG;
}ADC_Handle_t;



void ADC_Innit(ADC_Handle_t *pADCHandle);

void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t EnOrDi);

void ADC_Deinnit(ADC_Handle_t *pADCHandle);



#define ADC_CAL_DIF						1
#define ADC_CAL_SINGLE					0

#define ADC_MODE_DISCON_CONVERSION		2
#define ADC_MODE_CONT_CONVERSION		1
#define ADC_MODE_SINGLE_CONVERSION		0


#define ADC_DATA_RIGHT_ALIGN			0
#define ADC_DATA_LEFT_ALIGN				1

#define ADC_DATA_RES_12BIT				0
#define ADC_DATA_RES_10BIT				1
#define ADC_DATA_RES_8BIT				2
#define ADC_DATA_RES_6BIT				3

#define ADC_OVRMODE_ENABLE				1
#define ADC_OVRMODE_DISABLE             0
#endif /* INC_STM32F302XX_ADC_DRIVER_H_ */
