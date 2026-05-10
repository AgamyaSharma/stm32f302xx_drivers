#ifndef INC_STM32F302XX_ADC_DRIVER_H_
#define INC_STM32F302XX_ADC_DRIVER_H_

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
	uint8_t			ADC_State;
	uint16_t		*pADCBuffer;
	uint8_t			RxLen;
}ADC_Handle_t;



void ADC_Innit(ADC_Handle_t *pADCHandle);

void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t EnOrDi);

void ADC_Deinnit(ADC_Handle_t *pADCHandle);

void ADC_StartConversion(ADC_RegDef_t *pADCx);

void ADC_StopConversion(ADC_RegDef_t *pADCx);

void ADC_RecieveData(ADC_Handle_t *pADCHandle);

static void adc_eoc_interrupt_handle(ADC_Handle_t *pADCHandle);

void ADC_RecieveDataIT(ADC_Handle_t *pADCHandle);

void ADC_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);

void ADC_PriorityConfig(uint8_t IRQPriority,uint8_t IRQNumber);

__attribute__((weak)) void ADC_ApplicationEventCallback(ADC_Handle_t *pADCHandle,uint8_t APPEv);

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

#define ADC_SMP_1_5						0
#define ADC_SMP_2_5						1
#define ADC_SMP_4_5						2
#define ADC_SMP_7_5						3
#define ADC_SMP_19_5					4
#define ADC_SMP_61_5					5
#define ADC_SMP_181_5					6
#define ADC_SMP_601_5					7

#define ADC_EXTEN_DISABLED          	0
#define ADC_EXTEN_RISING_EDGE      	 	1
#define ADC_EXTEN_FALLING_EDGE      	2
#define ADC_EXTEN_BOTH_EDGES        	3

#define ADC_EXTSEL_TIM1_CC1         0x00
#define ADC_EXTSEL_TIM1_CC2         0x01
#define ADC_EXTSEL_TIM1_CC3         0x02
#define ADC_EXTSEL_TIM2_CC2         0x03
#define ADC_EXTSEL_TIM3_TRGO        0x04
#define ADC_EXTSEL_TIM4_CC4         0x05
#define ADC_EXTSEL_EXTI_11          0x06
// 0x07 (0111) is Reserved
// 0x08 (1000) is Reserved
#define ADC_EXTSEL_TIM1_TRGO        0x09
#define ADC_EXTSEL_TIM1_TRGO2       0x0A
#define ADC_EXTSEL_TIM2_TRGO        0x0B
#define ADC_EXTSEL_TIM4_TRGO        0x0C
#define ADC_EXTSEL_TIM6_TRGO        0x0D
#define ADC_EXTSEL_TIM15_TRGO       0x0E
#define ADC_EXTSEL_TIM3_CC4         0x0F

#define ADC_BUSY					1

#define ADC_EVENT_EOC				1

#endif /* INC_STM32F302XX_ADC_DRIVER_H_ */
