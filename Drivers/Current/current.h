#ifndef __CURRENT_H
#define __CURRENT_H
#endif

#include "main.h"
#include "math.h"

#define USING_5A_SENSOR 	1
#define USING_20A_SENSOR	0

#if USING_5A_SENSOR
	
	// Collibration ---------------------
	#define FIRST_CURRENT_mA 	3143
	#define FIRST_ADC_VALUE	 	2674.0

	#define SECOND_CURRENT_mA 0
	#define SECOND_ADC_VALUE	2014.0
	//-----------------------------------

#elif USING_20A_SENSOR
	
	// Collibration ---------------------
	#define FIRST_CURRENT_mA 	12572
	#define FIRST_ADC_VALUE	 	2474.0

	#define SECOND_CURRENT_mA 0
	#define SECOND_ADC_VALUE	1945.0
	//-----------------------------------

#else
	#error "This liberary only supports 5 and 20A sensors"
#endif

#define NUMBER_OF_SAMPLES 400
#define FILTER_POLE				1000
#define SAMPLE_PERIOD			((__my_htim->Init.Period+1)/1000000.0)

uint8_t current_start(void (*func)(uint16_t,uint32_t), TIM_HandleTypeDef* htim, ADC_HandleTypeDef* hadc);
void current_stop(void);
uint16_t __callculate_current(uint32_t input);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
