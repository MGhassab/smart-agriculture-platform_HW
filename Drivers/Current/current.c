#include "current.h"

const double SLOPE = (FIRST_CURRENT_mA - SECOND_CURRENT_mA)/(FIRST_ADC_VALUE - SECOND_ADC_VALUE);
const double ARZ_AZ_MABDA = FIRST_CURRENT_mA - SLOPE * FIRST_ADC_VALUE;

TIM_HandleTypeDef* __my_htim;
ADC_HandleTypeDef* __my_hadc;
void (*__run_me_on_final__)(uint16_t,uint32_t);

uint32_t __raw_values  [NUMBER_OF_SAMPLES];
uint32_t __LP_Filtered [NUMBER_OF_SAMPLES]; 
uint32_t __AR_Filtered [NUMBER_OF_SAMPLES];
uint16_t __current = 0.0;
float    __alpha   = 0.0;

/**
  * @brief  Start the timer-trigered and DMA-coupled ADC. 
  * @param  func is the callback function that runs after conversion.
  *         htim is the timer handler.
	* 				hadc is the adc handler.
  * @retval 1 in case of success, otherwise 0.
  */
uint8_t current_start(void (*func)(uint16_t,uint32_t), TIM_HandleTypeDef* htim, ADC_HandleTypeDef* hadc){
	__my_htim = htim;
	__my_hadc = hadc;
	__run_me_on_final__= func;
	
	if(HAL_OK !=	HAL_TIM_Base_Start(__my_htim))
		return 0;
	if(HAL_OK != HAL_ADC_Start_DMA(hadc, __raw_values, NUMBER_OF_SAMPLES))
		return 0;
	return 1;
}

/**
  * @brief  Stop the conversion. 
  * @retval None
  */
void current_stop(void){	
	HAL_TIM_Base_Stop(__my_htim);
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
	*					This routin has some steps:
	*					1- Stop timer:
	*						Stops the timer to avoid changes in DMA memory
	*						during processes.
	*					2- Setting the initial condition of filters.
	*					3- Apply a low-pass filter
	*					4- Apply a attack-release filter
	*					5- Start timer
	*					6- Pass the final value and final current to the user-defined function
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc==__my_hadc){

		//STOP TIMER3 
		//*
		{
		__my_htim->State = HAL_TIM_STATE_BUSY;
		__HAL_TIM_DISABLE(__my_htim);
		__my_htim->State = HAL_TIM_STATE_READY;
		}
		//*/
				
		__AR_Filtered[0] = __LP_Filtered[NUMBER_OF_SAMPLES-1];
		__LP_Filtered[0] = __LP_Filtered[NUMBER_OF_SAMPLES-1];		
		
		for(int i = 1; i<NUMBER_OF_SAMPLES; i++){			
			__LP_Filtered[i] = exp(-1*FILTER_POLE*SAMPLE_PERIOD)*__LP_Filtered[i-1] + (1-exp(-1*FILTER_POLE*SAMPLE_PERIOD)) * __raw_values[i];

			if(__LP_Filtered[i]<__AR_Filtered[i-1])
				__alpha = 1.0;
      else
				__alpha = 0.5;
			
      __AR_Filtered[i] = (__alpha*((float)__AR_Filtered[i-1]) + (1-__alpha)*((float)__LP_Filtered[i]));
		}
				 
		//START TIMER3
		//*
		{
		__my_htim->State = HAL_TIM_STATE_BUSY;
		__HAL_TIM_ENABLE(__my_htim);
		__my_htim->State = HAL_TIM_STATE_READY;
		}
		//*/

		__current = __callculate_current(__AR_Filtered[NUMBER_OF_SAMPLES/2]);
		(*__run_me_on_final__)(__current,__AR_Filtered[NUMBER_OF_SAMPLES/2]);
	}
}

/**
  * @brief  Calculate current from raw value according to calibration parameters.
  * @param  input is the raw value.
  * @retval current in mA unit.
  */
uint16_t __callculate_current(uint32_t input){
	return input * SLOPE + ARZ_AZ_MABDA;
}

/**
  * @brief  Regular conversion complete callback
  * @param  current_ma is the estimated current value in mA unit.
  *         raw is the raw value of currnet between 0 to 4095. This value used in calibration process.
  * @retval None
  */
__weak void your_callback_name(uint16_t current_ma, uint32_t raw){
	UNUSED(0);
}
