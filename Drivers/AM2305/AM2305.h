/**
  ******************************************************************************
  * @file    AM2305.h
  * @author  M.H Nikkhah
  * @brief   Header file of AM2305 Temperature & Humidity module.
  ******************************************************************************
  * @attention
  *	Version : 1.1
	*	Help:
	*	Please set the Gpio port and pin in  Definitions according to your hardware.
	* You must first adjust the clock settings of GPIO in the main function.
  ******************************************************************************
  */ 


#ifndef AM2305_
#define AM2305_

/* Definitions */
#ifndef AM2305_PORT
#define AM2305_PORT	GPIOA
#endif

#ifndef AM2305_PIN
#define AM2305_PIN	GPIO_PIN_5
#endif

/* Includes */
#include "dwt_delay.h"
#include "core_cm4.h"



/**
  * @brief Start AM2305 Function. this function is for make sensor 
					 be ready for fetching data. 	
  * @param None
  * @retval uint8_t response , 
		1 : response ok if the reterned value of the function is 1 means 
		that the sensor is ready for communication
		0 : response not ok
  */
uint8_t AM2305_IsReady(void);

/**
  * @brief GetByte AM2305 Function. Receive 1 byte data from sensor
  * @param None
  * @retval uint8_t i , 1byte data 
  */
uint8_t AM2305_GetByte(void);

/**
  * @brief GetData AM2305 Function. get all byte of data in each communication itteration
					 and preapare data for end user.
					 send data with uart
  * @param None
  * @retval float pointer. contains 2 float data type.
	          the first one is humidity value and the second one is temperature value 
  */
float * AM2305_GetData(void);
#endif 
