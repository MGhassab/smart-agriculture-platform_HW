/**
  ******************************************************************************
  * @file    AM2305.h
  * @author  M.H Nikkhah
  * @brief   Source file of AM2305 Temperature & Humidity module.
  ******************************************************************************
  * @attention
  *	Version : 1.1
	*	Help:
	*	Please set the Gpio port and pin in  Definitions according to your hardware.
	* You must first adjust the clock settings of GPIO in the main function.
  ******************************************************************************
  */ 

//#include "stm32f4xx_hal.h"          // change to whatever MCU you use
#ifndef __MAIN_H
#include "main.h"
#endif
#include "am2305.h"


GPIO_InitTypeDef GPIO_InitStruct = {0};


/**
  * @brief Start AM2305 Function. this function is for make sensor 
					 be ready for fetching data. 	
  * @param None
  * @retval uint8_t response , 
		1 : response ok if the reterned value of the function is 1 means 
		that the sensor is ready for communication
		0 : response not ok
  */
uint8_t AM2305_IsReady(void)
{
	uint8_t check=0;
	uint32_t	timeStart=0;
	
	HAL_GPIO_WritePin(AM2305_PORT, AM2305_PIN, GPIO_PIN_SET);
	
	// set data pin as output
	GPIO_InitStruct.Pin = AM2305_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	//GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AM2305_PORT, &GPIO_InitStruct);
	
	
	// send start signal to sensor from micrcontroller
	HAL_GPIO_WritePin(AM2305_PORT,AM2305_PIN,GPIO_PIN_RESET);
 
	DWT_Delay(1000);
	//HAL_Delay(10);
	HAL_GPIO_WritePin(AM2305_PORT,AM2305_PIN,GPIO_PIN_SET);
	DWT_Delay(30);
	
	
	// set data pin as input
  GPIO_InitStruct.Pin = AM2305_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AM2305_PORT, &GPIO_InitStruct);
		
		
	//wait until data pin set as low
	timeStart=HAL_GetTick();
	while(HAL_GPIO_ReadPin(AM2305_PORT,AM2305_PIN))
	{
		if( (HAL_GetTick()-timeStart) > 20)
			break;
	}

	
	if(!(HAL_GPIO_ReadPin(AM2305_PORT,AM2305_PIN)))
	{
		DWT_Delay(80);
		if(HAL_GPIO_ReadPin(AM2305_PORT,AM2305_PIN))
			check=1;
	}
	// wait for the pin to go low
	timeStart=HAL_GetTick();
	while(HAL_GPIO_ReadPin(AM2305_PORT,AM2305_PIN))
	{
		if( (HAL_GetTick()-timeStart) > 20)
			break;
	}
	
	return check;
}


/**
  * @brief GetByte AM2305 Function. Receive 1 byte data from sensor
  * @param None
  * @retval uint8_t i , 1byte data 
  */
uint8_t AM2305_GetByte(void)
{
	uint32_t timeStart=0;
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		// wait for the pin to go high
		timeStart=HAL_GetTick();
		while (!(HAL_GPIO_ReadPin (AM2305_PORT,AM2305_PIN)))
		{
			if( (HAL_GetTick()-timeStart) > 20)
				break;
		}			
		DWT_Delay (40);   
		// if the pin is low
		if (!(HAL_GPIO_ReadPin (AM2305_PORT,AM2305_PIN)))   
		{
			// write 0
			i&= ~(1<<(7-j));   
		}
		// if the pin is high, write 1
		else i|= (1<<(7-j));  
		// wait for the pin to go low
		timeStart=HAL_GetTick();
		while ((HAL_GPIO_ReadPin (AM2305_PORT,AM2305_PIN))) 
		{
			if( (HAL_GetTick()-timeStart) > 20)
				break;
		}
	}
	return i;
}

/**
  * @brief GetData AM2305 Function. get all byte of data in each communication itteration
					 and preapare data for end user.
					 send data with uart
  * @param None
  * @retval float pointer. contains 2 float data type.
	          the first one is humidity value and the second one is temperature value 
  */
float * AM2305_GetData(void)
{
	
	static float DATA[3];
	
	uint8_t HUMIDITY_byte1,HUMIDITY_byte2,TEMPERATURE_byte1,TEMPERATURE_byte2;
	int16_t HUMIDITY,TEMPERATURE;


	// check the statuse of sensor
	if (AM2305_IsReady())
		{	
			
			// get data byte by byte
			HUMIDITY_byte1		=AM2305_GetByte();
			HUMIDITY_byte2		=AM2305_GetByte();
			TEMPERATURE_byte1	=AM2305_GetByte();
			TEMPERATURE_byte2	=AM2305_GetByte();
			AM2305_GetByte();
			
			// merge byte together to access temp & humid bin type 
			HUMIDITY=(HUMIDITY_byte1<<8)|HUMIDITY_byte2;
			TEMPERATURE=(TEMPERATURE_byte1<<8)|TEMPERATURE_byte2;
			
			// calculate the sign of temp value and convert data to float type
		  DATA[0]=HUMIDITY*0.1;
			if (TEMPERATURE & 0x8000)	DATA[1]=(TEMPERATURE & 0x7fff)*-0.1;
			else 											DATA[1]=TEMPERATURE*0.1;
			DATA[2]=0;
		}
		else	DATA[2]=0xFFFF;
			
	return DATA;
}
