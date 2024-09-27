#include "SIM8xx.h"

extern TIM_HandleTypeDef htim5;

void sim80x_PWR(uint8_t state){
	if(state == ON)
	{
		if(HAL_GPIO_ReadPin(_STATUS_PORT,_STATUS_PIN) == 0)
		{
			HAL_GPIO_WritePin(_POWER_PORT,_POWER_PIN,GPIO_PIN_RESET);
			HAL_Delay(1200);
			HAL_GPIO_WritePin(_POWER_PORT,_POWER_PIN,GPIO_PIN_SET);
			HAL_Delay(3000);
			HAL_UART_Transmit(&debugUART,(uint8_t *)"\r\n*****SIM POWER IS ON*****\r\n",strlen("\r\n*****SIM POWER IS ON*****\r\n"),100);
			HAL_Delay(7000);
		}
	}
	
	if(state == OFF)
	{
		if(HAL_GPIO_ReadPin(_STATUS_PORT,_STATUS_PIN) == 1)
		{
			HAL_GPIO_WritePin(_POWER_PORT,_POWER_PIN,GPIO_PIN_RESET);
			HAL_Delay(1200);
			HAL_GPIO_WritePin(_POWER_PORT,_POWER_PIN,GPIO_PIN_SET);
			HAL_Delay(3000);
			HAL_UART_Transmit(&debugUART,(uint8_t *)"\r\n*****SIM POWER IS OFF*****\r\n",strlen("\r\n*****SIM POWER IS OFF*****\r\n"),100);
		}
	}
}
/**
  * @brief  Sending commands to SIM800
	* @param  ATCommand - e.g: "AT"
  * @param  Timeout in milliseconds
  * @retval HAL status
	* @result	SIM800 reponse stored in RxBuffer
  */
HAL_StatusTypeDef sim80x_ATC(char * ATCommand , uint32_t Timeout){
	memset(RxBuffer,NULL,size);	
	
	HAL_UART_Transmit(&simUART,(uint8_t *) ATCommand,strlen(ATCommand),Timeout);
	HAL_UART_Receive(&simUART,(uint8_t *)RxBuffer,size,Timeout);

	HAL_UART_Transmit(&debugUART,(uint8_t *) "***\r\n",5,100);
	HAL_UART_Transmit(&debugUART,(uint8_t *) RxBuffer,strlen(RxBuffer),1000);
	HAL_UART_Transmit(&debugUART,(uint8_t *) "***\r\n",5,100);
	
	if(HAL_UART_STATE_ERROR != HAL_UART_GetState(&simUART) && RxBuffer[1] != NULL )
	{
		if( strstr( (char *) RxBuffer, "OK" )!= NULL)				
		{
			ErrorCounter=0;
			return HAL_OK;
		}
		if( strstr( (char *) RxBuffer, "ERROR" ) != NULL)		
		{
			ErrorCounter++;
			return HAL_ERROR;
		}
	}
	else 
	{
		return HAL_TIMEOUT;
	}
}
/**
  * @brief  Checks the SIM800 state.
  * @param  Timeout in milliseconds.
  * @retval HAL status
  */
HAL_StatusTypeDef sim80x_ACK(uint32_t Timeout){	
	Sim80x_StatusTypeDef = sim80x_ATC("AT\r\n",Timeout);
	return Sim80x_StatusTypeDef;
}
/**
  * @brief  Sends SMS to a specific phone number
  * @param  phoneNumber is a string containing the phone number e.g "+98912xxxxxxx"
	* @param  msg is the message string e.g "Hello :)"
  * @param  Timeout in milliseconds
  * @retval HAL status
  */
/*
HAL_StatusTypeDef sim80x_SendSMS(char * phoneNumber , char * msg,uint32_t Timeout){
	Sim80x_StatusTypeDef = sim80x_ATC("AT+CMGF=1\r\n",Timeout);
	if (Sim80x_StatusTypeDef == HAL_OK)
	{
		
		Sim80x_StatusTypeDef = sim80x_ATC("AT+CSMP=17,167,0,0\r\n",Timeout);
		
		memset(str,NULL,size);
		snprintf(str,sizeof(str),"AT+CMGS=\"%s\"\r\n",phoneNumber);
		Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
		if (Sim80x_StatusTypeDef == HAL_OK)
		{
			memset(str,NULL,size);
			//snprintf(str,sizeof(str),"%s%c\n\r",msg,(char)26);
			snprintf(str,sizeof(str),"%s",msg);
			Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
			
			memset(str,NULL,size);
			//snprintf(str,sizeof(str),"%s%c\n\r",msg,(char)26);
			snprintf(str,sizeof(str),"%c",26);
			Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
					
			return Sim80x_StatusTypeDef;
		}
		else 	
		{
			return Sim80x_StatusTypeDef;
		}
	}
	else	
	{
		return Sim80x_StatusTypeDef;
	}
}
*/

/**
  * @brief  Sends SMS to a specific phone number
  * @param  phoneNumber is a string containing the phone number e.g "+98912xxxxxxx"
	* @param  msg is the message string e.g "Hello :)"
  * @param  Timeout in milliseconds
  * @retval HAL status
  */
HAL_StatusTypeDef sim80x_SendSMS(char * phoneNumber , char * msg, uint32_t Timeout){
	memset(str,NULL,size);
	snprintf(str,size,"AT+CMGS=\"%s\"\r\n",phoneNumber);
	Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
	if (Sim80x_StatusTypeDef == HAL_OK)
	{
		memset(str,NULL,size);
		snprintf(str,size,"%s",msg);
		Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
		
		memset(str,NULL,size);
		snprintf(str,size,"%c",26);
		Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
				
		return Sim80x_StatusTypeDef;
	}
	else       
	{
		return Sim80x_StatusTypeDef;
	}
}

/**
  * @brief  Starts an HTTP protocol
  * @retval HAL status
  */
HAL_StatusTypeDef sim80x_HTTP_Start(void){
	sim80x_ATC("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n",100); //1000
	sim80x_ATC("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n",300); //1000
	sim80x_ATC("AT+SAPBR=1,1\r\n",7000); //3000
	if (sim80x_ATC("AT+SAPBR=2,1\r\n",300) != HAL_OK)
	{
		simCardGprsOk = 0;
		return HAL_ERROR;
	}
	if (sim80x_ATC("AT+HTTPINIT\r\n",100) != HAL_OK) //500
	{
		simCardGprsOk = 0;
		return HAL_ERROR;
	}
	if (sim80x_ATC("AT+HTTPPARA=\"CID\",1\r\n",100) != HAL_OK) //500
	{
		simCardGprsOk = 0;
		return HAL_ERROR;
	}
	simCardGprsOk = 1;
	return HAL_OK;
}

/**
  * @brief  Ends the HTTP connection
  * @retval HAL status
  */
HAL_StatusTypeDef	sim80x_HTTP_Stop(void){
	sim80x_ATC("AT+HTTPTERM\r\n",200);
	sim80x_ATC("AT+SAPBR=0,1\r\n",1000);
	if(strstr( (char *) RxBuffer, "+HTTPACTION: 1,601" ) != NULL)	//detect 601 error ,This error not happend after AT+HTTPACTION
	{
		DEBUG("\r\nPower OFF-ON Sim800");
		sim80x_PWR(OFF);
		sim80x_PWR(ON);
	}
	return Sim80x_StatusTypeDef;
}

/**
  * @brief  Sending a POST request
  * @param  postResult is a string containing the server respone of this request
	* @param  IP is a string containing the server IP e.g "192.168.1.1"
  * @param  URL is the api string e.g "api/panel/get_clock/"
	* @param  Content is the JSON that you want to post in server e.g "{"id":5,"value":45}" 
  */
HAL_StatusTypeDef sim80x_HTTP_Post(char* postResult, char * IP, char * URL , char * Content){
		//sim80x_HTTP_Start();
		
		memset(str,NULL,size);
		snprintf(str,size,"AT+HTTPPARA=\"URL\",\"%s/%s\"\r\n",IP, URL);  //api/multiSensor/
		if(sim80x_ATC(str,100) != HAL_OK)
		{
			DEBUG("\r\nPost unsuccessful\r\n");
			return HAL_ERROR;
		}
		
		if(sim80x_ATC("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n",100) != HAL_OK)
			{
			DEBUG("\r\nPost unsuccessful\r\n");
			return HAL_ERROR;
			}
			
		if(sim80x_ATC("AT+HTTPDATA=500,4000\r\n",2000) != HAL_OK)
			{
			DEBUG("\r\nPost unsuccessful\r\n");
			return HAL_ERROR;
			}
		
		sim80x_ATC(Content,200);
		HAL_Delay(4000);
			
		if(sim80x_ATC("AT+HTTPACTION=1\r\n",6000) != HAL_OK)
			{
			DEBUG("\r\nPost unsuccessful\r\n");
			return HAL_ERROR;
			}
		if(strstr( (char *) RxBuffer, "+HTTPACTION: 1,6" ) != NULL)	// detect 6xx series error
		{
			DEBUG("\r\nPower OFF-ON Sim800");				// Reset Sim800
			sim80x_PWR(OFF);
			sim80x_PWR(ON);
			return HAL_ERROR;
		}
			
		if(sim80x_ATC("AT+HTTPREAD\r\n",500) != HAL_OK)
			{
			DEBUG("\r\nPost unsuccessful\r\n");
			return HAL_ERROR;
			}

		memset(postResult, NULL, size);
	  memcpy(postResult, RxBuffer, size);
		return HAL_OK;
	}

/**
  * @brief  ?
  */
void SIM800_handler(void){
		
	// unplug simcard handler
	if(ErrorCounter > 10 )
	{
		ErrorCounter=0;
		sim80x_PWR(OFF);
		sim80x_PWR(ON);
		action=0;
	}
	
	// unplug sim800 handler
	if(HAL_GPIO_ReadPin(SIM_STATUS_GPIO_Port,SIM_STATUS_Pin) == 0)
	{
		HAL_GPIO_WritePin(SIM_PWR_GPIO_Port,SIM_PWR_Pin,GPIO_PIN_RESET);
		HAL_Delay(1200);
		HAL_GPIO_WritePin(SIM_PWR_GPIO_Port,SIM_PWR_Pin,GPIO_PIN_SET);
		HAL_Delay(3000);
		HAL_UART_Transmit(&debugUART,(uint8_t *)"*************************************SIM POWER IS ON******************************************\r\n",strlen("*************************************SIM POWER IS ON******************************************\r\n"),100);
		HAL_Delay(7000);
	}
}

/**
  * @brief  This function makes sure that SIM800 is ready
	* @retval Returns 1 in case of success, returns 0 in case of failure.
  */
uint8_t ACKHandler(void){
	uint8_t ACKCounter=0;
	uint8_t CycleCounter=0;
	for(CycleCounter=0; CycleCounter<3; CycleCounter++){
		sim80x_PWR(ON);
		for(ACKCounter=0;ACKCounter<10;ACKCounter++){
			sim80x_ACK(500);
			if(Sim80x_StatusTypeDef == HAL_OK)
				return 1;
		}
		sim80x_PWR(OFF);
		sim80x_PWR(ON);
	}
	return 0;
}


/**
  * @brief  Set the SMS settings
  */
void SMSSetting(void){
	sim80x_ATC("AT+CMGF=1\r\n",6000);
	sim80x_ATC("AT+CSMP=17,167,0,0\r\n",4000);
}
	
/**
  * @brief  Get RSSI value from sim800
*   
  */

void sim80x_Get_RSSI(){
	  char *startAnswer; 
	  char *endAnswer;
		Sim80x_StatusTypeDef=sim80x_ATC("AT+CSQ\r\n",50);// SIM800 RSSI AT Command
		if(Sim80x_StatusTypeDef==HAL_OK)
			{
			//parsing sim800 response:	
			startAnswer	= strstr(RxBuffer, "+CSQ:")+5;
			endAnswer		= strstr(RxBuffer, ",");
			for(int i=(startAnswer-RxBuffer); i<(endAnswer-RxBuffer); i++)
				rssiStrValue[i-(startAnswer-RxBuffer)] = RxBuffer[i];	//ContentStr=RSSI value of sim800
			rssiIntValue= (uint8_t)atoi(rssiStrValue);
			}
			else
			{				
				 DEBUG( "***\r\n");
				 DEBUG("RSSI: Can not read RSSI from sim80x\r\n");
				 rssiIntValue=0;
				 DEBUG( "***\r\n");
			}
		}
/**
  * @brief  Check SimCard Status
  */
HAL_StatusTypeDef	Check_SimCard(uint8_t Previous_State)
{		
		if(Previous_State == 0)
		{
		memset(str,NULL,size);
		snprintf(str,size,"AT+CFUN=1,1\r\n"); 	// Reset SIM800 Module
		sim80x_ATC(str,2000);
		HAL_Delay(5000);
		}
		memset(str,NULL,size);
//	snprintf(str,size,"AT+CPIN?\r\n");   // check with SimCard PinCode
		snprintf(str,size,"AT+CCID\r\n");    // check with SimCard Integrated Code
		if(sim80x_ATC(str,2000) == HAL_OK)
		{
			DEBUG("\r\nSIM is plugged\r\n");
			return HAL_OK;
		}
		else
			{
			DEBUG("\r\nSIM is not plugged\r\n");
			return HAL_ERROR;
			}
}
/**
  * @brief  Check whether it has registered in the network. (0,1) means registered
  */
HAL_StatusTypeDef	Check_Network_Registered(uint8_t Previous_State)
{		
if(Previous_State == 0)
		{
		memset(str,NULL,size);
		snprintf(str,size,"AT+CFUN=1,1\r\n"); 	// Reset SIM800 Module
		sim80x_ATC(str,2000);
		HAL_Delay(5000);
		}
		memset(str,NULL,size);
		snprintf(str,size,"AT+CREG?\r\n"); 	// Check Network Registeration
		sim80x_ATC(str,2000);
		if((strstr((char *)RxBuffer, "0,1") != NULL) || (strstr((char *)RxBuffer, "0,2") != NULL))
		{
			DEBUG("\r\nRegistered On network\r\n");
			return HAL_OK;
		}
		else
		{
			DEBUG("\r\nNot Registered On network\r\n");
			return HAL_ERROR;
		}
}