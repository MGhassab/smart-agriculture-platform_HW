#include "main.h"
#include <stdlib.h>
#include "stdio.h" 
#include "string.h"
#include "usart.h"
#include "gpio.h"

#ifndef SIM8XX_
#define SIM8XX_

/* Definitions */

// POWER GPIO:
#ifndef _POWER_PORT
#define _POWER_PORT	SIM_PWR_GPIO_Port
#endif

#ifndef _POWER_PIN
#define _POWER_PIN	SIM_PWR_Pin
#endif

// STATUS GPIO
#ifndef _STATUS_PORT
#define _STATUS_PORT	SIM_STATUS_GPIO_Port
#endif

#ifndef _STATUS_PIN
#define _STATUS_PIN	SIM_STATUS_Pin
#endif


// NET GPIO
#ifndef _NET_PORT
#define _NET_PORT	SIM_NET_GPIO_Port
#endif

#ifndef _NET_PIN
#define _NET_PIN	SIM_NET_Pin
#endif

// SIM8xx UART
#define simUART huart1
//extern UART_HandleTypeDef simUART;

// Debug UART
#define debugUART huart4
//extern UART_HandleTypeDef debugUART;

//----------------------------------------------
typedef enum 
{
  OFF       = 0x00U,
  ON    		= 0x01U,
} OUTPUT_StatusTypeDef;


// Global vars
#define										size					3700			// Size of SIM800 string buffer.
extern char								str[size],ContentStr[size];
extern char								RxBuffer[size];
extern char 							rssiStrValue[5];
extern uint8_t						ErrorCounter;
extern uint8_t						action;
//extern uint8_t		        buttonsStatus[BUTTONS_NUM];			//status of the 2 external buttons 
extern uint8_t 						simCardGprsOk;
extern uint8_t 						rssiIntValue;						//include RSSI intiger value .
extern HAL_StatusTypeDef	Sim80x_StatusTypeDef;
/* Functions */
HAL_StatusTypeDef sim80x_ATC(char * ATCommand , uint32_t Timeout);
HAL_StatusTypeDef sim80x_ACK(uint32_t timeOut);
HAL_StatusTypeDef sim80x_SendSMS(char * phoneNumber , char * msg , uint32_t Timeout);
HAL_StatusTypeDef	sim80x_HTTP_Start(void);
HAL_StatusTypeDef	sim80x_HTTP_Stop(void);
HAL_StatusTypeDef sim80x_HTTP_Post(char* postResult, char * IP, char * URL , char * Content);
HAL_StatusTypeDef	Check_SimCard(uint8_t Previous_State);
HAL_StatusTypeDef	Check_Network_Registered(uint8_t Previous_State);
void 							sim80x_Get_RSSI();
void							sim80x_PWR(uint8_t state);
void 							SIM800_handler(void);
uint8_t						ACKHandler(void);
void							SMSSetting(void);
void 							sim80x_Get_RSSI();

#endif 
