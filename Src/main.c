/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cJSON.h"
#include <time.h>
#include "dwt_delay.h"
#include "AM2305.h"
#include "time.h"
#include "ee24.h"
#include "SIM8xx.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "current.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct program_t{
/* 0 */	uint8_t ID;
/* 1 */	uint8_t Month;
/* 2 */	uint8_t Day;
/* 3 */	uint8_t Hour;
/* 4 */	uint8_t Minutes;
/* 5 */	uint8_t PeriodDay;
/* 6 */	uint8_t PeriodHour; 
/* 7 */	uint8_t PeriodMinute;
/* 8 */	uint8_t Action[2]; ///////////NEW ADDED/////////////////
/* 9 */	uint8_t has_interval;
} Program;
typedef struct ProcessProgram_t{
/* 0 */	uint8_t  ID;
/* 1 */	uint32_t duration[10];	
/* 2 */	uint8_t  Hour[10];
/* 3 */	uint8_t  Minutes[10];
/* 4 */	uint8_t  Second[10];	
/* 5 */	uint8_t  spouts[10][2]; ///////////NEW ADDED/////////////////
/* 6 */	uint8_t  events_count;	
}
ProcessProgram;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// DEVICE NAME: "ABGIRI"

#define		__SENSORS_ARE_ENABLE__		0																						// If you want to use sensors, this value should be 1.
#define		__BUTTONS_ARE_ENABLE__		1																						// If you want to use external buttons, this value should be 1.
#define		__IS_3G_MODULE__					0																						// If you use 3G module this value should be 1.
#define		__INITIALLIGHT__					0	
#define		__IS_OLED__								0																						// If you use OLED this value should be 1.
#define  	__SERIAL_NUMBER						"12121212001"																// Unique serial number. We use this number to get "land ID" from server.12
#define		__WELCOME_TEXT						"WELCOME"																		// The text sent via SMS after reset.
#define		__ON_OFF_CURRENT_THRESH__	1000	 																			// If the load current (mA) exceeded this limit, it means the load in ON 		

///////////NEW ADDED/////////////////

#define   EE24_WRITE_COEFFICIENT_PROG		 20																		  // a COEFFICIENT used in addresing eeprom
#define   EE24_WRITE_COEFFICIENT_PROS		 100
#define		EE24_START_ADDRESS_PROCESS_PROGRAM 1000	
// The starting address of process programs in eeprom
#define		EE24_START_ADDRESS_PROGRAM 	100																				// The starting address of programs in eeprom
#define		MAX_NUM_PROGRAM 				30
#define		MAX_NUM_PROCESS_EVENT 	10
///////////////////////////////////////

#define		LAST_SYSTEM_RESET_STATUS	  9																					// The address of RTC backup register that using in reset system one time before starting 
#define		PROGRAM_ID_ADDRESS				 19																					// The address of RTC backup register storing the next program ID.
#define		LAST_STATUS_ADDRESS_LSB    15																					// The address of RTC backup register storing the last output status.
#define		LAST_STATUS_ADDRESS_MSB    16																					// The address of RTC backup register storing the last output status.
#define		MY_ID_ADDRESS							 18																					// The address of RTC backup register storing the "Land ID".
#define		PROCESS_PROGRAM_ID_ADDRESS 11																					// The address of RTC backup register storing the last process program ID.
#define		LAST_EVENT_NUMBER_ADDRESS  12																					// The address of RTC backup register storing the last event number of process program .
#define		LAST_CURRENT_TIME_STAMP		 13																					// The address of RTC backup register storing the last current time stamp.
#define		LAST_PROGRAMS_FLAG_STATUS	 14																					// The address of RTC backup register storing the last PROGRAMS flag status.
#define   LAST_EVENT_TIME_STAMP			 17																					// The address of RTC backup register storing the last event time stamp.
//#define		PROCESS_PROG_START_ADD		1000																				// The starting address of process programs in eeprom
#define		debugUART									huart4																			// Debug uart handler
#define		simUART										huart1																			// SIM800 uart handler





#define		SERVER_IP									"baghyar.darkube.app/panel/api/d"										// Server domain
#define 	phone											"+989140436272"	

#define 	U_ID       						    0x1FFF7A10 																	// Unique device ID register address
#define		nmbr_try_connect_site     5																						// Specify how many try to connect to the site
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct sTime_t{
/* 0 */	uint16_t year[BUTTONS_NUM];
/* 1 */	uint16_t  month[BUTTONS_NUM];	
/* 2 */	uint16_t  day[BUTTONS_NUM];
/* 3 */	uint16_t  hours[BUTTONS_NUM];
/* 4 */	uint16_t  minutes[BUTTONS_NUM];	
/* 5 */	uint16_t  seconds[BUTTONS_NUM]; 
}
sTime;

uint8_t									isConnect=1;										// Determines the connection status. 1 means connected
uint16_t								deviceId ;											// This is the device ID
uint16_t								landId ;												// This is the landId 
char										ContentStr[size],str[size],oledStr[size];			// Two general strings used in some functions as string buffers
char										SMStext[120];										// The text we want to send
uint32_t								counter;												// Used in flow meter
uint8_t				shomare=0;
RTC_TimeTypeDef					Time;														// Stores time
RTC_DateTypeDef					Date;														// Stores date
RTC_AlarmTypeDef				alarm_struct_A;									// Stores alarm setting
RTC_AlarmTypeDef				alarm_struct_B;									// Stores alarm setting
uint8_t									action = 1;											// This is a flag showing whether we want to send requests to server or not. This value is being changed every 3 minutes

float * 								TH;															// temp & humidity

uint32_t								AnalogSensors[3];								// ADC coupled DMA array
uint32_t								moisture[10];										// Moisture value

uint32_t								Flow1,Flow2;										// Flow values

uint8_t 								averageItterator=0;							// The itteration variable for averaging method

HAL_StatusTypeDef				Sim80x_StatusTypeDef;
uint8_t									ErrorCounter=0;									// Counts the errors returned by SIM800
char 						    		RxBuffer[size];									// A buffer for SIM800's response
uint8_t                 buttonsStatus[BUTTONS_NUM];			//status of the external buttons(set zero this array in user_code_begin2) 
char 										outputsStatusFeedBack[OUTPUTS_NUM+1];// used in "GetStatus" 
char  						  		rssiStrValue[5];								// include RSSI string value .
char *									startAnswer; 										//used in getting rssi antenna in initial section
char *									endAnswer;											//used in getting rssi antenna in initial section
char*								  	registerPassword="123456";
char								  	phoneNumber[10];
char	 									smsText[size];
uint8_t 						  	rssiIntValue=0;									//include RSSI intiger value .
volatile uint8_t				SIM800_Status[5] = {1,1,1,1,1}; // [0] -> SimCard previous Status		[1] -> NetWork Status
char										processProgram[size];						//used in processProgramParser

uint8_t 								LastStatus[2];									// Stores the last output status
uint8_t 								NewStatus[2];										//output status that Extracted from server response in GetOutput()
uint8_t 								get_output_result;							// The result of GetOutput fnc
uint8_t									SMSisPending=0;									// It is a flag showing whether we want to send SMS or not
uint8_t									AlarmIsSet = 0;									// Shows the status of alarm. 1 means an alarm was set
uint8_t   							processProgramsAlarmIsSet=0;		//used when we want enable/disable processprogramsAlarm
uint8_t  							  eventNumber[BUTTONS_NUM];				//Running process event number  for each buttons
uint32_t								eventsTurn[BUTTONS_NUM];				//The closest eventTimeStamp for each buttons 
uint32_t								nextEventTimeStamp=0;						//time stamp of next event for which the alarm should be set 
volatile uint8_t 				buttonId;												//it is the extenal botton id that is pushed
volatile uint8_t 				CheckAlarm_buttonId;						//used for check AlarmB is for which button
uint8_t									unfinishedEventFlag=0;					//It is set to 1 if we have an unfinished event 
uint8_t 								getProcessProgramsStatus=0;			//Getting process program  unsuccessful /successful  flag. used in external bottons & Getting process program section in oled
uint8_t 								getProgramsStatus=0;						//Getting  program  unsuccessful /successful  flag used in Getting  program section in oled
uint8_t									initializingFlag=0;							//set to 1 when initial setting started and then set to 0 when it done and remain 0 in all program
uint16_t								tim5CallbackCounter=0;					//a counter that used in HAL_TIM_PeriodElapsedCallback function
uint8_t									dotPointCounter=0;							// count dotpoint in initializing in HAL_TIM_PeriodElapsedCallback function
uint8_t 								blinker=0;											//used in blinking in server conection section in oled.
uint8_t									oledState=0;										// oled screen is a FSM and oledState is including its state.
uint16_t								lastTim5CallbackCounter=0;			//used to save last time in every state of oled screen
uint8_t									getProcessProgramsStarting=0;		//1 if we start to get procees programs used in showing on oled and 0 if get procees programs to be ended. 
uint8_t									getProgramsStarting=0;					//1 if we start to get  programs used in showing on oled 0 if get programs to be ended. 
uint32_t								lastTimeStamp;									//used when read last current time  stamp from eeprom
uint32_t								lastEventTimeStamp;							//used when read last event time  stamp from eeprom
uint32_t								changeButtonStatusTimeStamp[BUTTONS_NUM][2]; //used for //save last last time and last status of the button its status have been changed in  SetProcessProgramsAlarm;
uint8_t									isAbGiriInProgress = 1;					// Represent whether we want to do Abgiri or not.
uint8_t									isTimeForCurrentCheck = 1;			// Represent whether we check the load current or not.
uint8_t									Load_Status = 0;								// If load (e.g. pomp) is powered, this variable will be 1, otherwise 0.
uint8_t									Load_NumberOfTries = 0;					// Number of attemots to turning the load on.
uint8_t									simCardGprsOk=0;								// simcard gprs is ok. changed in sim80x_HTTP_Start() and used in rssi antenna
uint8_t									systemResetFlag;                // used for reset the system for one time After the device is turned on
sTime										buttonsLastTime;								// last time changing in buttons status used in TimestampTOdate

volatile bool						Alarm_A_occur = 0;											// This flag Set True in Alarm_A Interrupt
volatile bool						Alarm_B_occur[BUTTONS_NUM];							// This flag Set True in Alarm_B Interrupt
volatile bool						GetStatus_Is_Success = 0;								// This flag used for determine GetStatus function is successful or not
volatile bool 					GetProcessProgram_IS_Success = 0;			  // This flag used for determine GetProcessProgram function is successful or not
volatile bool						Output_Actor[OUTPUTS_NUM];							// This flag determin who set output state last time and used in priority between Program and proccess profram
uint8_t 								Buttons_num = BUTTONS_NUM;							// assign button number from macro to variables
volatile uint8_t				Button_Actor[BUTTONS_NUM];							// This flag determin who push button(0-> Server		1-> device)
char 										action_str[35];
uint16_t 								decimalOutputs;
uint8_t  								outputsStatus[OUTPUTS_NUM];
char  									sendStatus[size];// a JSON including output status, RSSI value , external bottons status that post to server
char 										ebcList[size];
char  									ebc_lc[size];
char  									ebcStatus[20];
volatile char  					programLastTime[50];
uint8_t 								OutputIsChanged=0;
uint8_t 								ProgramIsChanged=0;
uint8_t 								proccesProgramIsChanged=0;
volatile char						programButtonLastTime[BUTTONS_NUM][20];	// used for getting Last NewProgram Time for Buttons from server in GetStatus function
volatile char						ButtonCondition[BUTTONS_NUM][2];				// used for getting Button_NewProgram Conditioon from server in GetStatus function
char										ServerButtonState[BUTTONS_NUM];					// used for getting Button State from server in GetStatus function 
volatile uint16_t 			getStatusUnSuccessCounter = 0;					// counting GetStatus() unsuccess result to try 3 time this functin in while(acion)
	
char 										RID[5];													// Read ID from eeprom(used for debug)
char 										RID_land[5];										// Read Land_ID from eeprom(used for debug)
char 										serial_number_string[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; // Unique serial number. We use this number to get "ID" from server.
uint32_t 								unique_device_ID[3];						// 96 bit micro controller unique id
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void							my_delay(uint32_t interval);
uint16_t					GetID(void);
uint8_t						GetOutput(void);
uint8_t 					updateNextEvent();
void							measureAndLog(void);
void 							postSensors(void);		
uint8_t						str2bin(char* input);
void							writeProg(Program* prog);
void 							writeProcessProg(ProcessProgram* processProg);
Program						readProg(uint8_t progID);
ProcessProgram    readProcessProg(uint8_t progID);
void							deleteAllPrograms(void);
void							deleteAllProcessPrograms(void);
void							deleteProg(Program* prog);
void 							GetAllPrograms(void);
uint8_t						GetAllProcessPrograms(void);
void							ApplyAction(uint8_t OutputStatus[2]);
Program						ProgramParser(char* input_str);
ProcessProgram    processProgramParser(char* input_str);
void							PrintProgram(Program input);
void							PrintAllPrograms(void);
void 							PrintAllProcessProgram(int i);
void							SetAlarm(RTC_HandleTypeDef* rtc);
uint8_t 					SetProcessProgramsAlarm(RTC_HandleTypeDef* rtc);
void							SetNextAlarm(RTC_HandleTypeDef* rtc);
void 							SetNextAlarm_Processprograms(RTC_HandleTypeDef* rtc);
void							updatePrograms(void);
	
uint8_t						JSON2Str(char* result, char* raw_input, char* key);
uint8_t						JSON2int(char* result, char* raw_input, char* key);
uint8_t						JSON2Str_nested(char* result, char* raw_input, char* key_parent, char* key_child);
void							current_sensor_CallBack(uint16_t current_ma, uint32_t raw);
void 							AbGiriProccess(void);
uint8_t 					CheckingPhonenumber(char * phone_number );
void 							GET_SAVE_Time(void);
void						  Get_SAVE_ID(void);
void 							TimestampTOdate(long int seconds ,uint8_t butID );
uint8_t 					GetStatus(char* response);
uint16_t          str2dec(char* input);
uint16_t 					power(uint8_t a,uint8_t p);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	TH = malloc(2*sizeof(float));
	DWT_Init();
	


	if(HAL_RTCEx_BKUPRead(&hrtc,LAST_SYSTEM_RESET_STATUS)==1)//reset system one time before starting
{
	HAL_RTCEx_BKUPWrite(&hrtc, LAST_SYSTEM_RESET_STATUS, 2);
	// Write dafault time in program last check memory in eeprom for get true answer from server in first time
//	snprintf(programButtonLastTime[0],20,"12/08/21,16:52:40");
//	ee24_write(&hi2c1,10,(uint8_t *)programButtonLastTime[0],20,100);	// last program time
//	ee24_write(&hi2c1,40,(uint8_t *)programButtonLastTime[0],20,100);	// button 1 last program time
//	ee24_write(&hi2c1,60,(uint8_t *)programButtonLastTime[0],20,100);	// button 2 last program time
//	ee24_write(&hi2c1,80,(uint8_t *)programButtonLastTime[0],20,100); // button 3 last program time
	

	
	NVIC_SystemReset();
	
}

	HAL_RTCEx_BKUPWrite(&hrtc, LAST_SYSTEM_RESET_STATUS, 1);
	DEBUG("\n\r************************************************");
	DEBUG("\n\r*              Baghyar                         *");
	DEBUG("\n\r************************************************");
	DEBUG("\n\r*   Version      :   P200                      *");
	DEBUG("\n\r*   Programed By :   Dr. Taheri                *");
	DEBUG("\n\r*                    Mahdi Ghassab             *");
	DEBUG("\n\r*                                              *");
	DEBUG("\n\r************************************************\n\r");
	

	initializingFlag=1; //initializing seting is starting
  //*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|                 Tim start IT                  |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");

		#if(__IS_OLED__==1)
			HAL_Delay(250);
			ssd1306_Init();
			HAL_Delay(250);
			HAL_TIM_Base_Start_IT(&htim5);
		#endif
		HAL_TIM_Base_Start_IT(&htim5);
		HAL_TIM_Base_Start_IT(&htim8);
	DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
	

	/*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|           Starting current sensor             |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		current_start(current_sensor_CallBack, &htim3, &hadc2);
	  	DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
	
	//*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|         SET OUTPUTS STATUS                     |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		HAL_Delay(500);
		LastStatus[0]=0;
		LastStatus[1]=0;
//		if(HAL_RTCEx_BKUPRead(&hrtc, LAST_PROGRAMS_FLAG_STATUS)==1){ 
//			//we have unfinished program
//			LastStatus[0] =	HAL_RTCEx_BKUPRead(&hrtc, LAST_STATUS_ADDRESS_LSB);
//			LastStatus[1] = HAL_RTCEx_BKUPRead(&hrtc, LAST_STATUS_ADDRESS_MSB);	
//		}
		ApplyAction(LastStatus);
  	DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
//	
//	/*
//	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
//	DEBUG(						"|     READING FLOW1 AND FLOW2 FROM MEMORY       |");
//	DEBUG(				"\r\n-------------------------------------------------\r\n");
//		HAL_Delay(500);
//		Flow1=HAL_RTCEx_BKUPRead(&hrtc,1);	
//		Flow2=HAL_RTCEx_BKUPRead(&hrtc,2);	
//	  DEBUG("\n\r                 -----DONE-----                  \n\r");
//	//*/
//	
	//*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|           ACTIVATING SIM808 MODULE            |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		HAL_Delay(500);
		isConnect = 1;
		if(!ACKHandler()){
			isConnect = 0;
			DEBUG("\n\r SIM800 is not responding. I'm Offline now.");
		}
		
		
  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
		
	//*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|        Check Simcard & NetWork Status         |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		memset(str,NULL,size);
		snprintf(str,size,"AT+CFUN=1,1\r\n"); 	// Reset SIM800 Module
		sim80x_ATC(str,2000);
		HAL_Delay(5000);
		while(1)		// Check SimCard
		{
			if(Check_SimCard(SIM800_Status[0]) == HAL_OK)
			{
				SIM800_Status[0] = 1;
				break;
			}
			else
				SIM800_Status[0] = 0;
		}
		
		
		while(1)		// Check NetWork
		{
			if(Check_Network_Registered(SIM800_Status[1]) == HAL_OK)
				{
				SIM800_Status[1] = 1;
				break;
				}
			else
				SIM800_Status[1] = 0;
		}
	DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/		

		
	
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|             GETTING RSSI ANTENNA              |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		
    sim80x_Get_RSSI();

  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
	
			
	
//	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
//	DEBUG(						"|                  SMS SETTING                  |");
//	DEBUG(				"\r\n-------------------------------------------------\r\n");
//		SMSSetting();
//  DEBUG("\n\r                 -----DONE-----                  \n\r");
//	//*/
//	
//	 
//	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
//	DEBUG(						"|            Deletting All Messages             |");
//	DEBUG(				"\r\n-------------------------------------------------\r\n");
//		sim80x_ATC("AT+CMGD=1,4\r\n",2000);	
//  DEBUG("\n\r                 -----DONE-----                  \n\r");
//	//*/
//	
	/*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|                 SENDING SMS                   |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		HAL_Delay(500);
		sim80x_SendSMS(phone,__WELCOME_TEXT,6000);
  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
	
	
	//*
	
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|                 Setting Buttons                |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
	uint32_t 							currentTimeStamp = 0;		//used when convert current time to time stamp
	struct tm  						currentTime;
	Alarm_A_occur = 0;
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	currentTime.tm_year	= Date.Year + 2000  - 1900;
	currentTime.tm_mon		= Date.Month-1;
	currentTime.tm_mday	= Date.Date;
	currentTime.tm_hour	= Time.Hours;
	currentTime.tm_min		= Time.Minutes;
	currentTime.tm_sec		= Time.Seconds;
	currentTime.tm_isdst	= -1;	
	currentTimeStamp = mktime(&currentTime);//convert current time to time stamp	
	memset(eventNumber,0,BUTTONS_NUM);
	memset(eventsTurn,0,BUTTONS_NUM);
	memset(Alarm_B_occur,0,BUTTONS_NUM);
	for(uint8_t i=0;i<BUTTONS_NUM;i++)
	{
		buttonsStatus[buttonId]=0;
		changeButtonStatusTimeStamp[i][0]=currentTimeStamp;
		changeButtonStatusTimeStamp[i][1]=buttonsStatus[buttonId];
		TimestampTOdate(currentTimeStamp,i);
	}
	memset(str,NULL,size);
	sprintf(str,"\n\r **Buttons Status: %d%d \n\r",buttonsStatus[1],buttonsStatus[0]);
	DEBUG(str);
	memset(ContentStr,NULL,size);
	snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d   \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
	DEBUG(ContentStr);	

  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
		
		
//*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|                   STOPING HTTP                |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		sim80x_HTTP_Stop();
  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
		
		
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|                 STARTING HTTP                 |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		for(uint8_t	try_cntr = 0; try_cntr < nmbr_try_connect_site; try_cntr++)
		{
			if(sim80x_HTTP_Start() == HAL_OK)
				break;
		}
  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
		
		
	//*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|             GETTING ID & LAND_ID              |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		Get_SAVE_ID();
		ee24_read(&hi2c1,0,(uint8_t *)RID,5,100);				// Read ID froem EEPROM
		ee24_read(&hi2c1,5,(uint8_t *)RID_land,5,100);	// Read Land_ID froem EEPROM
		deviceId = (uint16_t)atoi(RID);			// Save Device ID in deviceId variable
		landId = (uint16_t)atoi(RID_land);	// Save Land ID in landId variable	
  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/



  //*	
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|       GETTING ALL PROGRAMS FROM SERVER        |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		HAL_Delay(500);
		if(isConnect==1){
			GetAllPrograms();
			PrintAllPrograms();
			AlarmIsSet = 0;
		}	
  DEBUG("\n\r                 -----DONE-----                  \n\r");
	HAL_Delay(6000);		
	//*/

	//*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|              SETTING NEXT ALARM               |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		HAL_Delay(500);
		SetNextAlarm(&hrtc);
  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/

	//*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|    GETTING ALL PROCESS PROGRAMS FROM SERVER   |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		HAL_Delay(500);
		if(isConnect==1)
	 {
			if( GetAllProcessPrograms()) // get all process programs: return 1 if is successful and return 0 is unsuccessful
				DEBUG("GetAllProcessPrograms is successful ");
			else
				DEBUG("ERROR in GetAllProcessPrograms ");
		}	
  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
	
	//*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|             GETTING DATE AND TIME             |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
	GET_SAVE_Time();
  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/	
	
		

	
	//*
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|                   STOPING HTTP                |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
		sim80x_HTTP_Stop();
  DEBUG("\n\r                 -----DONE-----                  \n\r");
	//*/
	
	HAL_Delay(1000);
	initializingFlag=0; //initializing seting done	
  HAL_TIM_Base_Stop_IT(&htim5);
	HAL_GPIO_WritePin(SW_Light3_GPIO_Port,SW_Light3_Pin,1);
	DEBUG("\n\r-----------<<< INITIALIZING DONE >>>-----------\n\r");
	HAL_Delay(1000);
	
	
	
	snprintf(programButtonLastTime[0],20,"12/08/20,16:52:40");
	snprintf(ButtonCondition[0],1,"0");
	
	snprintf(programButtonLastTime[1],20,"12/08/20,16:52:40");
	snprintf(ButtonCondition[1],1,"0");
	
	snprintf(programButtonLastTime[2],20,"12/08/20,16:52:40");	
	snprintf(ButtonCondition[2],1,"0");

	snprintf(programLastTime,50,"12/08/20,16:52:40");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Checking for sim800 errors:
		SIM800_handler();
		
		// Sending SMS in case of output changes
		if(SMSisPending==1){
			SMSisPending = 0;
			sim80x_SendSMS(phone, SMStext, 6000);	
		}
		
		#if(__SENSORS_ARE_ENABLE__==1)
			measureAndLog();  
		#endif
		
		HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
		HAL_RTC_GetAlarm(&hrtc, &alarm_struct_A, RTC_ALARM_A, RTC_FORMAT_BIN);
		HAL_RTC_GetAlarm(&hrtc, &alarm_struct_B, RTC_ALARM_B, RTC_FORMAT_BIN);
		
		
		if((AlarmIsSet==1) || (nextEventTimeStamp != 0))
		{
			if(AlarmIsSet==1)
			{
				memset(str,NULL,size);
				snprintf(str,sizeof(str),"Time: %d/%02d/%02d %02d:%02d:%02d -- Next Program Alarm: %d %02d:%02d:%02d -- Next Program ID: %d\r\n", 2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds, alarm_struct_A.AlarmDateWeekDay, alarm_struct_A.AlarmTime.Hours, alarm_struct_A.AlarmTime.Minutes, alarm_struct_A.AlarmTime.Seconds, HAL_RTCEx_BKUPRead(&hrtc, PROGRAM_ID_ADDRESS));
				DEBUG(str);
			}
			if(nextEventTimeStamp != 0 || processProgramsAlarmIsSet==1)	
			{
				memset(str,NULL,size);
				snprintf(str,sizeof(str),"Time: %d/%02d/%02d %02d:%02d:%02d -- Next Process Program Alarm: %d %02d:%02d:%02d --  Button ID: %d --Current Event Number: %d \r\n", 2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds, alarm_struct_B.AlarmDateWeekDay, alarm_struct_B.AlarmTime.Hours, alarm_struct_B.AlarmTime.Minutes, alarm_struct_B.AlarmTime.Seconds,buttonId+1 ,eventNumber[buttonId]+1);
				DEBUG(str);
			}
		}
		else 
		{
			memset(str,NULL,size);			
			snprintf(str,sizeof(str),"Time: %d/%02d/%02d %02d:%02d:%02d -- There is no active Alarm\r\n", 2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds);
			DEBUG(str);
		}
			HAL_Delay(2000);
		// This section runs each 5 minutes
	
		
		
		/////////////////////////////////////////////////////////////////
		if(Alarm_A_occur)	// if Alarm_A occur, This section runs
	 {
			HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
			memset(str,NULL,size);
			snprintf(str,sizeof(str),"\n\r  		    Time: %d/%02d/%02d %02d:%02d:%02d  \r\n", 2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds);
			DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
			DEBUG(				"|                Alarm A Event              |");
			DEBUG(str);
			DEBUG(				"\r\n-------------------------------------------------\r\n");
			if(AlarmIsSet)
			{
				
				SetAlarm(&hrtc);
				PrintAllPrograms();
				
				
			}				
			Alarm_A_occur = 0;
	 }
	 
  #if(BUTTONS_NUM==2)
		if(Alarm_B_occur[0]||Alarm_B_occur[1])
		{
	#endif
  #if(BUTTONS_NUM==3)
		if(Alarm_B_occur[0]||Alarm_B_occur[1]||Alarm_B_occur[2])
		{
	#endif	
  #if(BUTTONS_NUM==4)
		if(Alarm_B_occur[0]||Alarm_B_occur[1]||Alarm_B_occur[2]||Alarm_B_occur[3])
		{
	#endif				
			for(CheckAlarm_buttonId=0;CheckAlarm_buttonId<BUTTONS_NUM;CheckAlarm_buttonId++)	
			{		 
				 if(Alarm_B_occur[CheckAlarm_buttonId])	// if Alarm_B occur, This section runs
				 {
					 if(processProgramsAlarmIsSet)
					 {
							DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
							DEBUG(				"|         Procces Alarm B Event            |");
							DEBUG(				"\r\n-------------------------------------------------\r\n");
							eventNumber[CheckAlarm_buttonId]++;
							SetProcessProgramsAlarm(&hrtc);
					 }
				 }
			 }
		 }
		////////////////////////////////////////////////////////////////////////////////////////
		
		
		
		// Abgiri execution:
		//AbGiriProccess();
	getStatusUnSuccessCounter = 0;				
	while(action && (getStatusUnSuccessCounter < 3)){	
		
		// Check SimCard
		if(Check_SimCard(SIM800_Status[0]) == HAL_OK)  
			SIM800_Status[0] = 1;
		else
			SIM800_Status[0] = 0;
		
		// Check Network
		if(Check_Network_Registered(SIM800_Status[1]) == HAL_OK)
			SIM800_Status[1] = 1;
		else
			SIM800_Status[1] = 0;
		
		
		
		if(SIM800_Status[0] == 1 && SIM800_Status[1] == 1)  // If SimCard plugged and Registered on Network
		{
			sim80x_HTTP_Start();
			GetStatus((char*)"true");
		}
		
			/*
				get_output_result has 5 values:
					-- 0: Means the connection is lost
					-- 1: Means connection is stable and there is no change in programs and outputs 
					-- 2: Means connection is stable and programs are changed
					-- 3: Means connection is stable and outputs have to be changed
					-- 4: Means connection is stable and both of programs and outputs have to be changed
			*/
	getStatusUnSuccessCounter++;
	if(GetStatus_Is_Success)
	{
		getStatusUnSuccessCounter = 0;
		//AlarmIsSet = 0;
		isConnect = 1;
		
		if(Button_Actor[0] || Button_Actor[1] || Button_Actor[2]) // for fixing #bug1;
		{
			
		}
		else
		{
			action=0;
			ApplyAction(NewStatus);
			if(ProgramIsChanged)
			{
				DEBUG("\n\r --- CHANGE IN PROGRAMS --- \n\r");
				GetAllPrograms();
				if(getProgramsStatus)
				{	
					if(Alarm_A_occur)	// if Alarm_A occur, This section runs
				 {
						HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
						memset(str,NULL,size);
						snprintf(str,sizeof(str),"\n\r   		   Time: %d/%02d/%02d %02d:%02d:%02d  \r\n", 2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds);
						DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
						DEBUG(				"|                Alarm A Event              |");
					 	DEBUG(str);
						DEBUG(				"\r\n-------------------------------------------------\r\n");
						if(AlarmIsSet)
						{
							SetAlarm(&hrtc);		
						}		
						Alarm_A_occur = 0;
				 }
					SetNextAlarm(&hrtc);
					PrintAllPrograms();

				}
			}
			if(proccesProgramIsChanged)
			{
				DEBUG("\n\r --- CHANGE IN PROCCES PROGRAMS--- \n\r");
				if(GetAllProcessPrograms() == 1)
					GetProcessProgram_IS_Success = 1;
				else
					GetProcessProgram_IS_Success = 0;
			}
			GetStatus((char*)"false");
		}
		
	}
	else if((getStatusUnSuccessCounter == 2) && (GetStatus_Is_Success == 0))
	{
		
	}
	
	
	 
		 
		 
		 
			#if(__SENSORS_ARE_ENABLE__==1)
				if(isConnect==1)
					postSensors();
			#endif
			sim80x_HTTP_Stop();
			tim5CallbackCounter=0;
			lastTim5CallbackCounter=0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* RTC_WKUP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
  /* RTC_Alarm_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/* USER CODE BEGIN 4 */

/**

  * @brief  RTC wakeup interrupt.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc){
  action=1;	
}

/**
  * @brief  RTC alarm A interrupt.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	Alarm_A_occur = 1;
	DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
	DEBUG(						"|           ALARM A Callback happened           |");
	DEBUG(				"\r\n-------------------------------------------------\r\n");
}
/**
  * @brief  RTC alarm B interrupt.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  */
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc){
if(processProgramsAlarmIsSet)
		Alarm_B_occur[buttonId]=1;
		DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
		DEBUG(						"|           ALARM B Callback happened           |");
		DEBUG(				"\r\n-------------------------------------------------\r\n");
}
/**
  * @brief  GPIO external interrupt handler.
  * @param  pin number
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	#if(__SENSORS_ARE_ENABLE__==1)
		if(GPIO_Pin == flow1_EXIT_Pin)
		{
			while(!(HAL_GPIO_ReadPin(flow1_EXIT_GPIO_Port,flow1_EXIT_Pin)))
			counter++;
			if(counter>2000)
			{
				Flow1++;
				HAL_RTCEx_BKUPWrite(&hrtc,1,Flow1);
				counter=0;
			}
		}
		
		if(GPIO_Pin == flow2_EXIT_Pin)
		{
			while(!(HAL_GPIO_ReadPin(flow2_EXIT_GPIO_Port,flow2_EXIT_Pin)))
			counter++;
			
			if(counter>40000)
			{
				Flow2++;
				HAL_RTCEx_BKUPWrite(&hrtc,2,Flow2);
				counter=0;
			}
		}
	#endif
	
#if(__BUTTONS_ARE_ENABLE__==1)

  #if(BUTTONS_NUM == 2)
		while((HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin) || HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin)) && (counter<2000002))
			counter++;
		while(( !(HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin))  || !(HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin))) && (counter<2000002))
			counter++;
	#endif
	
	#if(BUTTONS_NUM == 3)
		while((HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin) || HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin)|| HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin)) && (counter<2000002))
			counter++;
		while((!(HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin)) || !(HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin))|| !(HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin))) && (counter<2000002))
			counter++;
	#endif
		
	#if(BUTTONS_NUM == 4)
		while((HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin) || HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin)|| HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin)|| HAL_GPIO_ReadPin(SW4_GPIO_Port,SW4_Pin)) && (counter<2000002))
			counter++;
		while((!(HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin)) || !(HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin))|| !(HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin))|| !(HAL_GPIO_ReadPin(SW4_GPIO_Port,SW4_Pin))) && (counter<2000002))
			counter++;
	#endif

		
		if(counter>2000000)
		{
			struct tm  			currentTime;
			uint32_t 				currentTimeStamp = 0;	
			HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
			currentTime.tm_year	= Date.Year + 2000  - 1900;
			currentTime.tm_mon		= Date.Month-1;
			currentTime.tm_mday	  = Date.Date;
			currentTime.tm_hour	  = Time.Hours;
			currentTime.tm_min		= Time.Minutes;
			currentTime.tm_sec		= Time.Seconds;
			currentTime.tm_isdst	= -1;	
			currentTimeStamp = mktime(&currentTime);//convert current time to time stamp
			counter = 0;
			
			#if(BUTTONS_NUM == 2 || BUTTONS_NUM == 3 || BUTTONS_NUM == 4 )
				if(GPIO_Pin == SW1_Pin)//button 1 interrupt		///////////NEW ADDED/////////////////
				{
					DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
					DEBUG(						"|         Button 1 Status has been changed      |");
					DEBUG(				"\r\n-------------------------------------------------\r\n");				
					buttonId = 0;
					if(buttonsStatus[buttonId] == 1 && getProcessProgramsStatus)
					{
						HAL_GPIO_WritePin(SW_Light1_GPIO_Port,SW_Light1_Pin,0); // Turn OFF LED_SW1
						buttonsStatus[buttonId] = 0;		//toggle buttonsStatus[0]
						processProgramsAlarmIsSet = 0;//disable process program Alarm
						DEBUG("\n\r **Buttons Status: ");
							memset(str,NULL,size);
							for(uint8_t i=0;i<BUTTONS_NUM;i++)
								{
									sprintf(str,"%d",buttonsStatus[i]);
									DEBUG(str);
								}
						memset(ContentStr,NULL,size);
						snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d    \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
						DEBUG(ContentStr);
						DEBUG("\n\r**Process Programs Alarm is disables \n\r");
						
						//save last last time and last status of the button its status have been changed:
						changeButtonStatusTimeStamp[buttonId][0] = currentTimeStamp;
						changeButtonStatusTimeStamp[buttonId][1] = buttonsStatus[buttonId];	
						SetProcessProgramsAlarm(&hrtc);
								
						 
					}
					else if(buttonsStatus[buttonId] == 0 && getProcessProgramsStatus)
					{
						//button 0 pushed & get process programs is successful 
						HAL_GPIO_WritePin(SW_Light1_GPIO_Port,SW_Light1_Pin,1); // Turn ON LED_SW1 
						buttonsStatus[buttonId] = 1; //toggle buttonsStatus[0]					
						//save last last time and last status of the button its status have been changed:
						changeButtonStatusTimeStamp[buttonId][0] = currentTimeStamp;
						changeButtonStatusTimeStamp[buttonId][1] = buttonsStatus[buttonId];
						DEBUG("\n\r **Buttons Status: ");
							memset(str,NULL,size);
							for(uint8_t i=0;i<BUTTONS_NUM;i++)
								{
									sprintf(str,"%d",buttonsStatus[i]);
									DEBUG(str);
								}
						memset(ContentStr,NULL,size);
						snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d    \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
						DEBUG(ContentStr);					
						eventNumber[buttonId] = 0;
						SetProcessProgramsAlarm(&hrtc);//Button 2 Status =1 and we must aplly its first event action and set alarm for another.
					}
					while(!HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin));
				}
				
				if(GPIO_Pin == SW2_Pin)//button 1 interrupt      ///////////NEW ADDED/////////////////
				{
					DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
					DEBUG(						"|         Button 2 Status has been changed      |");
					DEBUG(				"\r\n-------------------------------------------------\r\n");				
					 buttonId = 1;
					if(buttonsStatus[buttonId] == 1 && getProcessProgramsStatus) 
					{	
						HAL_GPIO_WritePin(SW_Light2_GPIO_Port,SW_Light2_Pin,0); // Turn OFF LED_SW2
						buttonsStatus[buttonId] = 0; //toggle buttonsStatus[1]
						processProgramsAlarmIsSet = 0; //disable process program Alarm
						DEBUG("\n\r **Buttons Status: ");
							memset(str,NULL,size);
							for(uint8_t i=0;i<BUTTONS_NUM;i++)
								{
									sprintf(str,"%d",buttonsStatus[i]);
									DEBUG(str);
								}
						memset(ContentStr,NULL,size);
						snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d    \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
						DEBUG(ContentStr);	
						DEBUG("\n\r**Process Programs Alarm is disables \n\r");
						
						//save last last time and last status of the button its status have been changed:
						changeButtonStatusTimeStamp[buttonId][0] = currentTimeStamp;
						changeButtonStatusTimeStamp[buttonId][1] = buttonsStatus[buttonId];						
						SetProcessProgramsAlarm(&hrtc);
					}
					else if(buttonsStatus[buttonId] == 0 && getProcessProgramsStatus)
					{
						//button 1 pushed & get process programs is successful 
						HAL_GPIO_WritePin(SW_Light2_GPIO_Port,SW_Light2_Pin,1); // Turn ON LED_SW2
						buttonsStatus[buttonId] = 1;//toggle buttonsStatus[1]
						
						//save last last time and last status of the button its status have been changed:
						changeButtonStatusTimeStamp[buttonId][0] = currentTimeStamp;
						changeButtonStatusTimeStamp[buttonId][1] = buttonsStatus[buttonId];
						DEBUG("\n\r **Buttons Status: ");
							memset(str,NULL,size);
							for(uint8_t i=0;i<BUTTONS_NUM;i++)
								{
									sprintf(str,"%d",buttonsStatus[i]);
									DEBUG(str);
								}
						memset(ContentStr,NULL,size);
						snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d    \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
						DEBUG(ContentStr);		
						eventNumber[buttonId] = 0;
						SetProcessProgramsAlarm(&hrtc); //Button 2 Status =1 and we must aplly its first event action and set alarm for anothers.
					}
					while(!HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin));
				}
			#endif
				
		#if(BUTTONS_NUM == 3 || BUTTONS_NUM == 4 )
			if(GPIO_Pin == SW3_Pin )
			{
				
				DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
				DEBUG(						"|         Button 3 Status has been changed      |");
				DEBUG(				"\r\n-------------------------------------------------\r\n");
				
				buttonId = 2;
				if((buttonsStatus[buttonId] == 1) && (getProcessProgramsStatus) && (HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin) == 0)) 
					{	
						buttonsStatus[buttonId] = 0; 		// toggle buttonsStatus[3]
						processProgramsAlarmIsSet = 0;  // disable process program Alarm
						DEBUG("\n\r **Buttons Status: ");
							memset(str,NULL,size);
							for(uint8_t i = 0; i < BUTTONS_NUM; i++)
								{
									sprintf(str, "%d", buttonsStatus[i]);
									DEBUG(str);
								}
						
						memset(ContentStr,NULL,size);
						snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d    \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
						DEBUG(ContentStr);	
						DEBUG("\n\r**Process Programs Alarm is disables \n\r");
						
						//save last last time and last status of the button its status have been changed:
						changeButtonStatusTimeStamp[buttonId][0] = currentTimeStamp;
						changeButtonStatusTimeStamp[buttonId][1] = buttonsStatus[buttonId];								
						SetProcessProgramsAlarm(&hrtc);
					}
				else if((buttonsStatus[buttonId] == 0) && (getProcessProgramsStatus) && (HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin) == 1))
					{
						//button 3 pushed & get process programs is successful 
						buttonsStatus[buttonId] = 1;//toggle buttonsStatus[2]
						
						//save last last time and last status of the button its status have been changed:
						changeButtonStatusTimeStamp[buttonId][0] = currentTimeStamp;
						changeButtonStatusTimeStamp[buttonId][1] = buttonsStatus[buttonId];
						DEBUG("\n\r **Buttons Status: ");
							memset(str,NULL,size);
							for(uint8_t i=0;i<BUTTONS_NUM;i++)
								{
									sprintf(str,"%d",buttonsStatus[i]);
									DEBUG(str);
								}
						memset(ContentStr,NULL,size);
						snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d    \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
						DEBUG(ContentStr);		
						eventNumber[buttonId] = 0;
						SetProcessProgramsAlarm(&hrtc); //Button 3 Status =1 and we must aplly its first event action and set alarm for anothers.
					}
					#if(DEVICE_TYPE != 2)
						while(!HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin));
					#endif
			}
		#endif
		#if(BUTTONS_NUM == 4 )
			if(GPIO_Pin == SW4_Pin )
			{
				
				DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
				DEBUG(						"|         Button 4 Status has been changed      |");
				DEBUG(				"\r\n-------------------------------------------------\r\n");
				buttonId = 3;
				
				if(buttonsStatus[buttonId] == 1 && getProcessProgramsStatus) 
					{	
						buttonsStatus[buttonId] = 0; 		// toggle buttonsStatus[4]
						processProgramsAlarmIsSet = 0;  // disable process program Alarm
						DEBUG("\n\r **Buttons Status: ");
							memset(str,NULL,size);
							for(uint8_t i=0;i<BUTTONS_NUM;i++)
								{
									sprintf(str,"%d",buttonsStatus[i]);
									DEBUG(str);
								}
						
						memset(ContentStr,NULL,size);
						snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d    \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
						DEBUG(ContentStr);	
						DEBUG("\n\r**Process Programs Alarm is disables \n\r");
						
						//save last last time and last status of the button its status have been changed:
						changeButtonStatusTimeStamp[buttonId][0] = currentTimeStamp;
						changeButtonStatusTimeStamp[buttonId][1] = buttonsStatus[buttonId];								
						SetProcessProgramsAlarm(&hrtc);
					}
				else if(buttonsStatus[buttonId] == 0 && getProcessProgramsStatus)
					{
						//button 3 pushed & get process programs is successful 
						buttonsStatus[buttonId] = 1;//toggle buttonsStatus[2]
						
						//save last last time and last status of the button its status have been changed:
						changeButtonStatusTimeStamp[buttonId][0] = currentTimeStamp;
						changeButtonStatusTimeStamp[buttonId][1] = buttonsStatus[buttonId];
						DEBUG("\n\r **Buttons Status: ");
							memset(str,NULL,size);
							for(uint8_t i=0;i<BUTTONS_NUM;i++)
								{
									sprintf(str,"%d",buttonsStatus[i]);
									DEBUG(str);
								}
						memset(ContentStr,NULL,size);
						snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d    \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
						DEBUG(ContentStr);		
						eventNumber[buttonId] = 0;
						SetProcessProgramsAlarm(&hrtc); //Button 3 Status =1 and we must aplly its first event action and set alarm for anothers.
					}

			}
		#endif
		counter = 0;
		
		
//			if(HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin) == 1)
//			{
//				HAL_GPIO_WritePin(relay1_GPIO_Port,relay1_Pin,1);
//			}
//			if(HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin) == 0)
//			{
//				HAL_GPIO_WritePin(relay1_GPIO_Port,relay1_Pin,0);
//			}
		}
		
	#endif
}
/**
  * @brief  Communicate with sensors and collect their values. Then log them in debug port.
  */
void measureAndLog(void){
	
		//Tempereture and humidity
		//*
		TH = AM2305_GetData();
		if(TH[2]==0xFFFF)
		{
			TH[0]=0.0;
			TH[1]=0.0;
		}
		//*/
				
		//Moisture
		//*
		HAL_ADC_Start_DMA(&hadc1, AnalogSensors, 3);
		moisture[averageItterator]=AnalogSensors[2];
		averageItterator++;
		if (averageItterator==10)	averageItterator=0;
		
		HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
		
		memset(str,NULL,size);
		snprintf(str,sizeof(str),"Time: %02d:%02d:%02d *** moisture: %2.2f *** Humidity: %2.1f *** Temperature: %2.1f *** Flow1: %d *** Flow2: %d\r\n", Time.Hours, Time.Minutes, Time.Seconds,(AnalogSensors[2]*3.3)/4095, TH[0], TH[1], Flow1, Flow2);
		DEBUG(str);
		HAL_Delay(1000);
		//*/
}
/**
  * @brief  Calculate the average of sensor values (if neesed) then POST them to server
  */
void postSensors(void){
	// calculating moisture average
	for(uint8_t index=0;index<10;index++)
		AnalogSensors[2]+=moisture[index];
	AnalogSensors[2]=AnalogSensors[2]/11;
			
	memset(ContentStr,NULL,size);
	snprintf(ContentStr,size,"{\"deviceId\":\"%d\",\"temp_value\":%2.2f,\"soil_moisture_value\":%2.2f,\"humidity_value\":%2.2f,\"water_value\":%d,\"trial_water_value\":%d}\r\n", deviceId, TH[1], (AnalogSensors[2]*3.3)/4095, TH[0], Flow1, Flow2);
	DEBUG(ContentStr);
			
	sim80x_HTTP_Post(ContentStr, SERVER_IP, "multiSensor/", ContentStr);	
}

/**
  * @brief  Calculate the average of sensor values (if neesed) then POST them to server
	* @retval An uint8_t value according to the following:
							-- 0: Means the connection is lost
							-- 1: Means connection is stable and there is no change in programs and outputs 
							-- 2: Means connection is stable and programs are changed
							-- 3: Means connection is stable and outputs have to be changed
							-- 4: Means connection is stable and both of programs and outputs have to be changed
						The output status (in case of change) is stored in "str" variable.
  */
uint8_t GetStatus(char* response){
	char *startAnswer;
	char *endAnswer;
	char  getStatusRaw[size];//used in separating GetStatus content
	char  value_str[30];
	uint16_t value_uint;

	uint8_t ReturnValue = 0;
	memset(rssiStrValue,NULL,5);
	sim80x_Get_RSSI();
	memset(ebcStatus, NULL,20);
	memset(ContentStr, NULL,size);
	memset(ebcList, NULL,size);
	memset(ebc_lc, NULL,size);
	memset(outputsStatusFeedBack,NULL,OUTPUTS_NUM+1);
//	memset(programLastTime,NULL,50);
	
#if(__IS_OLED__==1)
		HAL_TIM_Base_Stop_IT(&htim5);
#endif
	
	#if(BUTTONS_NUM==2)
	snprintf(ebcStatus,20,"%d%d",changeButtonStatusTimeStamp[0][1],changeButtonStatusTimeStamp[1][1]);
	TimestampTOdate(changeButtonStatusTimeStamp[0][0],0);//calculate last time  changing status of button 1 in sTime type
	TimestampTOdate(changeButtonStatusTimeStamp[1][0],1);//calculate last time  changing status of button 2 in sTime type
	snprintf(ebcList,size,"[{\"n\":1,\"c\":%d,\"lt\":\"%02d/%02d/%02d,%02d:%02d:%02d\",\"d\":%d},{\"n\":2,\"c\":%d,\"lt\":\"%02d/%02d/%02d,%02d:%02d:%02d\",\"d\":%d}]"
						,changeButtonStatusTimeStamp[0][1],buttonsLastTime.day[0],buttonsLastTime.month[0],buttonsLastTime.year[0],buttonsLastTime.hours[0]
						,buttonsLastTime.minutes[0],buttonsLastTime.seconds[0],Button_Actor[0],changeButtonStatusTimeStamp[1][1],buttonsLastTime.day[1],buttonsLastTime.month[1],buttonsLastTime.year[1],buttonsLastTime.hours[1]
						,buttonsLastTime.minutes[1],buttonsLastTime.seconds[1],Button_Actor[1]);					
	#endif
	
	#if(BUTTONS_NUM==3)
	snprintf(ebcStatus,20,"%d%d%d",changeButtonStatusTimeStamp[0][1],changeButtonStatusTimeStamp[1][1],changeButtonStatusTimeStamp[2][1]);
	TimestampTOdate(changeButtonStatusTimeStamp[0][0],0);//calculate last time  changing status of button 1 in sTime type
	TimestampTOdate(changeButtonStatusTimeStamp[1][0],1);//calculate last time  changing status of button 2 in sTime type
	TimestampTOdate(changeButtonStatusTimeStamp[2][0],2);//calculate last time  changing status of button 3 in sTime type
	snprintf(ebcList,size," \n\r [{\"n\":1,\"c\":%d,\"lt\":\"%02d/%02d/%02d,%02d:%02d:%02d\",\"d\":%d},{\"n\":2,\"c\":%d,\"lt\":\"%02d/%02d/%02d,%02d:%02d:%02d\",\"d\":%d},{\"n\":3,\"c\":%d,\"lt\":\"%02d/%02d/%02d,%02d:%02d:%02d\",\"d\":%d} ] \n\r"
						,changeButtonStatusTimeStamp[0][1],buttonsLastTime.day[0],buttonsLastTime.month[0],buttonsLastTime.year[0],buttonsLastTime.hours[0]
						,buttonsLastTime.minutes[0],buttonsLastTime.seconds[0],Button_Actor[0],changeButtonStatusTimeStamp[1][1],buttonsLastTime.day[1],buttonsLastTime.month[1],buttonsLastTime.year[1],buttonsLastTime.hours[1]
						,buttonsLastTime.minutes[1],buttonsLastTime.seconds[1],Button_Actor[1],changeButtonStatusTimeStamp[2][1],buttonsLastTime.day[2],buttonsLastTime.month[2],buttonsLastTime.year[2],buttonsLastTime.hours[2]
						,buttonsLastTime.minutes[2],buttonsLastTime.seconds[2],Button_Actor[2]);	
	#endif
	
	#if(BUTTONS_NUM==4)
	snprintf(ebcStatus,4,"%d%d%d%d",changeButtonStatusTimeStamp[0][1],changeButtonStatusTimeStamp[1][1],changeButtonStatusTimeStamp[2][1],changeButtonStatusTimeStamp[3][1]);
	TimestampTOdate(changeButtonStatusTimeStamp[0][0],0);//calculate last time  changing status of button 1 in sTime type
	TimestampTOdate(changeButtonStatusTimeStamp[1][0],1);//calculate last time  changing status of button 2 in sTime type
	TimestampTOdate(changeButtonStatusTimeStamp[2][0],2);//calculate last time  changing status of button 3 in sTime type
	TimestampTOdate(changeButtonStatusTimeStamp[3][0],3);//calculate last time  changing status of button 4 in sTime type
	snprintf(ebcList,size," \n\r [{\"n\":1,\"c\":%d,\"lt\":\"%02d/%02d/%02d,%02d:%02d:%02d\"},{\"n\":2,\"c\":%d,\"lt\":\"%02d/%02d/%02d,%02d:%02d:%02d\"},{\"n\":3,\"c\":%d,\"lt\":\"%02d/%02d/%02d,%02d:%02d:%02d\"} ] \n\r"
						,changeButtonStatusTimeStamp[0][1],buttonsLastTime.day[0],buttonsLastTime.month[0],buttonsLastTime.year[0],buttonsLastTime.hours[0]
						,buttonsLastTime.minutes[0],buttonsLastTime.seconds[0],changeButtonStatusTimeStamp[1][1],buttonsLastTime.day[1],buttonsLastTime.month[1],buttonsLastTime.year[1],buttonsLastTime.hours[1]
						,buttonsLastTime.minutes[1],buttonsLastTime.seconds[1],changeButtonStatusTimeStamp[2][1],buttonsLastTime.day[2],buttonsLastTime.month[2],buttonsLastTime.year[2],buttonsLastTime.hours[2]
						,buttonsLastTime.minutes[2],buttonsLastTime.seconds[2],changeButtonStatusTimeStamp[3][1],buttonsLastTime.day[3],buttonsLastTime.month[3],buttonsLastTime.year[3],buttonsLastTime.hours[3]
						,buttonsLastTime.minutes[3],buttonsLastTime.seconds[3]);			
			
	#endif
	if (simCardGprsOk == 1)			// "simCardGprsOK" variable get value in sim80x_HTTP_Start() function and show result of this function
	{
		for(uint8_t i = 0; i < BUTTONS_NUM; i++)	//set zero Actor button flag
				Button_Actor[i] = 0;			
	}
	else
		return 0;				// if sim80x_HTTP_Start() function was unsuccess, exit from GetStatus() function 
	//Reading outputs status as a feedback :
	
	#if(OUTPUTS_NUM==2)
		outputsStatusFeedBack[0]='0';
		outputsStatusFeedBack[1]='0';
		if(HAL_GPIO_ReadPin(relay1_GPIO_Port, relay1_Pin))
			outputsStatusFeedBack[0]='1';
		if(HAL_GPIO_ReadPin(relay2_GPIO_Port, relay2_Pin))
			outputsStatusFeedBack[1]='1';
	#endif
	 
	#if(OUTPUTS_NUM==3)
		outputsStatusFeedBack[0]='0';
		outputsStatusFeedBack[1]='0';
		outputsStatusFeedBack[2]='0';
		if(HAL_GPIO_ReadPin(relay1_GPIO_Port, relay1_Pin))
			outputsStatusFeedBack[0]='1';
		if(HAL_GPIO_ReadPin(relay2_GPIO_Port, relay2_Pin))
			outputsStatusFeedBack[1]='1';	
		if(HAL_GPIO_ReadPin(relay3_GPIO_Port, relay3_Pin))
			outputsStatusFeedBack[2]='1';
	#endif
	
	#if(OUTPUTS_NUM==4)
		outputsStatusFeedBack[0]='0';
		outputsStatusFeedBack[1]='0';
		outputsStatusFeedBack[2]='0';
		outputsStatusFeedBack[3]='0';
		if(HAL_GPIO_ReadPin(relay1_GPIO_Port, relay1_Pin))
			outputsStatusFeedBack[0]='1';
		if(HAL_GPIO_ReadPin(relay2_GPIO_Port, relay2_Pin))
			outputsStatusFeedBack[1]='1';	
		if(HAL_GPIO_ReadPin(relay3_GPIO_Port, relay3_Pin))
			outputsStatusFeedBack[2]='1';		
		if(HAL_GPIO_ReadPin(relay4_GPIO_Port, relay4_Pin))
			outputsStatusFeedBack[3]='1';			
			
	#endif
	
	#if(OUTPUTS_NUM==6)
		outputsStatusFeedBack[0]='0';
		outputsStatusFeedBack[1]='0';
		outputsStatusFeedBack[2]='0';
		outputsStatusFeedBack[3]='0';
		outputsStatusFeedBack[4]='0';
		outputsStatusFeedBack[5]='0';
		if(HAL_GPIO_ReadPin(relay1_GPIO_Port, relay1_Pin))
			outputsStatusFeedBack[0]='1';
		if(HAL_GPIO_ReadPin(relay2_GPIO_Port, relay2_Pin))
			outputsStatusFeedBack[1]='1';
		if(HAL_GPIO_ReadPin(relay3_GPIO_Port, relay3_Pin))
			outputsStatusFeedBack[2]='1';		
		if(HAL_GPIO_ReadPin(relay4_GPIO_Port, relay4_Pin))
			outputsStatusFeedBack[3]='1';
		if(HAL_GPIO_ReadPin(relay5_GPIO_Port, relay5_Pin))
			outputsStatusFeedBack[4]='1';		
		if(HAL_GPIO_ReadPin(relay6_GPIO_Port, relay6_Pin))
			outputsStatusFeedBack[5]='1';			
	#endif
	
	#if(OUTPUTS_NUM==8)
		outputsStatusFeedBack[0]='0';
		outputsStatusFeedBack[1]='0';
		outputsStatusFeedBack[2]='0';
		outputsStatusFeedBack[3]='0';
		outputsStatusFeedBack[4]='0';
		outputsStatusFeedBack[5]='0';
		outputsStatusFeedBack[6]='0';
		outputsStatusFeedBack[7]='0';
		if(HAL_GPIO_ReadPin(relay1_GPIO_Port, relay1_Pin))
			outputsStatusFeedBack[0]='1';
		if(HAL_GPIO_ReadPin(relay2_GPIO_Port, relay2_Pin))
			outputsStatusFeedBack[1]='1';
		if(HAL_GPIO_ReadPin(relay3_GPIO_Port, relay3_Pin))
			outputsStatusFeedBack[2]='1';		
		if(HAL_GPIO_ReadPin(relay4_GPIO_Port, relay4_Pin))
			outputsStatusFeedBack[3]='1';
		if(HAL_GPIO_ReadPin(relay5_GPIO_Port, relay5_Pin))
			outputsStatusFeedBack[4]='1';		
		if(HAL_GPIO_ReadPin(relay6_GPIO_Port, relay6_Pin))
			outputsStatusFeedBack[5]='1';	
		if(HAL_GPIO_ReadPin(relay7_GPIO_Port, relay7_Pin))
			outputsStatusFeedBack[6]='1';		
		if(HAL_GPIO_ReadPin(relay8_GPIO_Port, relay8_Pin))
			outputsStatusFeedBack[7]='1';				
	#endif
	
	
	//Reading Button last New_Progran status as ebc_lc :
	#if(BUTTONS_NUM==2)
//		ee24_read(&hi2c1,40,(uint8_t *)programButtonLastTime[0],20,100);
//		ee24_read(&hi2c1,60,(uint8_t *)programButtonLastTime[1],20,100);
		memset(ebc_lc, NULL, size);
		snprintf(ebc_lc,100,"[\"%s\",\"%s\"]",programButtonLastTime[0],programButtonLastTime[1]);
	#endif
	
	#if(BUTTONS_NUM==3)
//		ee24_read(&hi2c1,40,(uint8_t *)programButtonLastTime[0],20,100);
//		ee24_read(&hi2c1,60,(uint8_t *)programButtonLastTime[1],20,100);
//		ee24_read(&hi2c1,80,(uint8_t *)programButtonLastTime[2],20,100);
		memset(ebc_lc, NULL, size);
		snprintf(ebc_lc,100,"[\"%s\",\"%s\",\"%s\"]",programButtonLastTime[0],programButtonLastTime[1],programButtonLastTime[2]);
	#endif
	
//ee24_read(&hi2c1,10,(uint8_t *)programLastTime,20,100);  // read last program time from eeprom
snprintf(sendStatus,size,"{\"land\": %d,\"device\": %d,\"sc\": \"%s\",\"nc\": %d,\"ebc\": \"%s\",\"ebc_list\": %s, \"ebc_lc\": %s,\"response\": %s,\"ft\": \"%s\"}"
				,landId,deviceId,outputsStatusFeedBack,rssiIntValue,ebcStatus,ebcList,ebc_lc,response,programLastTime);

DEBUG("\n\r");	
DEBUG(sendStatus);	
DEBUG("\n\r");	
sim80x_HTTP_Post(sendStatus, SERVER_IP,"eo6/",sendStatus);
char	json_result[20];
	
			
if((Button_Actor[0] == 0) && (Button_Actor[1] == 0) && (Button_Actor[2] == 0)) // for fixing #bug1;
{
	if(strstr((char *)sendStatus, "pbc") != NULL)
	{
		//remove SIM800 commands and OK in response:
		startAnswer	= strstr(sendStatus, "{");
		endAnswer		= strstr(sendStatus, "\"tn\"") + 7;	
		memset(getStatusRaw, NULL, size);
		for(int i = (startAnswer - sendStatus); i < (endAnswer - sendStatus); i++)
			getStatusRaw[i-(startAnswer-sendStatus)] = sendStatus[i]; // separating processProgram of every buttons
		DEBUG("\n\r*************\n\r");
		DEBUG(getStatusRaw);
		DEBUG("\n\r*************\n\r");
		
		// Extracting outputs:
		startAnswer	= strstr(getStatusRaw, "\"o\":")+5;
		endAnswer		= strstr(getStatusRaw, "}") -1;	
		memset(value_str, NULL, 30);
		for(int i = (startAnswer - getStatusRaw); i < (endAnswer - getStatusRaw); i++)
			value_str[i-(startAnswer-getStatusRaw)] = getStatusRaw[i]; 
		value_uint= str2dec(value_str);// value_str is string like : "02102310".it is include base 4 number in string type and we Converts it to decimal number in uint16_t type 
		memset(ContentStr, NULL, size);
		NewStatus[0]=value_uint%256; 	 //Convert value_uint decimal number to a base 256 number (2byte)
		NewStatus[1]=value_uint/256;	
		memset(ContentStr, NULL, size);
		snprintf(ContentStr,30,"outputs Status is %s\n\r",value_str );
		DEBUG(ContentStr);
		
		// Extracting program status:
		startAnswer	= strstr(getStatusRaw, "\"p\":")+5;
		endAnswer		= strstr(getStatusRaw, ",\"lp\"") -1;	
		memset(value_str, NULL, 30);
		for(int i = (startAnswer - getStatusRaw); i < (endAnswer - getStatusRaw); i++)
			value_str[i-(startAnswer-getStatusRaw)] = getStatusRaw[i];
		if(getProgramsStatus == 1)
			ProgramIsChanged = atoi(value_str);
		else
			ProgramIsChanged = 1;
		memset(ContentStr, NULL, size);
		snprintf(ContentStr,30,"program status is %d\n\r",ProgramIsChanged );
		DEBUG(ContentStr);
		
		// Extracting program last time:
		startAnswer	= strstr(getStatusRaw, "\"lp\":")+6;
		endAnswer		= strstr(getStatusRaw, ",\"cps\"") -1;	
		memset(value_str, NULL, 30);
		for(int i = (startAnswer - getStatusRaw); i < (endAnswer - getStatusRaw); i++)
			value_str[i-(startAnswer-getStatusRaw)] = getStatusRaw[i]; 	
		memset(programLastTime, NULL,50);
		memcpy(programLastTime,value_str,sizeof(value_str));
//		ee24_write(&hi2c1,10,(uint8_t *)programLastTime,20,100);
		memset(ContentStr, NULL, size);
		snprintf(ContentStr,50,"program Last Time is %s\n\r",programLastTime );
		DEBUG(ContentStr);
		DEBUG("\n\r*************\n\r");
		
		
		// Extracting Button_1 New Program Status :
		startAnswer	= strstr(getStatusRaw, "\"n\":1")+11;									// get Button_1 condition
		ButtonCondition[0][0] = getStatusRaw[(startAnswer - getStatusRaw)];
		startAnswer	= strstr(getStatusRaw, "\"n\":1")+19;									// get Button_1 Last_time NEW_Program
		endAnswer		= strstr(getStatusRaw, "\"n\":1")+36;
		for(int i = (startAnswer - getStatusRaw); i < (endAnswer - getStatusRaw); i++)
			value_str[i-(startAnswer-getStatusRaw)] = getStatusRaw[i];
		memset(programButtonLastTime[0], NULL,50);
		memcpy(programButtonLastTime[0],value_str,sizeof(value_str));
//		ee24_write(&hi2c1,30,(uint8_t *)ButtonCondition[0],1,100);				// write New Program conditon in eeprom(0 or 1)
//		ee24_write(&hi2c1,40,(uint8_t *)programButtonLastTime[0],20,100); // Write time of last program in eeprom
		memset(ContentStr, NULL, size);
		snprintf(ContentStr,50,"Button1 New Program : %s - %s\n\r",ButtonCondition[0],programButtonLastTime[0] );
		DEBUG(ContentStr);

		// Extracting Button_2 New Program Status :
		startAnswer	= strstr(getStatusRaw, "\"n\":2")+11;									// get Button_2 condition
		ButtonCondition[1][0] = getStatusRaw[(startAnswer - getStatusRaw)];
		startAnswer	= strstr(getStatusRaw, "\"n\":2")+19;									// get Button_2 Last_time NEW_Program
		endAnswer		= strstr(getStatusRaw, "\"n\":2")+36;
		for(int i = (startAnswer - getStatusRaw); i < (endAnswer - getStatusRaw); i++)
			value_str[i-(startAnswer-getStatusRaw)] = getStatusRaw[i];
		memset(programButtonLastTime[1], NULL,50);
		memcpy(programButtonLastTime[1],value_str,sizeof(value_str));
//		ee24_write(&hi2c1,31,(uint8_t *)ButtonCondition[1],1,100);				// write New Program conditon in eeprom(0 or 1)
//		ee24_write(&hi2c1,60,(uint8_t *)programButtonLastTime[1],20,100);	// Write time of last program in eeprom
		memset(ContentStr, NULL, size);
		snprintf(ContentStr,50,"Button2 New Program : %s - %s\n\r",ButtonCondition[1],programButtonLastTime[1] );
		DEBUG(ContentStr);
		
		#if(BUTTONS_NUM==3)
		// Extracting Button_3 New Program Status :
		startAnswer	= strstr(getStatusRaw, "\"n\":3")+11;									// get Button_3 condition
		ButtonCondition[2][0] = getStatusRaw[(startAnswer - getStatusRaw)];
		startAnswer	= strstr(getStatusRaw, "\"n\":3")+19;									// get Button_3 Last_time NEW_Program
		endAnswer		= strstr(getStatusRaw, "\"n\":3")+36;
		for(int i = (startAnswer - getStatusRaw); i < (endAnswer - getStatusRaw); i++)
			value_str[i-(startAnswer-getStatusRaw)] = getStatusRaw[i];
		memset(programButtonLastTime[2], NULL,50);
		memcpy(programButtonLastTime[2],value_str,sizeof(value_str));
//		ee24_write(&hi2c1,32,(uint8_t *)ButtonCondition[2],1,100);				//write New Program conditon in eeprom(0 or 1)
//		ee24_write(&hi2c1,80,(uint8_t *)programButtonLastTime[2],20,100); //Write time of last program in eeprom
		memset(ContentStr, NULL, size);
		snprintf(ContentStr,50,"Button3 New Program : %s - %s\n\r",ButtonCondition[2],programButtonLastTime[2] );
		DEBUG(ContentStr);
		#endif
		
		#if(BUTTONS_NUM==2)
		if((ButtonCondition[0][0] != '0') || (ButtonCondition[1][0] != '0')  || (GetProcessProgram_IS_Success == 0))
			proccesProgramIsChanged=1;
		else
			proccesProgramIsChanged=0;
		#endif	
		
		#if(BUTTONS_NUM==3)
		if((ButtonCondition[0][0] != '0') || (ButtonCondition[1][0] != '0') || (ButtonCondition[2][0] != '0') || (GetProcessProgram_IS_Success == 0))
			proccesProgramIsChanged=1;
		else
			proccesProgramIsChanged=0;
		#endif
		
		#if(BUTTONS_NUM==4)
		if((ButtonCondition[0][0] != '0') || (ButtonCondition[1][0] != '0') || (ButtonCondition[2][0] != '0') || (ButtonCondition[3][0] != '0')|| (GetProcessProgram_IS_Success == 0))
			proccesProgramIsChanged=1;
		else
			proccesProgramIsChanged=0;
		#endif
		// Extracting Buttons State from Server
		startAnswer	= strstr(getStatusRaw, "\"ebc\":")+7;
		endAnswer		= strstr(getStatusRaw, ",\"pbc\"") -1;	
		memset(value_str, NULL, 30);
		for(int i = (startAnswer - getStatusRaw); i < (endAnswer - getStatusRaw); i++)
			value_str[i-(startAnswer-getStatusRaw)] = getStatusRaw[i];
		memset(ServerButtonState, NULL,2);
		memcpy(ServerButtonState,value_str,sizeof(value_str));
		memset(ContentStr, NULL, size);
		snprintf(ContentStr,50,"Buttons State from Server : %s \n\r",ServerButtonState);
		DEBUG(ContentStr);
		
		GetStatus_Is_Success = 1; 	// set flage true for action in main
		if(((ServerButtonState[0]-'0') != changeButtonStatusTimeStamp[0][1]) && (Button_Actor[0] == 0))		// check Button status change from server
		{
			counter = 2000001;
			HAL_GPIO_EXTI_Callback(SW1_Pin);																		// call externall interrupt fuction call back
		}
		if(((ServerButtonState[1]-'0') != changeButtonStatusTimeStamp[1][1]) && (Button_Actor[1] == 0))		// check Button status change from server
		{
			Button_Actor[1] = 0;
			counter = 2000001;
			HAL_GPIO_EXTI_Callback(SW2_Pin);																		// call externall interrupt fuction call back
		}
	#if(BUTTONS_NUM==3)	
		if(((ServerButtonState[2]-'0') != changeButtonStatusTimeStamp[2][1]) && (Button_Actor[2] == 0))		// check Button status change from server
		{
			Button_Actor[2] = 0;
			counter = 2000001;
			HAL_GPIO_EXTI_Callback(SW3_Pin);																		// call externall interrupt fuction call back
		}
	#endif	
		
		
		
//		counter = 2000001;
//		HAL_GPIO_EXTI_Callback(SW1_Pin);
//		ee24_read(&hi2c1,30,(uint8_t *)ButtonCondition[0],1,100);
//		ee24_read(&hi2c1,40,(uint8_t *)programButtonLastTime[0],20,100);
//		memset(ContentStr, NULL, size);
//		snprintf(ContentStr,50,"Button1 New Program : %s - %s\n\r",ButtonCondition[0],programButtonLastTime[0] );
//		DEBUG(ContentStr);
	}
	else if(strstr((char *)sendStatus, "ok"))
	{
		DEBUG("Server respose is Ok");
	}
	else
	{
		DEBUG("ERROR in get status");
		GetStatus_Is_Success = 0;
	}
}
else
{
	DEBUG("Repeat getStatus with respose:True");
	GetStatus_Is_Success = 1;
}


	#if(__IS_OLED__==1)
		HAL_TIM_Base_Start_IT(&htim5);
	#endif

}

/**
  * @brief  Gets the land ID according to serial number from the server
  * @retval The land ID. If the returned value was 0 it means the request was failed.
  */
uint16_t GetID(void){
	char *startAnswer;
	char *endAnswer;
	uint16_t output = 0;
	
	memset(ContentStr,NULL,size);
	snprintf(ContentStr,size,"{\"serial\":\"%s\"}\r\n", __SERIAL_NUMBER);
	DEBUG(ContentStr);
	sim80x_HTTP_Post(ContentStr, SERVER_IP,"get_land_by_device_serial/",ContentStr);
	
	if(strstr((char *)ContentStr,"land") != NULL){
		startAnswer = strstr(ContentStr, "land\":")+6;
		endAnswer  = strstr(ContentStr, "}");
		
	char out[30];
	memset(out, NULL, 30);
	for(int i=(startAnswer-ContentStr); i<(endAnswer-ContentStr); i++)
		out[i-(startAnswer-ContentStr)] = ContentStr[i];
	output = (uint16_t)atoi(out);
	}
	
	return output;
}/* 0 */uint8_t  ID;
/**
  * @brief  Writes a program in EEPROM
  * @param  prog is a pointer to your desired processProgram struct
  */
void writeProcessProg(ProcessProgram* processProg){ ///////////NEW ADDED/////////////////
	if(ee24_isConnected(&hi2c1))
	{
		//fill 	tempData with processprog data 
		uint8_t tempData[60];
		tempData[0] = processProg->ID;
		tempData[51] = processProg->events_count;
		for(int i=1;i<=processProg->events_count;i++)
		{
			tempData[i]    = processProg->Hour[i-1];
			tempData[10+i] = processProg->Minutes[i-1];
			tempData[20+i] = processProg->Second[i-1];
			tempData[30+i] = processProg->spouts[i-1][0];
			tempData[40+i] = processProg->spouts[i-1][1];			
		}

		ee24_write(&hi2c1, (processProg->ID - 1) * EE24_WRITE_COEFFICIENT_PROS + EE24_START_ADDRESS_PROCESS_PROGRAM, tempData, 52, 2000);//write tempdata into the eeprom

	}
	else
		DEBUG("EEPROM ERROR");
}


/**
  * @brief  Writes a program in EEPROM
  * @param  prog is a pointer to your desired Program struct
	* @param  is_interrupt: if you want to call this function in an interrupt routin you have to set this value as 1. Otherwise, set this 0.
  */
void writeProg(Program* prog){ ///////////NEW ADDED/////////////////
	if(ee24_isConnected(&hi2c1)){
		uint8_t tempData[11];
		
		tempData[0] = prog->ID;
		tempData[1] = prog->Month;
		tempData[2] = prog->Day;
		tempData[3] = prog->Hour;
		tempData[4] = prog->Minutes;
		tempData[5] = prog->PeriodDay;
		tempData[6] = prog->PeriodHour;
		tempData[7] = prog->PeriodMinute;
		tempData[8] = prog->Action[0];
		tempData[9] = prog->Action[1];
		tempData[10] = prog->has_interval;
		
		ee24_write(&hi2c1, ((prog->ID)-10)*EE24_WRITE_COEFFICIENT_PROG+EE24_START_ADDRESS_PROGRAM, tempData, 11, 2000);
	}
	else
		DEBUG("EEPROM ERROR");
}
/**
  * @brief  Reads a Processprogram from EEPROM.
  * @param  progID is your desired program ID( the number of button).
  * @retval processProgram struct.
  */
ProcessProgram readProcessProg(uint8_t progID){ ///////////NEW ADDED/////////////////
	ProcessProgram processProg;
	
	processProg.ID 					= NULL;
	processProg.events_count	= NULL;
	memset(processProg.Hour,NULL,10);
	memset(processProg.Minutes,NULL,10);
	memset(processProg.Second	,NULL,10);
	memset(processProg.spouts,NULL,10);
	
	if(ee24_isConnected(&hi2c1))
	{
		uint8_t tempData[52];
		
		ee24_read(&hi2c1, (progID-1) * EE24_WRITE_COEFFICIENT_PROS + EE24_START_ADDRESS_PROCESS_PROGRAM, tempData, 52, 2000);//read process program from eeprom and pour it into the tempData
		
		//fill processProg struct with tempdata arrays data:
		processProg.ID 					= tempData[0];
		processProg.events_count = tempData[51]; 
		for(int i = 1; i <= processProg.events_count; i++)
			{
				processProg.Hour[i - 1] = tempData[i] ;
				processProg.Minutes[i - 1]=	tempData[10 + i];
				processProg.Second[i - 1] = tempData[20 + i];
				processProg.spouts[i - 1][0] = tempData[30 + i];	
				processProg.spouts[i - 1][1] = tempData[40 + i];	
			}
	}
	else
		DEBUG("EEPROM ERROR");
	return processProg;
}



/**
  * @brief  Reads a program from EEPROM.
  * @param  progID is your desired program ID.
  * @retval Program struct.
  */
Program readProg(uint8_t progID){ ///////////NEW ADDED/////////////////
	Program output;
	output.ID 					= NULL;
	output.Month				= NULL;
	output.Day					= NULL;
	output.Hour					= NULL;
	output.Minutes			= NULL;
	output.PeriodDay		= NULL;
	output.PeriodHour		= NULL;
	output.PeriodMinute	= NULL;
	output.Action[0]		= NULL;
	output.Action[1]		= NULL;
	output.has_interval	= NULL;
	
	if(ee24_isConnected(&hi2c1)){
		
		uint8_t tempData[11];
		
		ee24_read(&hi2c1, ((progID) - 10)*EE24_WRITE_COEFFICIENT_PROG+EE24_START_ADDRESS_PROGRAM, tempData, 11, 2000);
		
		output.ID 					= tempData[0];
		output.Month 				= tempData[1]; 
		output.Day 					= tempData[2];
		output.Hour					= tempData[3];
		output.Minutes			= tempData[4];
		output.PeriodDay		= tempData[5];
		output.PeriodHour		= tempData[6];
		output.PeriodMinute	= tempData[7];
		output.Action[0]		= tempData[8];
		output.Action[1]		= tempData[9];
		output.has_interval	= tempData[10];
	}
	else
		DEBUG("EEPROM ERROR");
	return output;
}
 /**
	* @brief  Delete All Previous process programs stored in EEPROM.
  * @param  prog is a pointer to your desired ProcessProgram struct
*/
void deleteAllProcessPrograms(void){ ///////////NEW ADDED/////////////////
	if(ee24_isConnected(&hi2c1))
	{
	
//		for(int i=0;i<1000;i++)
//			ee24_write(&hi2c1, PROCESS_PROG_START_ADD +i , (uint8_t *)0xff, 1, 1);

	uint8_t values[EE24_WRITE_COEFFICIENT_PROS * MAX_NUM_PROCESS_EVENT];
	memset(values, 255, EE24_WRITE_COEFFICIENT_PROS * MAX_NUM_PROCESS_EVENT);
	ee24_write(&hi2c1, EE24_START_ADDRESS_PROCESS_PROGRAM , values, EE24_WRITE_COEFFICIENT_PROS * MAX_NUM_PROCESS_EVENT, 1000);//Delete All Previous process programs. Address: from PROCESS_PROG_START_ADD to PROCESS_PROG_START_ADD+1000
		
	}
	else
		DEBUG("EEPROM ERROR");	

}
/**
  * @brief  Delete a programs stored in EEPROM.
  * @param  prog is a pointer to your desired Program struct
	* @param  is_interrupt: if you want to call this function in an interrupt routin you have to set this value as 1. Otherwise, set this 0.
  */
void deleteProg(Program* prog){ ///////////NEW ADDED/////////////////
	if(ee24_isConnected(&hi2c1)){
		uint8_t tempData[11];
		
		tempData[0] = 0xFF;
		tempData[1] = 0xFF;
		tempData[2] = 0xFF;
		tempData[3] = 0xFF;
		tempData[4] = 0xFF;
		tempData[5] = 0xFF;
		tempData[6] = 0xFF;
		tempData[7] = 0xFF;
		tempData[8] = 0xFF;
		tempData[9] = 0xFF;
		tempData[10] = 0xFF;

		ee24_write(&hi2c1, ((prog->ID) - 10) * EE24_WRITE_COEFFICIENT_PROG+EE24_START_ADDRESS_PROGRAM, tempData, 11, 2000);
	}
	else
		DEBUG("EEPROM ERROR");	
}

/**
  * @brief  Sends a request to get all process programs from sever, parse the programs and store them in EEPROM.
  */
uint8_t GetAllProcessPrograms(void){

	char*		startAnswer;
	char*		endAnswer;
	char    processProgram_raw[size];//used in separating processProgram of evrery buttons
	int 		buttonsNum; 
	getProcessProgramsStarting = 1;

	ProcessProgram processProg;
	HAL_Delay(2000);
	#if(__IS_OLED__==1)
	HAL_TIM_Base_Stop_IT(&htim5);//stop timer interrupt before writeProcessProg in eeprom
	#endif
	memset(ContentStr,NULL,size);
	snprintf(ContentStr,size,"{\"deviceId\":%d}\r\n", deviceId);									// INPUT FORMAT --> { "land" : xx }
	DEBUG(ContentStr);
	
	// start GET processPrograms from server:
	
	snprintf(str,sizeof(str),"AT+HTTPPARA=\"URL\",\"http://%s/device_processes/%d/\"\r\n",SERVER_IP, deviceId);//process prpgram api
	if(	sim80x_ATC(str,1000)== HAL_OK)
	{
		if(sim80x_ATC("AT+HTTPACTION=0\r\n",10000) == HAL_OK)
		{
			if(sim80x_ATC("AT+HTTPREAD\r\n",2000) == HAL_OK)
			{	
				memset(ContentStr, NULL, size);
				memcpy(ContentStr, RxBuffer, size);					
				
			// end GET processPrograms from server
				if(strstr((char *)ContentStr, "events_count") != NULL)
				{
					  getProcessProgramsStatus = 0;//Get process program  successful /Unsuccessful flag
						deleteAllProcessPrograms(); //delete All previous process programs from eeprom
						buttonsNum = 0;
						memcpy(ContentStr, ContentStr+30, size);  // Delete sim800 contents from server response and Fetch pure JSON (complete later)
						while(buttonsNum < BUTTONS_NUM) // continue until processProgram of every buttons is parsed.
						{
							//remove SIM800 commands and OK in response:
							startAnswer	= strstr(ContentStr, "{\"number\":");
							endAnswer		= strstr(ContentStr, "\"events_count\"") + 18;	
							memset(processProgram_raw, NULL, size);
							for(int i = (startAnswer - ContentStr); i < (endAnswer - ContentStr); i++)
								processProgram_raw[i-(startAnswer-ContentStr)] = ContentStr[i]; // separating processProgram of every buttons
							DEBUG("\n\r*************\n\r");
							DEBUG(processProgram_raw);
							processProg = processProgramParser(processProgram_raw);	//	parsing the parameter of every processProgram including : number(ID), events, events_count
							
							writeProcessProg(&processProg);// write processPrograms parametter in eeprom
							DEBUG("\n\r*************\n\r");
							memcpy(ContentStr, ContentStr+strlen(processProgram_raw), size);
							buttonsNum++;
						}

					for(buttonsNum = 1; buttonsNum <= BUTTONS_NUM ; buttonsNum++)
						PrintAllProcessProgram(buttonsNum);// Prints all valid Process programs stored in EEPROM.	
						#if(__IS_OLED__==1)
							HAL_TIM_Base_Start_IT(&htim5);//start timer interrupt before writeProcessProg in eeprom
						#endif
						getProcessProgramsStatus = 1;//Get process program  successful
						getProcessProgramsStarting = 0;			
						HAL_Delay(1000);						
						return  getProcessProgramsStatus;	
				}
				
				else if(strstr((char *)ContentStr, "[]") != NULL)
				{
					DEBUG("\n\r --- There is no process program in server --- \n\r");
					getProcessProgramsStatus = 0;//Get process program  unsuccessful 
					getProcessProgramsStarting = 0;	
						#if(__IS_OLED__ == 1)
							HAL_TIM_Base_Start_IT(&htim5);//start timer interrupt
						#endif
					
					HAL_Delay(1000);
					return  getProcessProgramsStatus;
				}
				
				else
				{
					DEBUG("\n\r --- Erorr in getAllprocessPrograms --- \n\r");
					#if(__IS_OLED__ == 1)
						HAL_TIM_Base_Start_IT(&htim5);//start timer interrupt
					#endif
					getProcessProgramsStatus = 0;//Get process program  unsuccessful 
					getProcessProgramsStarting = 0;	
					HAL_Delay(1200);
					return  getProcessProgramsStatus;
				}
				
			}
			else getProcessProgramsStatus = 0;//Get process program  unsuccessful 
		}	
		else
			getProcessProgramsStatus = 0;//Get process program  unsuccessful 
	}
	else
		getProcessProgramsStatus = 0;//Get process program  unsuccessful 
	
	getProcessProgramsStarting = 0;
	#if(__IS_OLED__ == 1)
		HAL_TIM_Base_Start_IT(&htim5);//start timer interrupt
	#endif
	HAL_Delay(1000);
return  getProcessProgramsStatus;	

	
}	

/**
  * @brief  Sends a request to get all programs from sever, parse the programs and store them in EEPROM.
  */
void GetAllPrograms(void){
	char*		startAnswer;
	char*		endAnswer;
	Program temp;
	getProgramsStarting=1;
	HAL_Delay(1000);
	#if(__IS_OLED__==1)
		HAL_TIM_Base_Stop_IT(&htim5);
	#endif
	memset(ContentStr,NULL,size);

	snprintf(ContentStr,size,"{\"device\":%d}\r\n", deviceId);									// INPUT FORMAT --> { "device" : xx }
	DEBUG(ContentStr);
	sim80x_HTTP_Post(ContentStr, SERVER_IP,"pd_list/",ContentStr);
//	memcpy(ContentStr, "[{\"pk\":\"10\",\"start\":\"2021/07/12,23:06:16\",\"interval\":\"7 00:00:00\",\"action\":\"A1\",\"has_interval\":true},{\"pk\":\"11\",\"start\":\"2021/07/13,00:06:16\",\"interval\":\"7 00:00:00\",\"action\":\"5C\",\"has_interval\":true}]", sizeof("[{\"pk\":\"10\",\"start\":\"2021/07/12,23:06:16\",\"interval\":\"7 00:00:00\",\"action\":\"A1\",\"has_interval\":true},{\"pk\":\"11\",\"start\":\"2021/07/13,00:06:16\",\"interval\":\"7 00:00:00\",\"action\":\"5C\",\"has_interval\":true}]"));
	if(strstr((char *)ContentStr,"has_interval") != NULL){
		
		//remove SIM800 commands and OK in response:		
		startAnswer	= strstr(ContentStr, "[")+1;
		endAnswer		= strstr(ContentStr, "]")+1;
		
		memset(str, NULL, size);
		for(int i=(startAnswer-ContentStr); i<(endAnswer-ContentStr); i++)
			str[i-(startAnswer-ContentStr)] = ContentStr[i];
		memset(ContentStr, NULL, size);
		memcpy(ContentStr, str, size);									// Fetching pure JSON from SIM800 response
		
		// Deleting previous programs:
		deleteAllPrograms();
		
		// Extract programs:
		DEBUG("All Programs:\n\r");
		
		char program_raw[110];		
		while(strstr((char *)ContentStr,"{") != NULL){			
			startAnswer = strstr(ContentStr, "{");
			endAnswer	  = strstr(ContentStr, "}")+1;
			
			memset(program_raw, NULL, 110);
			for(int i=(startAnswer-ContentStr); i<(endAnswer-ContentStr); i++)
				program_raw[i-(startAnswer-ContentStr)] = ContentStr[i];
			
			DEBUG("\n\r");
			DEBUG(program_raw);
			
			temp = ProgramParser(program_raw);
			writeProg(&temp);
			PrintProgram(temp);
			memcpy(ContentStr, ContentStr+strlen(program_raw)+1, size);	
			getProgramsStatus = 1;
		}
	}
	else if(strstr((char *)ContentStr,"[]") != NULL){
		DEBUG("\n\r --- There is no program in server --- \n\r");
		deleteAllPrograms();
		getProgramsStatus = 1;
		AlarmIsSet = 0;
	}
	else{
		DEBUG("\n\r --- Erorr in getAllPrograms --- \n\r");
		getProgramsStatus=0;
	}
	getProgramsStarting=0;
	#if(__IS_OLED__==1)
		HAL_TIM_Base_Start_IT(&htim5);
	#endif
	HAL_Delay(1000);
}


/**
  * @brief  Parse a string received from server to a processProgram struct.
	* @param  input_str is a JSON icluding one button process program like : "{"number":2,"events":[{"duration":"33.0","spouts":"010000"},{"duration":"12.0","spouts":"101100"},{"duration":"20.0","spouts":"111100"},{"duration":"15.0","spouts":"010011"}],"events_count":4}]
	* @retval processProgram struct
  */
ProcessProgram processProgramParser(char* input_str){
	char*	startAnswer;
	char*	endAnswer;
	char  value_str[200];// used in debuging Extracted JSON results
	char	json_result[20];	
	uint16_t  value_uint;
	//Creates an empty instance from processProgram struct by NULL values
	ProcessProgram processProg;
	processProg.ID     				= NULL;
	memset(processProg.duration,NULL,10);
	memset(processProg.Hour,NULL,10);
	memset(processProg.Minutes,NULL,10);
	memset(processProg.Second,NULL,10);
	memset(processProg.spouts	,NULL,10);
	processProg.events_count		= NULL;
// Extracting number  ==> "number ":xxx, ...
	if(JSON2int(json_result, input_str, "number"))
	{
		processProg.ID= (uint8_t)atoi(json_result);
		sprintf(value_str, "\n\r   ID : \"%d\"", processProg.ID);
		DEBUG(value_str);
	}
	// Extracting events_count  ==> "events_count ":xxx, ...

	if(JSON2int(json_result, input_str, "events_count"))
	{
		processProg.events_count  = (uint8_t)atoi(json_result);
		sprintf(value_str, "\n\r   events_count : \"%d\"", processProg.events_count);
		DEBUG(value_str);
	}
	DEBUG("\n\r");

	// Extracting events:
	memset(str,NULL,size);
	startAnswer = strstr(input_str, "events")+9;
	endAnswer	  = strstr(input_str, "]")+1;
	for(int i=(startAnswer-input_str); i<(endAnswer-input_str); i++)
		str[i-(startAnswer-input_str)] = input_str[i];
	
// Extracting duration & spouts in every events  ==> "duration ":xxx, ...

	for(int i=0;i<processProg.events_count;i++)
	{
		
		memset(processProgram,NULL,size);
		startAnswer = strstr(str, "{");
		endAnswer	  = strstr(str, "}")+1;
		for(int j=(startAnswer-str); j<(endAnswer-str); j++)
				processProgram[j-(startAnswer-str)] = str[j];	
		if(JSON2int(json_result, processProgram, "delay"))
			{
				processProg.duration[i]	= (uint32_t)atoi(json_result);
				
				//calculate Hour,Minutes,Second of every events from the received "duration(s)"
				processProg.Hour[i]=processProg.duration[i]/3600;
				processProg.Minutes[i]=processProg.duration[i]/60-processProg.Hour[i]*60;
				processProg.Second[i]=processProg.duration[i]-processProg.Minutes[i]*60-processProg.Hour[i]*3600;
				sprintf(value_str, "\n\r  event:%d Time duration   :  %02d:%02d:%02d ",i,processProg.Hour[i], processProg.Minutes[i],processProg.Second[i]);
				DEBUG(value_str);
			}			
		if(JSON2Str(json_result, processProgram, "spouts"))
			{	
		    value_uint= str2dec(json_result);				// value_str is string like : "02102310".it is include base 4 number in string type and we Converts it to decimal number in uint16_t type 
				processProg.spouts[i][0]=value_uint%256; //Convert value_uint decimal number to a base 256 number (2byte)
				processProg.spouts[i][1]=value_uint/256;

				sprintf(value_str, "\n\r  event:%d  spouts  : \"%s\"",i,json_result);
				DEBUG(value_str);
			}
		memcpy(str, str+strlen(processProgram)+1, size);
		DEBUG("\n\r");
	}
		

	return processProg;

}

/**
  * @brief  Parse a string received from server to a Program struct.
  * @param  input_str is a string like " ... "
	* @retval Program struct
  */

Program ProgramParser(char* input_str){
	char*	startAnswer;
	char*	endAnswer;
	char  value_str[50];
	uint16_t  value_uint;
	char	json_result[20];
		
	//Creates an empty instance from Program struct by NULL values
	Program output;
	output.ID 					= NULL;
	output.Month				= NULL;
	output.Day					= NULL;
	output.Hour					= NULL;
	output.Minutes			= NULL;
	output.PeriodDay		= NULL;
	output.PeriodHour		= NULL;
	output.PeriodMinute	= NULL;
	output.Action[0]		= NULL;
	output.Action[1]		= NULL;
	output.has_interval	= NULL;
	
	// Extracting ID ==> "id":xxx, ...		
		if(JSON2Str(json_result, input_str, "pk")){
			sprintf(value_str, "\n\r  Extracted ID is \"%d\"", atoi(json_result));
			DEBUG(value_str);
			output.ID = atoi(json_result);
		}
	
	//Extract Time ==> ... ,"start":"yyyy/MM/dd,hh:mm:ss", ...	
		if(JSON2Str(json_result, input_str, "start")){
			sprintf(value_str, "\n\r  Extracted Time is \"%s\"", json_result);
			DEBUG(value_str);
			
			//	| y | y | y | y | / | M | M | / | d | d | ,  | h  | h  | :  | m  | m  | :  | s  | s  |
			//  | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 		
			output.Month		= (uint8_t)( atoi(&json_result[5]  ));
			output.Day			= (uint8_t)( atoi(&json_result[8]  ));
			output.Hour			= (uint8_t)( atoi(&json_result[11] ));
			output.Minutes	= (uint8_t)( atoi(&json_result[14] ));
		}
		
	//Extract Raw Interval ==> ... ,"interval":"d hh:mm:ss:", ...
		if(JSON2Str(json_result, input_str, "interval")){
			sprintf(value_str, "\n\r  Extracted interval is \"%s\"\n\r", json_result);
			DEBUG(value_str);
			
			sprintf(value_str, "\"%s\"", json_result);
			
				//Extract Period Day ==> "ddd ...
					char  interval_str[10];
					
					startAnswer = strstr(value_str, "\"")+1;
					endAnswer  	= strstr(value_str, " ");
					
					memset(interval_str, NULL, 10);
					for(int i=(startAnswer-value_str); i<(endAnswer-value_str); i++)
						interval_str[i-(startAnswer-value_str)] = value_str[i];
					
					DEBUG("   -- Extracted Period Day is ");
					DEBUG(interval_str);
					DEBUG("\n\r");
					
					output.PeriodDay = (uint8_t)(atoi(interval_str));
				
				//Extract Period Hour and Minutes ==> ... hh:mm:ss"
					startAnswer = strstr(value_str, " ")+1;
					endAnswer  	= strstr(value_str+1, "\""); // +1 for ignoring first "
					
					memset(interval_str, NULL, 10);
					for(int i=(startAnswer-value_str); i<(endAnswer-value_str); i++)
						interval_str[i-(startAnswer-value_str)] = value_str[i];
					
					DEBUG("   -- Extracted Period Time is ");
					DEBUG(interval_str);
					DEBUG("\n\r");
					
					output.PeriodHour		= (uint8_t)(atoi(&interval_str[0]));
					output.PeriodMinute	= (uint8_t)(atoi(&interval_str[3]));
			}
		
	//Extract Action ==> ... ,"action":"xx", ...
		if(JSON2Str(json_result, input_str, "action")){
			sprintf(value_str, "\n\r  Extracted action is \"%s\"\n\r", json_result);
			DEBUG(value_str);
			///////////NEW ADDED/////////////////
		  value_uint= str2dec(json_result);// value_str is string like : "02102310".it is include base 4 number in string type and we Converts it to decimal number in uint16_t type 
			output.Action[0]=value_uint%256; //Convert value_uint decimal number to a base 256 number (2byte)
			output.Action[1]=value_uint/256;
			/////////////////////////////////
			
	  	//memcpy(output.Action, json_result,2);
		}
	
	//Extract has_interval ==> ... ,"has_interval":true/false}			
		if(JSON2Str(json_result, input_str, "has_interval")){
			sprintf(value_str, "\n\r  Extracted has_interval is \"%s\"\n\r", json_result);
			DEBUG(value_str);
		}
		
		if(!strcmp(json_result, "false")){
			output.PeriodDay		= 0x00;
			output.PeriodHour		= 0x00;
			output.PeriodMinute	= 0x00;
			output.has_interval = 0x00;
		}
		else{
			output.has_interval = 1;
		}
		
	return output;
}

/**
  * @brief  Apply output on GPIOs
	* @param  status is a number between 0 and 15 representing the outputs. For example 13 means: | 1 | 1 | 0 | 1 |
  */
void ApplyAction(uint8_t status[2]){ ///////////NEW ADDED/////////////////
	/*
		* Tozih:
			voroodi in tabe, yek adad beyne 0 ta 15 ast. bayad tabdil be binarry beshe. har bitesh male yek khorooji ast.
			masalan vorrodi hast 12,  Outputs ha injoori mishan: Output[0] = 1, Output[1] = 1, Output[2] = 0, Output[3] = 0
			nahveye map shodane har bit be har kanale relay tooye oonja ke WritePin kardam ghabele thagyeere.
	*/

	memset( outputsStatus, 0,OUTPUTS_NUM);
	decimalOutputs= status[0]+status[1]*256; //tabdil adad mabnaye 256 status be yek adad decimal 
	for(uint8_t i=0;i<OUTPUTS_NUM;i++)  // change from decinal to base 4 number
	{
	 if(i!=OUTPUTS_NUM-1)
		outputsStatus[i]=decimalOutputs%4;
	 else
		outputsStatus[OUTPUTS_NUM-1]=decimalOutputs;
	 
   decimalOutputs=decimalOutputs/4;
	}
//	 memset(str,NULL,size); 
//	sprintf(str,"\n\r ###########   outputs: %d %d %d %d %d %d %d %d\n\r ACTION APPLIED\n\r ",outputsStatus[7],outputsStatus[6],outputsStatus[5],outputsStatus[4],outputsStatus[3],outputsStatus[2],outputsStatus[1],outputsStatus[0]);
//	DEBUG(str);
	//  Apply outputs according to	OUTPUTS_NUM that is defined before		:	

	#if(OUTPUTS_NUM==2)
	
	 if(outputsStatus[0]==1)
		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[0]==0)
		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[0]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[1]==1)
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[1]==0)
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[1]==2)
	 { 
		 //NOP
	 }

	DEBUG("\n\r **Action applied --> ");
	memset(action_str, NULL, 20);
	snprintf(action_str, 50, "| %d | %d | \n\r" ,outputsStatus[0],outputsStatus[1]);
	DEBUG(action_str);
	 
	
	#endif
	
	#if(OUTPUTS_NUM==3)
	 if(outputsStatus[0]==1)
		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[0]==0)
		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[0]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[1]==1)
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[1]==0)
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[1]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[2]==1)
		HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[2]==0)
		HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[2]==2)
	 { 
		 //NOP
	 }

	DEBUG("\n\rAction applied --> ");
	memset(action_str, NULL, 20);
	snprintf(action_str, 50, "| %d | %d | %d | \n\r" ,outputsStatus[0],outputsStatus[1],outputsStatus[2]);
	DEBUG(action_str);
	#endif
	
	
	
	#if(OUTPUTS_NUM==4)
	 if(outputsStatus[0]==1)
		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[0]==0)
		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[0]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[1]==1)
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[1]==0)
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[1]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[2]==1)
		HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[2]==0)
		HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[2]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[3]==1)
		HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[3]==0)
		HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[3]==2)
	 { 
		 //NOP
	 }
	DEBUG("\n\rAction applied --> ");
	memset(action_str, NULL, 20);
	snprintf(action_str, 50, "| %d | %d | %d | %d | \n\r" ,outputsStatus[0],outputsStatus[1],outputsStatus[2],outputsStatus[3]);
	DEBUG(action_str);
	#endif

	#if(OUTPUTS_NUM==6)
	 if(outputsStatus[0]==1)
		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[0]==0)
		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[0]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[1]==1)
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[1]==0)
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[1]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[2]==1)
		HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[2]==0)
		HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[2]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[3]==1)
		HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[3]==0)
		HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[3]==2)
	 { 
		 //NOP
	 }
	 	 if(outputsStatus[4]==1)
		HAL_GPIO_WritePin(relay5_GPIO_Port, relay5_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[4]==0)
		HAL_GPIO_WritePin(relay5_GPIO_Port, relay5_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[4]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[5]==1)
		HAL_GPIO_WritePin(relay6_GPIO_Port, relay6_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[5]==0)
		HAL_GPIO_WritePin(relay6_GPIO_Port, relay6_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[5]==2)
	 { 
		 //NOP
	 }
	DEBUG("\n\rAction applied --> ");
	memset(action_str, NULL, 20);
	snprintf(action_str, 50, "| %d | %d | %d | %d | %d | %d  \n\r" ,outputsStatus[0],outputsStatus[1],outputsStatus[2],outputsStatus[3],outputsStatus[4],outputsStatus[5]);
	DEBUG(action_str);
	#endif
		#if(OUTPUTS_NUM==8)
	 if(outputsStatus[0]==1)
		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[0]==0)
		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[0]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[1]==1)
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[1]==0)
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[1]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[2]==1)
		HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[2]==0)
		HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[2]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[3]==1)
		HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[3]==0)
		HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[3]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[4]==1)
		HAL_GPIO_WritePin(relay5_GPIO_Port, relay5_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[4]==0)
		HAL_GPIO_WritePin(relay5_GPIO_Port, relay5_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[4]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[5]==1)
		HAL_GPIO_WritePin(relay6_GPIO_Port, relay6_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[5]==0)
		HAL_GPIO_WritePin(relay6_GPIO_Port, relay6_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[5]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[6]==1)
		HAL_GPIO_WritePin(relay7_GPIO_Port, relay7_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[6]==0)
		HAL_GPIO_WritePin(relay4_GPIO_Port, relay7_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[6]==2)
	 { 
		 //NOP
	 }
	 if(outputsStatus[7]==1)
		HAL_GPIO_WritePin(relay8_GPIO_Port, relay8_Pin, GPIO_PIN_SET );
	 else if(outputsStatus[7]==0)
		HAL_GPIO_WritePin(relay8_GPIO_Port, relay8_Pin, GPIO_PIN_RESET );
	 else if(outputsStatus[7]==2)
	 { 
		 //NOP 
	 }
	DEBUG("\n\rAction applied --> ");
	memset(action_str, NULL, 20);
	snprintf(action_str, 50, "| %d | %d | %d | %d | %d | %d | %d |%d | \n\r" ,outputsStatus[0],outputsStatus[1],outputsStatus[2],outputsStatus[3],outputsStatus[4],outputsStatus[5],outputsStatus[6],outputsStatus[7]);
	DEBUG(action_str);
	#endif
	 
	 

	
}
/**
  * @brief  Print a string in debug port
  * @param  input is the string that you want to print in UART
  */
void DEBUG(char* input){
	HAL_UART_Transmit(&debugUART, (uint8_t*)input, strlen(input), 1000);
}


/**
* 	@brief  This toutin first apply  the process programs event  that it is time to   and then set alarm for next event actions.
  * @param  hrtc handler
 *  @retval return 0 if The Botton "x"  process has been done
  */
uint8_t SetProcessProgramsAlarm(RTC_HandleTypeDef* rtc){
	
	ProcessProgram 	processProg;
	struct tm  			currentTime;
	uint8_t					WhatToDo[2];						//The outputs status that must be applyed
	uint8_t					filterNewStatus[2];
	uint32_t 				currentTimeStamp = 0;		//used when convert current time to time stamp
	uint32_t 				eventTimeStamp=0;
	processProg		= readProcessProg(buttonId+1);//read desired button process program from eeprom
	
	//calculate current time:
 HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
 HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
 currentTime.tm_year	= Date.Year + 2000  - 1900;
 currentTime.tm_mon		= Date.Month-1;
 currentTime.tm_mday	= Date.Date;
 currentTime.tm_hour	= Time.Hours;
 currentTime.tm_min		= Time.Minutes;
 currentTime.tm_sec		= Time.Seconds;
 currentTime.tm_isdst	= -1;	
 currentTimeStamp = mktime(&currentTime);//convert current time to time stamp
	
	if((eventNumber[buttonId] < processProg.events_count) && (buttonsStatus[buttonId] == 1))
	{
		WhatToDo[0] = processProg.spouts[eventNumber[buttonId]][0];//read outputs status of the event that it is time to.
		WhatToDo[1] = processProg.spouts[eventNumber[buttonId]][1];//read outputs status of the event that it is time to.
//		sprintf(str,"\n\r WhatToDo[1]=%d WhatToDo[0]=%d  \n\r",WhatToDo[1],WhatToDo[0]);
//		DEBUG("\n\r################\n\r");
//		DEBUG(str);	
//		DEBUG("\n\rr################n\r");

			// set Output_Actor of each output that change in Process Promgram one
			decimalOutputs= WhatToDo[0]+WhatToDo[1]*256; //tabdil adad mabnaye 256 status be yek adad decimal 
			for(uint8_t i = 0; i<OUTPUTS_NUM; i++)  // change from decinal to base 4 number
			{
				if(decimalOutputs%4 != 2)
					Output_Actor[i] = 1;						// if buttonState = 1 -> set the Output_Actor one
				decimalOutputs=decimalOutputs/4;
			}


		
		ApplyAction(WhatToDo); //apply the outputs status
		memset(str,NULL,size); 
		sprintf(str,"\n\r **Button %d , event %d ,event duration: %02d:%02d:%02d ,Total events %d \n\r ",buttonId +1,eventNumber[buttonId]+1,processProg.Hour[eventNumber[buttonId]],processProg.Minutes[eventNumber[buttonId]] ,processProg.Second[eventNumber[buttonId]],processProg.events_count);
		DEBUG(str);
		
		//Calculate event Time Stamp:
	 HAL_RTCEx_BKUPWrite(&hrtc, LAST_CURRENT_TIME_STAMP, currentTimeStamp);//stroe currentTimeStamp into eeprom
	 eventTimeStamp = currentTimeStamp + processProg.Minutes[eventNumber[buttonId]]*60 + processProg.Hour[eventNumber[buttonId]]*3600 + processProg.Second[eventNumber[buttonId]];// Add current time stamp to event duration in sec to make event Time Stamp// Add current time stamp to event duration in sec to make event Time Stamp
	 eventsTurn[buttonId] = eventTimeStamp;

	}
	else if((buttonsStatus[buttonId] == 0) || (eventNumber[buttonId] >= processProg.events_count))
	{
		memset(filterNewStatus,0,2);
		if(buttonsStatus[buttonId] == 0)
		{
			WhatToDo[0] = processProg.spouts[eventNumber[buttonId]][0];//read outputs status of the event that it is time to.
			WhatToDo[1] = processProg.spouts[eventNumber[buttonId]][1];//read outputs status of the event that it is time to.
		}
		else if(eventNumber[buttonId] >= processProg.events_count)
		{
			WhatToDo[0] = processProg.spouts[eventNumber[buttonId]-1][0];//read outputs status of the event that it is time to.
			WhatToDo[1] = processProg.spouts[eventNumber[buttonId]-1][1];//read outputs status of the event that it is time to.
			
			//save last last time and last status of the button its status have been changed:
		  changeButtonStatusTimeStamp[buttonId][0] = currentTimeStamp;
			
			
			/////////////// S NEW CODE  ////////////////////////
			DEBUG("\r\n\r\n\r\n-------------------------------------------------\r\n");
			snprintf(ContentStr,size,"|         Button %d Status has been changed      |",buttonId+1);
			DEBUG(ContentStr);
			DEBUG(				"\r\n-------------------------------------------------\r\n");		
				Button_Actor[buttonId] = 1;		
				DEBUG("\n\r **Buttons Status: ");
				memset(str, NULL, size);
				for(uint8_t i = 0; i < BUTTONS_NUM; i++)
				{
					sprintf(str, "%d", buttonsStatus[i]);
					DEBUG(str);
				}
				memset(ContentStr,NULL,size);
				snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d    \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
				DEBUG(ContentStr);
				DEBUG("\n\r**Process Programs Alarm is disables \n\r");
				/////////////// S NEW CODE  ////////////////////////
		}
		filterNewStatus[0] = WhatToDo[0] & 170;
		filterNewStatus[1] = WhatToDo[1] & 170;
//		sprintf(str,"\n\r WhatToDo[1]=%d WhatToDo[0]=%d   filterNewStatus[1]=%d filterNewStatus[0]=%d\n\r",WhatToDo[1],WhatToDo[0],filterNewStatus[1],filterNewStatus[0]);
//		DEBUG("\n\r################\n\r");
//		DEBUG(str);	
//		DEBUG("\n\rr################n\r");
		eventsTurn[buttonId] = 0;
		buttonsStatus[buttonId] = 0;	
    eventNumber[buttonId] = 0;	
	  changeButtonStatusTimeStamp[buttonId][1] = buttonsStatus[buttonId];		
		//processProgramsAlarmIsSet=0;//Alarm disable
		if(processProg.events_count!=0)
		{
			
			// set Output_Actor of each output that change in Process Promgram one
			decimalOutputs= filterNewStatus[0]+filterNewStatus[1]*256; //tabdil adad mabnaye 256 status be yek adad decimal 
			for(uint8_t i = 0; i < OUTPUTS_NUM; i++)  // change from decinal to base 4 number
			{
				if(decimalOutputs%4 != 2)
					Output_Actor[i] = 0;						// if buttonState = 0 -> set the Output_Actor zero
				decimalOutputs=decimalOutputs/4;
			}


			ApplyAction(filterNewStatus);		
			memset(ContentStr,NULL,size);
			snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d    \n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds );
			DEBUG(ContentStr);
      switch (buttonId)
				{
					case 0:
						HAL_GPIO_WritePin(SW_Light1_GPIO_Port,SW_Light1_Pin,0);
					break;
					case 1:
						HAL_GPIO_WritePin(SW_Light2_GPIO_Port,SW_Light2_Pin,0);
					break;
					
				}
			memset(str,NULL,size);
		  sprintf(str,"\n\r **The Button %d Process has been done\n\r",buttonId+1);
		  DEBUG(str);
		}
		else
		{
			memset(str,NULL,size);
		  sprintf(str,"\n\r **The Button %d does not have any process programs\n\r",buttonId+1);
		  DEBUG(str);
		}
	}
	Alarm_B_occur[buttonId] = 0; // for fixing #Bug 7
	updateNextEvent();
	if(nextEventTimeStamp != 0)
		SetNextAlarm_Processprograms(&hrtc);//set alarm for next event
	else
	{
		processProgramsAlarmIsSet = 0;
		memset(str,NULL,size);
		sprintf(str,"\n\r **All events have been ended \n\r");

		DEBUG(str);	
		
	}
	
	DEBUG("\n\r----------------<< DONE >>----------------  \n\r");


}
/**
  * @brief  This is a toutin that run a program in its time.
  * @param  hrtc handler
  */
void SetAlarm(RTC_HandleTypeDef* rtc){
	struct tm CurrentProgramTime;
	Program 	CurrentProgram;
	uint8_t 	CurrentProgramID = 0;
	uint32_t 	epoch=0;
	uint8_t		WhatToDo[2];
	char 			ConvertedTimeStamp[30];
	
	// 1- Ejraye hokm:
	CurrentProgramID	= HAL_RTCEx_BKUPRead(&hrtc, PROGRAM_ID_ADDRESS);
	CurrentProgram		= readProg(CurrentProgramID);
	WhatToDo[0]				= CurrentProgram.Action[0];///////////NEW ADDED/////////////////
	WhatToDo[1]				= CurrentProgram.Action[1];
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	memset(SMStext, NULL, 120);
	//snprintf(SMStext, 120, "ProgramID:%d\nTime: %d:%d\nOutput:\n| %d | %d | %d | %d |", CurrentProgramID, Time.Hours, Time.Minutes, ((WhatToDo>>3)&0x01), ((WhatToDo>>2)&0x01), ((WhatToDo>>1)&0x01), ((WhatToDo>>0)&0x01));
//	DEBUG(SMStext);
//	SMSisPending = 1;
	
	for(uint8_t i = 0; i < OUTPUTS_NUM; i++) // check periority; if periority = 1 -> proccess program has been run -> program skipped
	{
		if(Output_Actor[i] == 1)
		{
			DEBUG("\n\r Program Skipped \n\r");
			break;
		}
		if(i == OUTPUTS_NUM-1)
		{
			ApplyAction(WhatToDo);
			DEBUG("\n\r ACTION APPLIED \n\r");
		}
	}
	
	
	// 2- Save sheers status in backup registers:
//	HAL_RTCEx_BKUPWrite(&hrtc, LAST_STATUS_ADDRESS_LSB, WhatToDo[0]);
//	HAL_RTCEx_BKUPWrite(&hrtc, LAST_STATUS_ADDRESS_MSB, WhatToDo[1]);
//	DEBUG("\n\r VALUE SAVED \n\r");
	
	// 3- Jam kardane period ba zamane fe'li:
	CurrentProgramTime.tm_year	= Date.Year + 2000 - 1900;
	CurrentProgramTime.tm_mon		= CurrentProgram.Month -1;
	CurrentProgramTime.tm_mday	= CurrentProgram.Day;
	CurrentProgramTime.tm_hour	= CurrentProgram.Hour;
	CurrentProgramTime.tm_min		= CurrentProgram.Minutes;
	CurrentProgramTime.tm_sec		= 0;
	CurrentProgramTime.tm_isdst	= -1;
	
	epoch = mktime(&CurrentProgramTime);
	epoch = epoch + CurrentProgram.PeriodMinute*60 + CurrentProgram.PeriodHour*3600 + CurrentProgram.PeriodDay*(24*3600);	
	memset(ConvertedTimeStamp, NULL, 30);
	snprintf(ConvertedTimeStamp, 30, "%s", ctime(&epoch));
	
	CurrentProgram.Day			= (uint8_t)(atoi(&ConvertedTimeStamp[8]));
	CurrentProgram.Hour			= (uint8_t)(atoi(&ConvertedTimeStamp[11]));
	CurrentProgram.Minutes	= (uint8_t)(atoi(&ConvertedTimeStamp[14]));
	
	DEBUG("\n\r SUMATION DONE \n\r");
	
	writeProg(&CurrentProgram);
	DEBUG("\n\r PROGRAM SAVED IN MEMORY \n\r");
	
	// 4- neveshtane barname ye update shode dar hafeze ya pak kardane on agar tekrarshavande nabashad:
	if(CurrentProgram.has_interval==0){					// In khat check mikone bebine barname tekrar shavande hast ya na. Age nabood, hazfesh mikone
		//delete that program
		deleteProg(&CurrentProgram);
		DEBUG("\n\r PROGRAM DELETED \n\r");
	}
	
	SetNextAlarm(rtc);
}
/**
  * @brief  This routin set alarm for next process programs event.
  * @param  hrtc handler
  */
void SetNextAlarm_Processprograms(RTC_HandleTypeDef* rtc){
	 
	 char 						eventTime[30];					//used when convert eventTimeStamp to time
	 //HAL_RTCEx_BKUPWrite(rtc, PROGRAM_ID_ADDRESS, processProg.ID);

 	 memset(eventTime, NULL, 30);
	 snprintf(eventTime, 30, "%s", ctime(&nextEventTimeStamp));//convert eventTimeStamp to time in this format:  Fri Apr  2 20:02:22 2021
	// HAL_RTCEx_BKUPWrite(&hrtc, LAST_EVENT_TIME_STAMP, eventTimeStamp);//stroe eventTimeStamp into eeprom
	 
	//setting next event alarm:
	 RTC_AlarmTypeDef sAlarm = {0}; 
	 sAlarm.AlarmTime.Hours =  (uint8_t)(atoi(&eventTime[11]));
	 sAlarm.AlarmTime.Minutes = (uint8_t)(atoi(&eventTime[14]));
	 sAlarm.AlarmTime.Seconds =(uint8_t)(atoi(&eventTime[17]));
	 sAlarm.AlarmTime.SubSeconds = 0;
	 sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	 sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	 sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	 sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	 sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	 sAlarm.AlarmDateWeekDay = (uint8_t)(atoi(&eventTime[8]));
	 sAlarm.Alarm = RTC_ALARM_B;
	 if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  	 Error_Handler();
	 
	processProgramsAlarmIsSet = 1; //Enable Alarm
	memset(ContentStr,NULL,size);
	 snprintf(ContentStr,size,"\n\r **Time: %d/%02d/%02d %02d:%02d:%02d -- ALARM SET FOR Button %d  event:%d  : Day:%d  %02d:%02d:%02d   *\n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds,buttonId+1,eventNumber[buttonId]+1,sAlarm.AlarmDateWeekDay,  sAlarm.AlarmTime.Hours, sAlarm.AlarmTime.Minutes,sAlarm.AlarmTime.Seconds);
	DEBUG(ContentStr);

	}
/**
  * @brief  This routin find the nearst program and set an alarm to run on the time.
  * @param  hrtc handler
  */
void SetNextAlarm(RTC_HandleTypeDef* rtc){
	struct tm ProgramTimeToCompare;
	Program 	program_index;
	uint8_t 	EarlierProgramID = 0;
	uint32_t 	MiniDiffSeconds = 0xFFFFFFFF;
	
	// 4.5- Update kardane barnme ha baraye jologiri az conflict dar barname ha va hang
	updatePrograms();
	
	// 5- Finding the nearest program:
	for(int i=10; i<10 + MAX_NUM_PROGRAM; i++)
	{

		program_index = readProg(i);
		ProgramTimeToCompare.tm_year	= Date.Year +2000 - 1900;
		ProgramTimeToCompare.tm_mon	= program_index.Month-1;
		ProgramTimeToCompare.tm_mday	= program_index.Day;
		ProgramTimeToCompare.tm_hour	= program_index.Hour;
		ProgramTimeToCompare.tm_min	= program_index.Minutes;
		ProgramTimeToCompare.tm_sec	= 0;
		ProgramTimeToCompare.tm_isdst	= -1;
		
		if(program_index.ID==i){
			if( mktime(&ProgramTimeToCompare)<MiniDiffSeconds ){
				MiniDiffSeconds = mktime(&ProgramTimeToCompare);
				EarlierProgramID = program_index.ID;
			}
		}
	}
	
	// 6- Save the nearest program ID in 19th backup registers
	program_index = readProg(EarlierProgramID);
	
	if(program_index.ID == EarlierProgramID)
	{
	
		DEBUG("\n\rNearest is: \n\r");
		PrintProgram(program_index);
		DEBUG("\n\r");
		
		HAL_RTCEx_BKUPWrite(rtc, PROGRAM_ID_ADDRESS, program_index.ID);
		
		// 7- Set alarm:
		 RTC_AlarmTypeDef sAlarm = {0};
		 sAlarm.AlarmTime.Hours = program_index.Hour;
		 sAlarm.AlarmTime.Minutes = program_index.Minutes;
		 sAlarm.AlarmTime.Seconds = 0;
		 sAlarm.AlarmTime.SubSeconds = 0;
		 sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		 sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
		 sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
		 sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
		 sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		 sAlarm.AlarmDateWeekDay = program_index.Day;
		 sAlarm.Alarm = RTC_ALARM_A;
		 if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
		 {
		   Error_Handler();
		 }
		AlarmIsSet = 1;
		memset(ContentStr,NULL,size);
		snprintf(ContentStr,size,"\n\r * ALARM SET FOR: %02d %02d:%02d:00 *\n\r", program_index.Day, program_index.Hour, program_index.Minutes);
		DEBUG(ContentStr);
	 }
	else{
		/*
			THERE IS NO SCHEDULE:
				Disable alarm.
		*/
		AlarmIsSet = 0;
		DEBUG("\n\r <no schedule found>");
	}
	
}
/**
  * @brief  Itterate on all programs, then finds the programs in the past, deletes the unrepeatable ones and moves repeatable programss to the feuture.
  */
void updatePrograms(void){
	struct tm	ProgramTimeToCompare;
	Program		program_index;
	char 			ConvertedTimeStamp[30];
	uint32_t 	epoch=0;
	uint8_t 	flag=0;
	uint8_t 	changed=0;

	struct tm CurrentTimeToCompare;
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	CurrentTimeToCompare.tm_year	= Date.Year + 2000  - 1900;
	CurrentTimeToCompare.tm_mon		= Date.Month-1;
	CurrentTimeToCompare.tm_mday	= Date.Date;
	CurrentTimeToCompare.tm_hour	= Time.Hours;
	CurrentTimeToCompare.tm_min		= Time.Minutes;
	CurrentTimeToCompare.tm_sec		= Time.Seconds;
	CurrentTimeToCompare.tm_isdst	= -1;	
	uint32_t CurrentTime = 0;
	CurrentTime = mktime(&CurrentTimeToCompare);
	
	for(int i = 10; i < 10 + MAX_NUM_PROGRAM; i++){

		program_index = readProg(i);
		ProgramTimeToCompare.tm_year	= Date.Year + 2000 - 1900;
		ProgramTimeToCompare.tm_mon		= program_index.Month-1;
		ProgramTimeToCompare.tm_mday	= program_index.Day;
		ProgramTimeToCompare.tm_hour	= program_index.Hour;
		ProgramTimeToCompare.tm_min		= program_index.Minutes;
		ProgramTimeToCompare.tm_sec		= 0;
		ProgramTimeToCompare.tm_isdst	= -1;
		
		if(program_index.ID == i){
			changed = 0;
			while( CurrentTime > mktime(&ProgramTimeToCompare) ){    // yani ta vaghti in barname male gozashte ast edame bede
				
				if( program_index.has_interval == 0 ){
					deleteProg(&program_index);
					flag = 1;
					break;
				}
				

				epoch = mktime(&ProgramTimeToCompare);
				epoch = epoch + program_index.PeriodMinute*60 + program_index.PeriodHour*3600 + program_index.PeriodDay*(24*3600);	
				memset(ConvertedTimeStamp, NULL, 30);
				snprintf(ConvertedTimeStamp, 30, "%s", ctime(&epoch));
				
				program_index.Day			= (uint8_t)(atoi(&ConvertedTimeStamp[8]));
				program_index.Hour		= (uint8_t)(atoi(&ConvertedTimeStamp[11]));
				program_index.Minutes	= (uint8_t)(atoi(&ConvertedTimeStamp[14]));
				
				if(program_index.Day < ProgramTimeToCompare.tm_mday )
					program_index.Month++;
				changed = 1;
				
				ProgramTimeToCompare.tm_year	= Date.Year + 2000 - 1900;
				ProgramTimeToCompare.tm_mon		= program_index.Month-1;
				ProgramTimeToCompare.tm_mday	= program_index.Day;
				ProgramTimeToCompare.tm_hour	= program_index.Hour;
				ProgramTimeToCompare.tm_min		= program_index.Minutes;
				ProgramTimeToCompare.tm_sec		= 0;
				ProgramTimeToCompare.tm_isdst	= -1;
			}
			
			if(flag==0){
				if(changed==1){
					writeProg(&program_index);
					DEBUG("\n\rUpdated program is:\n\r");
					PrintProgram(program_index);
				}
			}
			else{
				DEBUG("\n\r A NONE REPEATABLE PROGRAM FROM PAST DELETED\n\r");
				flag = 0;
			}
		}
	}
}
/**
  * @brief  Selects the nearest event time stamp between the active process buttons
  * 
  */
uint8_t updateNextEvent(){
	
	uint8_t counter = 0;
	for(uint8_t i = 0; i < BUTTONS_NUM; i++)
	{
		if(eventsTurn[i] != 0)
			for(uint8_t j = 0; j < BUTTONS_NUM; j++)
			{
				if((eventsTurn[i] < eventsTurn[j]) || (eventsTurn[j] == 0))
					counter++;
			}
		if(counter == BUTTONS_NUM - 1)
		{
			nextEventTimeStamp = eventsTurn[i];
			buttonId = i; 
			
			return 0;
		}
		else
		{
			counter = 0;
			nextEventTimeStamp = 0;			
		}			
	}
	
	
	
}
/**
  * @brief  Print a program in debug port
  * @param  input is the program you want to be printed
  */
void PrintProgram(Program input){
		char actions[30];	
		memset( outputsStatus, 0,OUTPUTS_NUM);
		memset( actions, NULL ,30);
		decimalOutputs=  input.Action[0]+input.Action[1]*256; //tabdil adad mabnaye 256 status be yek adad decimal 
		for(uint8_t i=0;i<OUTPUTS_NUM;i++)  // change from decinal to base 4 number
		{
		 if(i!=OUTPUTS_NUM-1)
			outputsStatus[i]=decimalOutputs%4;
		 else
			outputsStatus[OUTPUTS_NUM-1]=decimalOutputs;
		 decimalOutputs=decimalOutputs/4;
		}
		#if(OUTPUTS_NUM==2)
		sprintf(actions, "%d %d ", outputsStatus[1], outputsStatus[0]); 
		#endif
		#if(OUTPUTS_NUM==3)
		sprintf(actions, " %d %d %d ",outputsStatus[2], outputsStatus[1], outputsStatus[0]); 
		#endif
		#if(OUTPUTS_NUM==4)
		sprintf(actions, " %d %d %d %d ", outputsStatus[3], outputsStatus[2], outputsStatus[1], outputsStatus[0]); 
		#endif
		#if(OUTPUTS_NUM==6)
		sprintf(actions, " %d %d %d %d %d %d", outputsStatus[5],outputsStatus[4], outputsStatus[3], outputsStatus[2], outputsStatus[1], outputsStatus[0]); 
		#endif
		#if(OUTPUTS_NUM==8)
		sprintf(actions, "%d %d %d %d %d %d %d %d",outputsStatus[7], outputsStatus[6],outputsStatus[5], outputsStatus[4], outputsStatus[3], outputsStatus[2], outputsStatus[1], outputsStatus[0]); 
		#endif
		memset(str,NULL,size);
		snprintf(str,sizeof(str)," -- Program ID: %d\n\r -- Program Month: %d\n\r -- Program Day: %d\n\r -- Program Hour: %d\n\r -- Program Minutes: %d\n\r -- Program Period Day: %d\n\r -- Program Period Hour: %d\n\r -- Program Period Minutes: %d\n\r -- Program Action:%s \n\r -- Program has_interval: %d\n\r",
		input.ID, input.Month, input.Day, input.Hour, input.Minutes, input.PeriodDay, input.PeriodHour, input.PeriodMinute,actions, input.has_interval);
		DEBUG(str);
}
/**
	* @brief  Converts a string like "1101" to its binarry equevalent like: 0b1101
  * @param  input is the string you want to convert to binarry
	* @param  uint8_t
  */
uint8_t str2bin(char* input){
	uint8_t string2bin=0x00;
	for(int i=0; i<strlen(input); i++){
		string2bin = string2bin * 2;
		if(input[i]=='1')
			string2bin = string2bin + 1;
	}
	return string2bin;
}
/**
	* @brief  Converts base 4 number in string type to decimal number in uint16_t type
  * @param  input is a base 4 number in string type you want to convert to decimal
	* @param  uint16_t decimal number
  */
uint16_t str2dec(char* input){///////////NEW ADDED/////////////////
	
	uint16_t string2dec=0x00;
	for(int i=0; i<strlen(input); i++){
		 if(input[i]=='0')
			string2dec = string2dec + 0*power(4,i);
		else if(input[i]=='1')
			string2dec = string2dec + 1*power(4,i);
		else if(input[i]=='2')
			string2dec = string2dec + 2*power(4,i);
	}
	return string2dec;
}

/**
	* @brief  caculate a^p
  * @param  input is 2 parameter a and p
	* @param  uint16_t decimal number --> result of the calculation
  */
uint16_t 	power(uint8_t a, uint8_t p){ ///////////NEW ADDED/////////////////

uint16_t m = 1;	
for(uint8_t i = 1; i <= p; i++)
m *= a;
return m;
	
}
/**
  * @brief  Delete all programs stord in EEPROM
  */
void deleteAllPrograms(void){ ///////////NEW ADDED/////////////////
	uint8_t values[MAX_NUM_PROGRAM * EE24_WRITE_COEFFICIENT_PROG];
	memset(values, 255, MAX_NUM_PROGRAM * EE24_WRITE_COEFFICIENT_PROG);
	ee24_write(&hi2c1, EE24_START_ADDRESS_PROGRAM, values, MAX_NUM_PROGRAM * EE24_WRITE_COEFFICIENT_PROG, 1000);
}
/**
  * @brief  Prints all valid programs stored in EEPROM.
  */
void PrintAllPrograms(void){
	Program programToPrint;
	char printedStr[30];
	
	HAL_Delay(500);
	for(int i=10; i<10+MAX_NUM_PROGRAM; i++){
		programToPrint = readProg(i);
		if(programToPrint.ID == i){
			memset(printedStr, NULL, 30);
			snprintf(printedStr, 30, "\n\rProgram number %d:\n\r", i);
			DEBUG(printedStr);
			PrintProgram(programToPrint);
		}
	}
}
/**
  * @brief  Prints all valid Process programs stored in EEPROM.
  */
void PrintAllProcessProgram(int i){
	DEBUG("\n\r**************\n\r");
	DEBUG("\n\r debug read process program from eeprom: \n\r");
	ProcessProgram 		pprogram;
	pprogram=readProcessProg(i);//read Process programs stored in EEPROM.
	memset(str,NULL,size);
	sprintf(str, "\n\r ID is \"%d\"", pprogram.ID);
	DEBUG(str);
	memset(str,NULL,size);
	sprintf(str,"\n\r events_count is \"%d\"", pprogram.events_count);
	DEBUG(str);
	DEBUG("\n\r-------\n\r");
	for(int i = 0; i < pprogram.events_count; i++)
	{
		memset(str, NULL, size);
		sprintf(str, "\n\r   event:   %d ",i);
		DEBUG(str);		
		memset(str, NULL, size);
		sprintf(str, "\n\r   Time duration  is :   %02d:%02d:%02d ", pprogram.Hour[i], pprogram.Minutes[i], pprogram.Second[i]);
		DEBUG(str);
		
		memset( outputsStatus, 0,OUTPUTS_NUM);
		decimalOutputs=  pprogram.spouts[i][0]+pprogram.spouts[i][1]*256; //tabdil adad mabnaye 256 status be yek adad decimal 
		for(uint8_t i=0;i<OUTPUTS_NUM;i++)  // change from decinal to base 4 number
		{
		 if(i!=OUTPUTS_NUM-1)
			outputsStatus[i]=decimalOutputs%4;
		 else
			outputsStatus[OUTPUTS_NUM-1]=decimalOutputs;
		 decimalOutputs=decimalOutputs/4;
		}
		
		memset(str, NULL, size);
		#if(OUTPUTS_NUM==2)
		sprintf(str, "\n\r   spouts  is :   %d %d ", outputsStatus[1], outputsStatus[0]); 
		#endif
		#if(OUTPUTS_NUM==3)
		sprintf(str, "\n\r   spouts  is :   %d %d %d ",outputsStatus[2], outputsStatus[1], outputsStatus[0]); 
		#endif
		#if(OUTPUTS_NUM==4)
		sprintf(str, "\n\r   spouts  is :   %d %d %d %d ", outputsStatus[3], outputsStatus[2], outputsStatus[1], outputsStatus[0]); 
		#endif
		#if(OUTPUTS_NUM==6)
		sprintf(str, "\n\r   spouts  is :   %d %d %d %d %d %d", outputsStatus[5],outputsStatus[4], outputsStatus[3], outputsStatus[2], outputsStatus[1], outputsStatus[0]); 
		#endif
		#if(OUTPUTS_NUM==8)
		sprintf(str, "\n\r   spouts  is :   %d %d %d %d %d %d %d %d",outputsStatus[7], outputsStatus[6],outputsStatus[5], outputsStatus[4], outputsStatus[3], outputsStatus[2], outputsStatus[1], outputsStatus[0]); 
		#endif
		
		DEBUG(str);
		DEBUG("\n\r-------\n\r");
	}
	

	
	
	DEBUG("\n\r**************\n\r");
}
//*
void my_delay(uint32_t interval){
	uint16_t times=0;
	uint16_t residue=0;
	
	times	= (interval * 1000) / 0xffff;
	residue	= (interval * 1000) % 0xffff;
	
	TIM2->CNT=0;
	HAL_TIM_Base_Start(&htim2);
	while(TIM2->CNT<residue*1000);
	HAL_TIM_Base_Stop(&htim2);
	
	for(uint16_t i=0; i< times; i++){
		TIM2->CNT=0;
		HAL_TIM_Base_Start(&htim2);
		while(TIM2->CNT<0xffff*1000);
		HAL_TIM_Base_Stop(&htim2);
	}
}
//*/
uint8_t JSON2Str(char* result, char* raw_input, char* key){
	char value_str[60];
	cJSON *server_response = cJSON_Parse(raw_input);		
	const cJSON *value	= NULL; //string
	value = cJSON_GetObjectItemCaseSensitive(server_response, key);
	if (cJSON_IsString(value) && (value->valuestring != NULL))
	{
		//sprintf(value_str, "\n\r  Extracted value is \"%s\"", value->valuestring);
		//DEBUG(value_str);
		sprintf(result, "%s", value->valuestring);
		cJSON_Delete(server_response);
		return 1;
	}
	cJSON_Delete(server_response);
	return 0;
}

uint8_t JSON2Str_nested(char* result, char* raw_input, char* key_parent, char* key_child){
	char value_str[60];
	cJSON *server_response = cJSON_Parse(raw_input);		
	const cJSON *value_p	= NULL;
	const cJSON *value_c	= NULL;
	value_p = cJSON_GetObjectItemCaseSensitive(server_response, key_parent);
	value_c = cJSON_GetObjectItemCaseSensitive(value_p, key_child);
	if (cJSON_IsString(value_c) && (value_c->valuestring != NULL))
	{
		//sprintf(value_str, "\n\r  Extracted value is \"%s\"", value_c->valuestring);
		//DEBUG(value_str);
		sprintf(result, "%s", value_c->valuestring);
		cJSON_Delete(server_response);
		return 1;
	}
	cJSON_Delete(server_response);
	return 0;
}

uint8_t JSON2int(char* result, char* raw_input, char* key){

	char value_str[60];
	cJSON *server_response = cJSON_Parse(raw_input);		
	const cJSON *value	= NULL; //int
	value = cJSON_GetObjectItemCaseSensitive(server_response, key);
	//if (cJSON_IsNumber(value) && (value->valueint != NULL))
	{
		//sprintf(value_str, "\n\r  Extracted value is \"%d\"", value->valueint);
		//DEBUG(value_str);
		sprintf(result, "%d", value->valueint);
		cJSON_Delete(server_response);
		return 1;
	}
	cJSON_Delete(server_response);
	return 0;
}
/**
  * @brief  Regular conversion complete callback
  * @param  current_ma is the estimated current value in mA unit.
  *         raw is the raw value of currnet between 0 to 4095. This value used in calibration process.
  * @retval None
  */
void current_sensor_CallBack(uint16_t current_ma, uint32_t raw){
	char dbg_string[70];
	Load_Status = (current_ma >= __ON_OFF_CURRENT_THRESH__);
	snprintf(dbg_string, 70, "\n\rCurrent = %d \tRaw = %d \tState = %s \tNumber of tries = %d", current_ma, raw, Load_Status ? "ON" : "OFF", Load_NumberOfTries);
	DEBUG(dbg_string);
}

void AbGiriProccess(void){
	// Is it Abgiri time?
	if(isTimeForCurrentCheck && isAbGiriInProgress){
		
		// OK, turn the pomp on:
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_SET);
		
		// Wait 4 seconds please...
		HAL_Delay(4000);
		
		// Was the pomp really turned on?
		if(Load_Status==0){
			DEBUG("\n\r   I CHECKED THE POMP, THAT WAS OFF I THINK.");
			DEBUG("\n\r   I AM GOING TO RESET IT...");
			
			// Oh no pomp is off! Please turn it on again:
			HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET);
			HAL_Delay(5000);
			HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_SET);
			
			DEBUG("\n\r   I RESET THE POMP.");
			
			// Increament number of tries please.
			// Did we exceed the maximum allowed tries?
			if(Load_NumberOfTries++ >= 10){
				DEBUG("\n\r --MAXIMUM TRIES EXCEEDED. I IGNORED THE ABGIRI--");
				
				// Ah shit we tried to turn the pomp on 10 times! So ignore the Abgiri:
				HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET);
				isAbGiriInProgress = 0;
				
				/*
					Add some codes here if you want to show in debug what happened.
				*/
				
			}
		}
		else{
			DEBUG("\n\r   I CHECKED THE POMP, THAT WAS -ON- I THINK.");
		}
	}
	
	// Take down the current feedback flag:
	isTimeForCurrentCheck = 0;
}

/**
  * @brief  Get Time & Date from "worldtimeapi.org" and save in RTC
  */
void GET_SAVE_Time(void){
	char  	 year[4];					//variables : Save time in RTC 	
	char  	 month[2];
	char  	 day_of_month[2];
	char     day_of_week[1];
	char  	 hour[2];
	char 	   minute[2];
	char  	 second[2];
	char  	 time_result[60];
	
	char     *startAnswer;				//variables : Get time from Server  
	char     *endAnswer;
	
		if (isConnect == 1)		// SIM800 is Active
		{ 
			uint8_t try_connect = 0;
			while(try_connect < nmbr_try_connect_site)			//try 5 time for connect to the site
			{
				sim80x_ATC("AT+HTTPPARA=\"URL\",\"http://worldtimeapi.org/api/timezone/asia/tehran/\"\r\n",100); // Connect to www.worldtimeapi.org and get time	
				sim80x_ATC("AT+HTTPACTION=0\r\n",6000);		  	//request GET_MODE 
				sim80x_ATC("AT+HTTPREAD\r\n",500);							//read SIM800 response
				
							startAnswer	= strstr(RxBuffer, "{");		// Fetching pure JSON from SIM800 response
							endAnswer		= strstr(RxBuffer, "}")+1;
							
							memset(str, NULL, size);
							for(int i=(startAnswer-RxBuffer); i<(endAnswer-RxBuffer); i++)
								str[i-(startAnswer-RxBuffer)] = RxBuffer[i];

				
				
				if(str[2] == 'a' && str[3] == 'b' && str[4] == 'b' && str[5] == 'r' && str[6] == 'e')		// check the site response is valid
				{
					try_connect = 5;		// exit from while
					/*Raw Data-> {"abbreviation":"-03","client_ip":"217.218.17.209","datetime":"2021-03-04T06:16:34.272629-03:00",
						"day_of_week":4,"day_of_year":63,"dst":false,"dst_from":null,"dst_offset":0,"dst_until":null,
						"raw_offset":-10800,"timezone":"America/Argentina/Cordoba","unixtime":1614849394,
						"utc_datetime":"2021-03-04T09:16:34.272629+00:00","utc_offset":"-03:00","week_number":9}*/
//						DEBUG(str);
						JSON2Str(time_result, str,"datetime");			//Separate  date_time section
						JSON2int(day_of_week, str,"day_of_week");		//Fetch Day_of_week
						
						DEBUG("\n\rServer Time: ");
						DEBUG(time_result);										//Show date_time section
					
					  memcpy(year,time_result+0,4);   				//Fetch Year
						memcpy(month,time_result+5,2); 				  //Fetch Month
						memcpy(day_of_month,time_result+8,2);   //Fetch Day of month

						memcpy(hour,time_result+11,2);  				//Fetch Hour
						memcpy(minute,time_result+14,2); 			  //Fetch Minute
						memcpy(second,time_result+17,2);  			//Fetch Second

				//*/	
					
				//*
				DEBUG("\n\rSETTING DATE AND TIME...");	
						RTC_TimeTypeDef sTime = {0};
						RTC_DateTypeDef sDate = {0};
						// Setting Time
						sTime.Hours   = ((uint8_t) hour[1] - '0')   + 10*((uint8_t) hour[0] - '0');				 // Convert Hour parameter
						sTime.Minutes = ((uint8_t) minute[1] - '0') + 10*((uint8_t) minute[0] - '0'); 		 // Convert Minute parameter
						sTime.Seconds = ((uint8_t) second[1] - '0') + 10*((uint8_t) second[0] - '0'); 		 // Convert Second parameter
						sTime.DayLightSaving = RTC_DAYLIGHTSAVING_ADD1H;
						sTime.StoreOperation = RTC_STOREOPERATION_RESET;
						if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)						// Set Time
						{
							Error_Handler();
						}
						// Setting Date
						sDate.Year	  =  (((uint16_t) year[3] - '0') + 10*((uint16_t) year[2] - '0')+100*((uint16_t) year[1] - '0') + 1000*((uint16_t) year[0] - '0'))-2000; // Convert Year parameter
						sDate.Month	  = ((uint8_t) month[1] - '0') + 10*((uint8_t) month[0] - '0');								// Convert Month parameter
						sDate.Date 		= ((uint8_t) day_of_month[1] - '0') + 10*((uint8_t) day_of_month[0] - '0'); // Convert Date parameter
						
						sDate.WeekDay = ((uint8_t) day_of_week[0] - '0');																					// Convert WeekDay parameter
						if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) 					// Set Date
						{
							Error_Handler();
						}
						// SHOW Device DATE AND TIME 
						HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);		// Get time
						HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);		// Get date
						snprintf(str,sizeof(str),"\n\r<<<<<<< Device Time: %d/%02d/%02d %02d:%02d:%02d >>>>>>", 2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds);
						DEBUG(str);																				// print time & date in terminal
				}
				else
				{
					try_connect++;														// Coutn number of try to connect
					if(try_connect == nmbr_try_connect_site)	// return fail result and exit from try_while
						DEBUG("\n\r   Server Connection FAILURE.\n\r");	
				}
			}					
											
 
 
			}
			else
				DEBUG("\n\r  SIM800 is not responding.\n\r");
}


void TimestampTOdate(long int seconds ,uint8_t butID )
{


    // Number of days in month
    // in normal year
    int daysOfMonth[] = { 31, 28, 31, 30, 31, 30,
                          31, 31, 30, 31, 30, 31 };
 
    long int currYear, daysTillNow, extraTime,
        extraDays, index, date, month, hours,
        minutes, secondss, flag = 0;
 
    // Calculate total days unix time T
    daysTillNow = seconds / (24 * 60 * 60);
    extraTime = seconds % (24 * 60 * 60);
    currYear = 1970;
 
    // Calculating current year
    while (daysTillNow >= 365) {
        if (currYear % 400 == 0
            || (currYear % 4 == 0
                && currYear % 100 != 0)) {
            daysTillNow -= 366;
        }
        else {
            daysTillNow -= 365;
        }
        currYear += 1;
    }
 
    // Updating extradays because it
    // will give days till previous day
    // and we have include current day
    extraDays = daysTillNow + 1;
 
    if (currYear % 400 == 0
        || (currYear % 4 == 0
            && currYear % 100 != 0))
        flag = 1;
 
    // Calculating MONTH and DATE
    month = 0, index = 0;
    if (flag == 1) {
        while (true) {
 
            if (index == 1) {
                if (extraDays - 29 < 0)
                    break;
                month += 1;
                extraDays -= 29;
            }
            else {
                if (extraDays
                        - daysOfMonth[index]
                    < 0) {
                    break;
                }
                month += 1;
                extraDays -= daysOfMonth[index];
            }
            index += 1;
        }
    }
    else {
        while (true) {
 
            if (extraDays
                    - daysOfMonth[index]
                < 0) {
                break;
            }
            month += 1;
            extraDays -= daysOfMonth[index];
            index += 1;
        }
    }
 
    // Current Month
    if (extraDays > 0) {
        month += 1;
        date = extraDays;
    }
    else {
        if (month == 2 && flag == 1)
            date = 29;
        else {
            date = daysOfMonth[month - 1];
        }
    }
 
    // Calculating HH:MM:YYYY
    hours = extraTime / 3600;
    minutes = (extraTime % 3600) / 60;
    secondss = (extraTime % 3600) % 60;
		buttonsLastTime.hours[butID]=(uint16_t)hours;
		buttonsLastTime.minutes[butID]=(uint16_t)minutes;
		buttonsLastTime.seconds[butID]=(uint16_t)secondss;
		buttonsLastTime.year[butID]=(uint16_t)currYear-2000;
		buttonsLastTime.month[butID]=(uint16_t)month;
		buttonsLastTime.day[butID]=(uint16_t)date;
//		memset(ContentStr,NULL,size);
//	  snprintf(ContentStr,size,"\n\r #####Time: %d/%02d/%02d %02d:%02d:%02d   ID:%d \n\r",buttonsLastTime.year[butID], buttonsLastTime.month[butID],buttonsLastTime.day[butID],buttonsLastTime.hours[butID], buttonsLastTime.minutes[butID], buttonsLastTime.seconds[butID],butID);
//  	DEBUG(ContentStr);
		

}
/**
  * @brief  Connect to the server and send the uniq chip code and receive ID and land from server.
  */
void Get_SAVE_ID(void){
	
//  uint32_t (*unique_id_1) = (uint32_t*)(U_ID ); 				// BASE address
//	uint32_t (*unique_id_2) = (uint32_t*)(U_ID + 0x04 );  // BASE address + 0x04 offset
		uint32_t (*unique_id_3) = (uint32_t*)(U_ID + 0x08);   // BASE address + 0x14 0ffset
		char *startAnswer;
		char *endAnswer;
		char  get_id[5];					// use for Get ID from RAW_DATA
		char  get_land_id[5];			// use for Get ID from RAW_DATA
		sprintf(serial_number_string,"%d",(*unique_id_3));			// convert unique code from uint_32 to char 
		if(isConnect==1)																				// sim800 connect
		{
//			DEBUG("\n\r   sending serial number to server...");	
			uint8_t try_connect = 0;
			while(try_connect < nmbr_try_connect_site)				    //try "nmbr_try_connect_site" time for connect to the sever
			{
				memset(ContentStr,NULL,size);
				snprintf(ContentStr,size,"{\"serial\":\"%s\"}\r\n",__SERIAL_NUMBER);			// input arguman for Post_Request = device serial number
				sim80x_HTTP_Post(ContentStr, SERVER_IP,"get_land_by_device_serial/",ContentStr); // Post_Request for getting ID
				// Fetching pure JSON from SIM800 response		
					startAnswer	= strstr(ContentStr, "{");
					endAnswer		= strstr(ContentStr, "}")+1;
					memset(str, NULL, size);
					for(int i=(startAnswer-ContentStr); i<(endAnswer-ContentStr); i++)	// str <- {...} pure JSON
						str[i-(startAnswer-ContentStr)] = ContentStr[i];

					if(str[2] == 'l' && str[3] == 'a')		// check the server response is valid
					{
						try_connect = 5;										// exit from while
									
						JSON2int(get_id, str,"device");					// Fetch device ID
						JSON2int(get_land_id, str,"land");			// Fetch Land ID
						ee24_read(&hi2c1,0,(uint8_t *)RID,5,100);					// Read ID froem EEPROM
						ee24_read(&hi2c1,5,(uint8_t *)RID_land,5,100);	  // Read Land_ID froem EEPROM
						if( (strcmp(get_id, RID) == 0) && (strcmp(get_land_id, RID_land) == 0))			//check there is ID and Land_ID in EEPROM
						{
							DEBUG("\n\r<<<<<< ID & Land_ID There are in EEPROM >>>>>>>\n\r");
						}
						else				//ID & Land_ID didn't save in EEPROM
						{
							DEBUG("\n\r<<<<<<<<<< ID & Land_ID didn't save >>>>>>>>>>>\n\r");
							DEBUG("<<<<<<<<<<<<<< Saving:");
							if(ee24_write(&hi2c1,0,(uint8_t *)get_id,5,100)  && ee24_write(&hi2c1,5,(uint8_t *)get_land_id,5,100) == true)		// Write in EEPROM ("ID" at address : 0 and "Land_ID" at address : 1) and check  the result of writing   
								DEBUG("SUCCESSFUL >>>>>>>>>>>>>>\n\r");			    // Writing was successful
							else
								DEBUG("FAILED >>>>>>>>>>>>>>>>>>\n\r");				  // Writing was unsuccessful
						}
								ee24_read(&hi2c1,0,(uint8_t *)RID,5,100);				// Read ID froem EEPROM
								ee24_read(&hi2c1,5,(uint8_t *)RID_land,5,100);	// Read Land_ID froem EEPROM
								DEBUG("<<<<<<<<<<<<<<<<<< ID : ");
								DEBUG(RID);																			// Print ID in terminal
								DEBUG(" >>>>>>>>>>>>>>>>>>>>>>\n\r");
								
								DEBUG("<<<<<<<<<<<<<<<<Land ID : ");
								DEBUG(RID_land);																// Print Land ID in terminal
								DEBUG(" >>>>>>>>>>>>>>>>>>>>\n\r");						
						
					}
					else										// if the server response is not valid
					{
						try_connect++;				// Coutn number of try to connect
						if(try_connect == nmbr_try_connect_site)	// return fail result and exit from try_while
							DEBUG("\n\r<<<<<<<<<<< Server Connection FAILURE >>>>>>>>>>\n\r");	
					}
			}
					
		}
		else
			DEBUG("\n\r<<<<<<<<<<< SIM800 is not responding >>>>>>>>>>>\n\r");
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM8)
	{
		isTimeForCurrentCheck = 1;
	}

	
	if(htim->Instance==TIM5)
	{
		if(initializingFlag)
		HAL_GPIO_TogglePin(SW_Light3_GPIO_Port,SW_Light3_Pin);
		else
		HAL_GPIO_WritePin(SW_Light3_GPIO_Port,SW_Light3_Pin,1);	
		#if(__IS_OLED__==1)
		HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
		tim5CallbackCounter++;
		if(oledState!=0)// if we are not in ldm logo screen
		{
				memset(oledStr,NULL, size);
				snprintf(oledStr,sizeof(oledStr)," %02d:%02d",  Time.Hours, Time.Minutes);
				ssd1306_SetCursor(84, 3);
				ssd1306_WriteString(oledStr, Font_7x10, White);//show time on oled
				ssd1306_draw_bitmap(1, 17, line, 128, 2);//line under time and antenna
			 if(tim5CallbackCounter%40==0)
			 {
			  memset(oledStr,NULL, size);
			  snprintf(oledStr,sizeof(oledStr),"\n\r tim5CallbackCounter:%d  lastTim5CallbackCounter:%d\n\r", tim5CallbackCounter, lastTim5CallbackCounter);
			  DEBUG(oledStr);
			 }
						//////////RSSI antenna conection ///////// 
			
				ssd1306_SetCursor(0,0);
				if( rssiIntValue<32&&rssiIntValue>21)//rssi Excellent
				{
					ssd1306_clear_screen(5,30,0,15);
					ssd1306_draw_bitmap(7, 2, rssiSingal_4, 22, 11);//RSSI antenna: 4
				}
				else if( rssiIntValue<22&&rssiIntValue>16)//rssi Good
				{
					ssd1306_clear_screen(5,30,0,15);
					ssd1306_draw_bitmap(7, 2, rssiSingal_3, 22, 11);//RSSI antenna: 3
				}
				else if( rssiIntValue<17&&rssiIntValue>11)//rssi Ok
				{
						ssd1306_clear_screen(5,30,0,15);
					ssd1306_draw_bitmap(7, 2, rssiSingal_2, 22, 11);//RSSI antenna: 2
				}
				else if( rssiIntValue<12&&rssiIntValue>3)//rssi Marginal
				{
						ssd1306_clear_screen(5,30,0,15);
					ssd1306_draw_bitmap(7, 2, rssiSingal_1, 22, 11);//RSSI antenna: 1
				}
				else
				{
						ssd1306_clear_screen(5,30,0,15);
						ssd1306_draw_bitmap(7, 0, noSignal, 16, 15);//no anten
				}
				
				//////////server conection /////////
				
				if(isConnect == 0&&get_output_result==0&&rssiIntValue!=0)//if server  not conected "!" blinking beside rssi antenna.it is apply when get_output_result has taken value 0 and rssi value !=0
				{
					if	(blinker<4)//show "!" for 2 sec
					{
					 ssd1306_SetCursor(1,5);
					 ssd1306_WriteString("!", Font_7x10, White);
						blinker++;
					}
					else if(blinker<8)//show " " for 2 sec
					{
					 ssd1306_SetCursor(1,5);
					 ssd1306_WriteString(" ", Font_7x10, White);
					 blinker++;
					}
					if(blinker==8)
						blinker=0;
				}
				if(isConnect == 1&&rssiIntValue!=0&&(get_output_result==1||get_output_result==2||get_output_result==3||get_output_result==4))//if server  conected "s" blinking beside rssi antenna. it is apply when and get_output_result has taken value other than 0 and srssi value !=0
				{
					if	(blinker<6)//show "s" for 3 sec
					{
					 ssd1306_SetCursor(1,5);
					 ssd1306_WriteString("s", Font_7x10, White);
						blinker++;
					}
					else if(blinker<8)//show " " for 1 sec
					{
					 ssd1306_SetCursor(1,5);
					 ssd1306_WriteString(" ", Font_7x10, White);
					 blinker++;
					}
					if(blinker==8)
						blinker=0;
				}
				
				////////lora antenaa////////
				
				//ssd1306_draw_bitmap(25, 0, noSignal, 16, 15);//no anten
			
		}
	
//	if(oledPageNumber==0)
//	{
		switch(oledState)
		{
			case 0: 
					 if(2<tim5CallbackCounter&&tim5CallbackCounter<10)
					 {
						 ssd1306_draw_bitmap(10, 10, ldm, 119, 48);//show ldm logo for 4 sec
					 }
					 else if (tim5CallbackCounter&&tim5CallbackCounter>10)
					 {
						ssd1306_clear_screen(0,130,0,64);	//clear logo 
						oledState=1; // go to next state
					 }
			break;		 
			case 1:
					if(initializingFlag)
					{
						ssd1306_SetCursor(2,30);
						ssd1306_WriteString("Initializing", Font_7x10, White);//show "Initializing" until initializing done
						switch(dotPointCounter) // show dot points in front of "Initializing" until initializing done
						{
							case 0:
								ssd1306_DrawPixel(93,38, White);//"Initializing ."
								dotPointCounter++;
							break;

							case 1:
								ssd1306_DrawPixel(93,38, White);//"Initializing . ."
								ssd1306_DrawPixel(98,38, White);
								dotPointCounter++;
							break;
							case 2:
								ssd1306_DrawPixel(93,38, White);//"Initializing . . ."
								ssd1306_DrawPixel(98,38, White);
								ssd1306_DrawPixel(103,38, White);
								dotPointCounter++;
							break;
							case 3:
								dotPointCounter=0;
						  	//clear last 3 dot point:
								ssd1306_DrawPixel(93,38,  Black);
								ssd1306_DrawPixel(98,38, Black);
								ssd1306_DrawPixel(103,38, Black);
							break;
						}
//						if(getProcessProgramsStarting)
//						{
//							ssd1306_clear_screen(0,128,20,64);	//clear logo 
//							oledState=4; 
//						}
//						else if(getProgramsStarting)
//						{
//							ssd1306_clear_screen(0,128,20,64);	//clear logo 
//							oledState=6; 
//						}
					}
					else
					{
						oledState=2;  // go to next state when initializing done
						lastTim5CallbackCounter=tim5CallbackCounter; // save last time in state 1
					}
			break;
			case 2:
					if(tim5CallbackCounter<8+lastTim5CallbackCounter)//show "Initializing Done" CheckRight and  for 4 sec after initializing done
					 {
						 ssd1306_clear_screen(0,128,20,64);//clear main section of screen
						 ssd1306_SetCursor(2,30);
						 ssd1306_WriteString("Initializing Done", Font_7x10, White);
						 ssd1306_draw_bitmap(45, 42, checkRight, 16, 21);
					 }
					 else//after 4 secT clear screen and go to next state
					 {
						 ssd1306_clear_screen(0,128,20,64);//clear main section of screen
						 oledState=3;// go to next state
					 }
		  break;
			case 3:
			 //show last outouts status:
					ssd1306_clear_screen(0,128,20,64);//clear main section of screen

			 //show last outputs status:
		
					if(HAL_GPIO_ReadPin(relay1_GPIO_Port, relay1_Pin))
						ssd1306_draw_bitmap(60, 34, tapOn , 20, 30);//tapOn--> relay 1
					else
						ssd1306_draw_bitmap(60, 34, tapOff , 20, 30);//tapOff--> relay 1
					
					if(HAL_GPIO_ReadPin(relay2_GPIO_Port, relay2_Pin))
						ssd1306_draw_bitmap(90, 34, tapOn , 20, 30);//tapOn--> relay 2
					else
						ssd1306_draw_bitmap(90, 34, tapOff , 20, 30);//tapOff--> relay 2
					
//					if(HAL_GPIO_ReadPin(relay3_GPIO_Port, relay3_Pin))
//						ssd1306_draw_bitmap(60, 34, tapOn , 20, 30);//tapOn	
//					else
//						ssd1306_draw_bitmap(60, 34, tapOff , 20, 30);//tapOff	
//					
//					if(HAL_GPIO_ReadPin(relay4_GPIO_Port, relay4_Pin))
//						ssd1306_draw_bitmap(85, 34, tapOn , 20, 30);//tapOn
//					else
//						ssd1306_draw_bitmap(85, 34, tapOff , 20, 30);//tapOff	
					
				if(	getProcessProgramsStarting) //If we need to getting Process programs, clear the screen in this state and show "Downloading process programs" until  getting programs to be ended in state 4 & 5
				{
					oledState=4;
					ssd1306_clear_screen(0,128,20,64);//clear main section of screen
				}
				if(getProgramsStarting)//If we need to getting programs, clear the screen in this state and show "Downloading programs" until  getting programs to be ended in state 6 & 7
				{
					oledState=6;
					ssd1306_clear_screen(0,128,20,64);//clear main section of screen	
				}
//				if(tim5CallbackCounter>18+lastTim5CallbackCounter)
//				{
//					oledPageNumber=1;
//					lastTim5CallbackCounter=tim5CallbackCounter;
//				}
			break; 
			case 4:

					ssd1306_SetCursor(29,26);//show "Getting process programs" until Getting process programs to be ended
					ssd1306_WriteString("Downloading ", Font_7x10, White);
					ssd1306_SetCursor(9,39);
					ssd1306_WriteString("process programs", Font_7x10, White);
						switch(dotPointCounter) // show dot points under "Getting process programs" until Getting process programs to be ended
						{
							case 0:
								ssd1306_DrawPixel(55,55, White);//"Getting process programs."
								dotPointCounter++;
							break;

							case 1:
								ssd1306_DrawPixel(55,55, White);//"Getting process programs . ."
								ssd1306_DrawPixel(60,55, White);
								dotPointCounter++;
							break;
							case 2:
								ssd1306_DrawPixel(55,55, White);//"Getting process programs . . ."
								ssd1306_DrawPixel(60,55, White);
								ssd1306_DrawPixel(65,55, White);
								dotPointCounter++;
							break;
							case 3:
								dotPointCounter=0;
						  	//clear last 3 dot point:
								ssd1306_DrawPixel(55,55,  Black);
								ssd1306_DrawPixel(60,55, Black);
								ssd1306_DrawPixel(65,55, Black);
							break;
						}
				if(	getProcessProgramsStarting==0)
				{
					oledState=5;	// after "Getting process programs" to be ended we go to next state 
					lastTim5CallbackCounter=tim5CallbackCounter;// save last time in state 4
					ssd1306_clear_screen(55,128,54,64);//clear main section of screen
				}
			break;
	 	  case 5:
				if(tim5CallbackCounter<2+lastTim5CallbackCounter) //show the result of Getting process programs  for 1 sec
					{
						if(getProcessProgramsStatus==1)//Getting process programs is successful
						{
							ssd1306_SetCursor(53,52);
							ssd1306_WriteString("Done", Font_7x10, White);
						}
						else //Getting process programs is unsuccessful
						{
							ssd1306_SetCursor(48,52);
							ssd1306_WriteString("Error!", Font_7x10, White);	
						}
						
					}
				else
					{
//						if(initializingFlag)
//						{
//							ssd1306_clear_screen(0,128,20,64);//clear main section of screen
//							oledState=1;
//						}
//						else
//						{
					    ssd1306_clear_screen(0,128,20,64);//clear main section of screen and back to state 3 after 5 sec
							oledState=3;
//						}
					}
			break;
			case 6:
					ssd1306_SetCursor(29,24);//show "Getting  programs" until Getting  programs to be ended
					ssd1306_WriteString("Downloading ", Font_7x10, White);
					ssd1306_SetCursor(41,37);
					ssd1306_WriteString("Programs", Font_7x10, White);
					switch(dotPointCounter) // show dot points under "Getting  programs" until Getting  programs to be ended
					{
						case 0:
							ssd1306_DrawPixel(61,53, White);//"Getting  programs."
							dotPointCounter++;
						break;

						case 1:
							ssd1306_DrawPixel(61,53, White);//"Getting  programs . ."
							ssd1306_DrawPixel(66,53, White);
							dotPointCounter++;
						break;
						case 2:
							ssd1306_DrawPixel(61,53, White);//"Getting  programs . . ."
							ssd1306_DrawPixel(66,53, White);
							ssd1306_DrawPixel(71,53, White);
							dotPointCounter++;
						break;
						case 3:
							dotPointCounter=0;
							//clear last 3 dot point:
							ssd1306_DrawPixel(61,53,  Black);
							ssd1306_DrawPixel(66,53, Black);
							ssd1306_DrawPixel(71,53, Black);
						break;
					}
					if(	getProgramsStarting==0)// after "Getting  programs" to be ended we go to next state 
					{
						oledState=7;	
						lastTim5CallbackCounter=tim5CallbackCounter;// save last time in state 6
						ssd1306_clear_screen(55,128,52,64);//clear main section of screen
					}
			break;
			case 7:
				if(tim5CallbackCounter<2+lastTim5CallbackCounter) //show the result of Getting process programs for 1 sec
					{
						if(getProgramsStatus==1)//Getting process programs is successful
						{
							ssd1306_SetCursor(55,53);
							ssd1306_WriteString("Done", Font_7x10, White);
						}
						else //Getting process programs is not successful
						{
							ssd1306_SetCursor(50,53);
							ssd1306_WriteString("Error!", Font_7x10, White);	
						}
						
					}
				else
					{
					    ssd1306_clear_screen(0,128,20,64);//clear main section of screen and back to state 3 after 5 sec
							oledState=3;
//						}
					}
			break;
		}
		


		ssd1306_UpdateScreen();// update oled screnn and apply changes
	
	#endif	
	}
	
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	DEBUG("\n\rERROR HANDLER\n\r");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
