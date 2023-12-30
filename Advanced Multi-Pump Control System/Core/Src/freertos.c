/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "external_flash.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "ESP_UART_Config.h"
#include "semphr.h"
#include "usart.h"
#include "tim.h"
#include "VFD.h"
#include "adc.h"
#include "rtc.h"
#include "scheduler.h"
#include "modbus.h"
#include "backup_resgisters_operations.h"
#include "s25fl256s.h"
#include "quadspi.h"
#include "stm32h7xx_it.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MULTI_VFD
//#define SINGLE_VFD
//#define NO_VFD


#if defined(SINGLE_VFD)
uint8_t baseLoadPumpWokring=0;
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//*start uart_send task  -------------------------------------------------------------*/
#define MANUAL_MODE 0
#define AUTO_MODE 1

// flag send the operation mode to the app
//to show weither it we are on manual mode
//or PID when entering the home page to show the appropriate settings
uint8_t sendOperationMode=0;

//the pump id and speed that the user changed from the APP when
//the app is on the manual mode
uint8_t manualPumpID = 0;
uint16_t manualPumpSpeed = 0;

//current operaton mode
uint8_t operationMode = MANUAL_MODE;
/*flags to erase sections of the external flash while the code running by me
 * by editing them while debugging*/
volatile uint8_t eraseEntireChipFlag = 0;
volatile uint8_t eraseDatedRecordsFlag = 0;
volatile uint8_t eraseWeeklyRecordsFlag = 0;
volatile uint8_t eraseAlarmsRecordsFlag = 0;

//Flag when the user enter the alarm page on the app this will be 1 (alarms page)
uint8_t sendAlarmRecords = 0;
//Flag when the user enter the dated scheduler  page on the app this will be 1 (Scheduler page)
uint8_t sendSchedData = 0;




//(custom order page)
// the slave id on custom command page to send or receive from it
uint8_t Slave_ID;
uint8_t functionNumber;
uint64_t slaveAddress;
uint16_t dataToSend;
uint16_t responseData;
uint8_t sendNow;
uint8_t thereIsData;





//(pid task)
//start timer variables
uint8_t KIreceived = 0;
//end timer variables
//flag to stop the pid if no single vfd response at all
volatile uint8_t fullSystemTrip = 0;
//number of working currently working pumps on the system
uint8_t activePumps = 0;
//helpers to pid algortim variables
float accumulatedError = 0;
float lastError = 0;
float prevControlAction = 0;
float controlAction;
uint16_t desiredSpeedPerPump = 0;
uint8_t prevActivePumps = 0;
int16_t controlActionDifference = 0;
uint8_t schedulerPumpToShutDownIndex = 0;
uint8_t schedulerPumpToRiseIndex = 0;



//(StartDefaultTask)
//rgb led variables
uint16_t redValue = 64000;
uint16_t greenValue = 32000;
uint16_t blueValue = 32000;



//(global between task)
// holder of modbus request
uint16_t requestResponseHolder1 = danfoss101_stop;
uint16_t requestResponseHolder = 0;
//current speed of each pump on the system
uint16_t speedPercentageOfPump[8] =
{ 0, 0, 0, 0, 0, 0, 0, 0 };
//current pump current of each pump on the system
uint16_t pumpCurrent[8] =
{ 0, 0, 0, 0, 0, 0, 0, 0 };
//flag to indicate weither we are on home page or not to start or stop sending the pid data
uint8_t inHomePage = 0;
//flag to check if there is dry run condition to stop the pumps in this case
uint8_t dryRunDetected = 0;
// alarm array which contain all the alarms of the system
extern char alarmsArray[MAX_ALARMS][RECORD_SIZE];
uint16_t DanfossVfdMaxSpeed = 16384;
// if we enter the scheduler page this will be one to send the rtc date and time
uint8_t setRTC = 0;
//  hold current info about each pump on the array
PumpInfo pumpInfoArray[MAX_NUM_PUMPS];
// time out for modebus request if not response on this time (modbus configuration)
uint32_t TimeoutValue = 65; //minimum for a successful communication is 35ms
//test the case of connectvity with the vfd and this is every around 5 seconds
uint8_t testIFNoVfdResponse = 1;
// if the pressure sensor error is not connected probelry this flag will be 1
uint8_t pressureSensorError = 0;
// handler of  configurations of modbus
modbus_t telegram[2];



//(Factory Configuration page)
//used to to maintain the current value of pid ouput when hte user change the ki constant
float OldKI = 0;
float setPointRequired = 5;
uint8_t getFactoryData = 0;
//pack those three varibles on 4 bytes of the back domain
uint16_t offset = 1800;
float cutIn = 1.5;
float cutOff = 1.5;
uint8_t VFD_type = 0;
uint8_t systemPumpsNumber = 4;
//pack those two variables on one 32bit register of the backup domain
uint16_t firstADCCalibrationReading = 7743;
uint16_t secondADCCalibrationReading = 64196;
char RTCString[25];  // Replace this line with the actual received string
float Kp = 1000;
float Ki = .2;
float Kd = 0;
float currentOfTheFirstReading = 4.00000192;
float currentOfTheSecondReading = 20.5100198;
float_t minimumPressure = 0;
float MaximumPressure = 16;
uint16_t changeOverTime = 20;
//the flags try pack them in one byte
uint8_t enableScheduler = 0;
uint8_t enablePID = 0;
uint8_t save = 0;
uint8_t sendRTC = 1;
//	first current received from the setting page to calibrate the adc (Factory Configuration page)
uint8_t firstCurrentReadingRecived = 0;
//	second current received from the setting page to calibrate the adc
uint8_t secondCurrentReadingRecived = 0;
float coffA = 0.00029245f; // Calibration multiplier
float coffB = 1.735515392f; // Calibration offset
char vfdType[101] =
{ 0 };
//enable dry run switch
uint8_t enableDryRun = 0;



//(no vfd pid)
//used to hold each set point the system will start a new pump on it
float pumpsSetPoints[MAX_NUM_PUMPS];



//(Pumps_Scheduler task)
PumpInfo tempPumpInfoArray[MAX_NUM_PUMPS];
//if one of hte pumps excedded change over time it this flag will be set to one to make the
//scheduler task stop it and start one with less wokring hours
volatile uint8_t arrangeChanged = 0;



//modbus handlers and buffers and in our application it it allowed to use two instance of modbus communication
extern modbusHandler_t ModbusH;
extern uint16_t ModbusDATA[128];
extern modbusHandler_t ModbusH2;
extern uint16_t ModbusDATA2[128];

//varible that the rtc will fill with the current date and time
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;


//if the user in the app enter the alarms page this flag will be 1
uint8_t sendAlarms = 0;
// current presusre that is being get from the sensor connected to the board
float currentPressure;
//end ADC TASK VARibles

//(alarm and warning page)
// the user choosed from the alarms page that if there is a dry run it will trip the system
uint8_t dryRunSysStop = 0;
//the user enabled the pressure sensor not detected erro
uint8_t pressureAlarmEnable = 0;
//the user choose from alarm page that the pressure sensor not detected will trip the system
uint8_t pressureSysStop = 0;


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes =
{ .name = "defaultTask", .stack_size = 256 * 4, .priority =
		(osPriority_t) osPriorityNormal, };
/* Definitions for UART1Task */
osThreadId_t UART1TaskHandle;
const osThreadAttr_t UART1Task_attributes =
{ .name = "UART1Task", .stack_size = 512 * 4, .priority =
		(osPriority_t) osPriorityLow, };
/* Definitions for Task4_20mAscan */
osThreadId_t Task4_20mAscanHandle;
const osThreadAttr_t Task4_20mAscan_attributes =
{ .name = "Task4_20mAscan", .stack_size = 256 * 4, .priority =
		(osPriority_t) osPriorityLow, };
/* Definitions for myTaskMaster */
osThreadId_t myTaskMasterHandle;
const osThreadAttr_t myTaskMaster_attributes =
{ .name = "myTaskMaster", .stack_size = 256 * 4, .priority =
		(osPriority_t) osPriorityLow, };
/* Definitions for ExternalFlash */
osThreadId_t ExternalFlashHandle;
const osThreadAttr_t ExternalFlash_attributes =
{ .name = "ExternalFlash", .stack_size = 512 * 4, .priority =
		(osPriority_t) osPriorityLow, };
/* Definitions for UARTSend */
osThreadId_t UARTSendHandle;
const osThreadAttr_t UARTSend_attributes =
{ .name = "UARTSend", .stack_size = 512 * 4, .priority =
		(osPriority_t) osPriorityLow, };
/* Definitions for PID */
osThreadId_t PIDHandle;
const osThreadAttr_t PID_attributes =
		{ .name = "PID", .stack_size = 512 * 4, .priority =
				(osPriority_t) osPriorityLow, };
/* Definitions for PumpsScheduler */
osThreadId_t PumpsSchedulerHandle;
const osThreadAttr_t PumpsScheduler_attributes =
{ .name = "PumpsScheduler", .stack_size = 256 * 4, .priority =
		(osPriority_t) osPriorityLow, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void UART1_task(void *argument);
void ADC_4_20mApolling(void *argument);
void StartTaskMaster(void *argument);
void external_flash(void *argument);
void UART_Send(void *argument);
void PIDControl(void *argument);
void Pumps_Scheduler(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */

	printf("Stack overflow in task: %s (Handle: %p)\n", pcTaskName, xTask);

	while (1)
		;
}
/* USER CODE END 4 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);
//
//  /* creation of UART1Task */
	UART1TaskHandle = osThreadNew(UART1_task, NULL, &UART1Task_attributes);
//
//  /* creation of Task4_20mAscan */
	Task4_20mAscanHandle = osThreadNew(ADC_4_20mApolling, NULL,
			&Task4_20mAscan_attributes);
//
//  /* creation of myTaskMaster */
	myTaskMasterHandle = osThreadNew(StartTaskMaster, NULL,
			&myTaskMaster_attributes);
//
	/* creation of ExternalFlash */
	ExternalFlashHandle = osThreadNew(external_flash, NULL,
			&ExternalFlash_attributes);
////
//  /* creation of UARTSend */
	UARTSendHandle = osThreadNew(UART_Send, NULL, &UARTSend_attributes);
//
//  /* creation of PID */
	PIDHandle = osThreadNew(PIDControl, NULL, &PID_attributes);
//
//  /* creation of PumpsScheduler */
	PumpsSchedulerHandle = osThreadNew(Pumps_Scheduler, NULL,
			&PumpsScheduler_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/**
 * @brief  StartDefaultTask is a multi-purpose helper function designed for embedded systems.
 *         It performs several critical background tasks that are essential for the overall
 *         system's stability and responsiveness. This function is structured to run as a
 *         continuous task in an RTOS environment, adhering to MISRA C:2012 guidelines for
 *         safety-critical systems. The function's responsibilities encompass alarm monitoring,
 *         real-time clock management, RGB LED control, and VFD monitoring.
 *
 *         Key Functionalities:
 *         1. Alarm Monitoring: Monitors various system alarms (e.g., pressure sensor errors, dry run detections)
 *            and manages system state transitions in response to these alarms. It controls the PID
 *            operation and scheduler based on alarm states.
 *         2. RTC Management: Handles user-driven RTC settings, ensuring system time is accurately
 *            maintained and synchronized. It includes parsing date and time from user inputs and
 *            managing errors in RTC settings.
 *         3. RGB LED Control: Manages the color of RGB LEDs through hardware timers, adjusting the
 *            outputs based on system or user inputs, reflecting the system's state visually.
 *         4. VFD Monitoring: Regularly sets a flag that will trigger a check for  the responsiveness of connected Variable Frequency Drives,
 *            ensuring the motor control subsystem operates correctly and efficiently.
 *         5.send modbus request  when the user use the custom order page from the app
 *         6.set to each vfd certain speed which will make the app mark this vfd as faulty when he sees this
 *         certain speed
 *         7.calculate if there is no response at all from any vfd so it will stop the pid algorithm
 *         @note :5,6,7 will be migrated from starttaskmaster to this task later.
 *
 *         The function operates within an infinite loop, with a fixed delay for task scheduling.
 *         This design allows for consistent execution of background tasks without monopolizing
 *         the CPU, thereby maintaining system responsiveness.
 *
 *         Usage Context:
 *         This task is critical in systems where multiple background operations need to be
 *         handled concurrently without specific module associations, such as in industrial
 *         automation systems. It ensures seamless integration of various system components and
 *         contributes to the overall reliability and efficiency of the system.
 *
 * @param  argument: Not used. Placeholder for RTOS task compatibility.
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN StartDefaultTask */
	uint16_t injectCounter = 0;
	//to move to pid task
	restore_alarm_data();
	uint8_t alarmTripsTheSystem = 0;
	uint8_t systemTripBecauseAlarm = 0;
	//current state of digital input one from the board that is connected to dry run sensor
	uint8_t inputOnestate = 0;
	//end to move to pid task

	//PrintWeeklyValidRecords();

	/* Infinite loop */
	for (;;)
	{

		//alarm Section

		//if the system PID was stopped because of any kind of system fault
		//mark it as the cause of the system trip due to alarm not by the user
		if (pressureAlarmEnable && pressureSensorError && pressureSysStop)
		{
			alarmTripsTheSystem = 1;
		}
		else if (enableDryRun && dryRunDetected && dryRunSysStop)
		{
			alarmTripsTheSystem = 1;
		}
		else
		{
			alarmTripsTheSystem = 0;
		}
		//if an alarm trips the system stop the pid
		if (alarmTripsTheSystem && enablePID == 1)
		{
			printf("alarm trip disabled pid\n");
			controlAction = 0;
			accumulatedError = 0;
			enablePID = 0;
			enableScheduler = 0;
			systemTripBecauseAlarm = 1;
		}
		//else start the pid again
		else
		{
			if (systemTripBecauseAlarm == 1 && alarmTripsTheSystem == 0)
			{
				enablePID = 1;
				enableScheduler = 1;
				systemTripBecauseAlarm = 0;
				printf("alarm Clearing enabled PID \n");
			}
		}
		//if dry run is enabled start reading and acting due to the pid of dry run sensor
		if (enableDryRun)
		{
			inputOnestate = HAL_GPIO_ReadPin(GPIOC, Din1_Pin);
			if (inputOnestate == GPIO_PIN_SET)
			{
				dryRunDetected = 1;
			}
			else
			{
				dryRunDetected = 0;

			}
		}

// if the user requested to change the date and time of the rtc from the app set it
		if (setRTC == 1)
		{
			// The array of char that is having the received string from the app is named RTCString
			//char RTCString[25] = "2021-05-11T01:14:00";  // Replace this line with the actual received string

			uint16_t year;
			uint8_t month, day, hour, minute, weekDay;

			// Temporary variables for parsing
			int temp_month, temp_day, temp_hour, temp_minute, temp_weekDay;

			int parsed = sscanf(RTCString, "%4hu-%2d-%2dT%2d:%2d:00,%1d", &year,
					&temp_month, &temp_day, &temp_hour, &temp_minute,
					&temp_weekDay);

			// Assign the parsed values to the uint8_t variables
			month = temp_month;
			day = temp_day;
			hour = temp_hour;
			minute = temp_minute;
			weekDay = temp_weekDay;
			// Print each variable
//				printf("Year: %d\n", year);
//				printf("Month: %d\n", month);
//				printf("Day: %d\n", day);
//				printf("Hour: %d\n", hour);
//				printf("Minute: %d\n", minute);
//				printf("weekDay %d\n",weekDay);

			if (parsed == 6) // Check if all five components were successfully parsed
			{

				HAL_PWR_EnableBkUpAccess(); // Enable access to the backup domain
				Set_RTC_Values(year, month, day, hour, minute, weekDay); // Set the parsed values
				HAL_PWR_DisableBkUpAccess(); // Disable access to the backup domain

				setRTC = 0;
				sendRTC = 1;
			}
			else
			{
				// Handle the error if parsing failed
				Error_Handler();
			}
		}
//react to rgb led changes
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, redValue);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, greenValue);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, blueValue);
		// get the current rtc values
		Get_RTC_Values();
// a counter that is responsable for setting a flag that will tirgger a test for if any vfd not responded to modbus requests
		injectCounter++;
		if (injectCounter > 2000)
		{
			testIFNoVfdResponse = 1;
			injectCounter = 0;
		}
		osDelay(5);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_UART1_task */
/**
 * @brief Function implementing the UART1Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UART1_task */
void UART1_task(void *argument)
{
	/* USER CODE BEGIN UART1_task */
	returned_ST_Data Data;
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &d_value, 1);
	/* Infinite loop */
	for (;;)
	{
		//checks if the uart have recived data from the esp32 which is connected to the main board microcontroller
		//stm32h750
		if (usart1HaveNewData)
		{
//			printf("id %s \n", Data.identifier);
//			printf("id %s \n", Data.value);
//			printf("\n");
			// printf("the coming data is %s \n",getBuffer);
			//get the received id and value after the custom crc check
			Data = ReceiveData();
			//check for the type of id of recived value and update the corresponding variable with the  value .
			//after converting it from string to (int or float )
			if (strcmp(Data.identifier, "ledRed") == 0
					&& redValue != (64000 - atoi((Data.value)) * 250))
			{
				redValue = 64000 - atoi((Data.value)) * 250;
				//printf("the red value %d\n", redValue);
			}

			if (strcmp(Data.identifier, "ledGreen") == 0
					&& greenValue != (64000 - atoi((Data.value)) * 250))
			{
				greenValue = 64000 - atoi((Data.value)) * 250;
				//printf("the green value %d\n", greenValue);
			}
			if (strcmp(Data.identifier, "ledBlue") == 0
					&& blueValue != (64000 - atoi((Data.value)) * 250))
			{
				blueValue = 64000 - atoi((Data.value)) * 250;
				//printf("the blue value %d\n", blueValue);
			}

			if (strcmp(Data.identifier, "setPointRequiredStmRecive") == 0)
			{
				setPointRequired = atof(Data.value);

				//printf("setPoint: %.3f\n", setPointRequired);
			}

			if (strcmp(Data.identifier, "offset") == 0
					&& atoi(Data.value) != offset)
			{
				offset = atoi(Data.value);
				//printf("offset: %d\n", offset);
			}
			if (strcmp(Data.identifier, "cutIn") == 0
					&& atof(Data.value) != cutIn)
			{
				cutIn = atof(Data.value);
				//printf("cutIn: %f\n", cutIn);
			}
			if (strcmp(Data.identifier, "cutOff") == 0
					&& atof(Data.value) != cutOff)
			{
				cutOff = atof(Data.value);
				//printf("cutOff: %f\n", cutOff);
			}
			if (strcmp(Data.identifier, "vfdType") == 0
					&& strcmp(Data.value, vfdType) != 0)
			{
				strcpy(vfdType, Data.value);
				//printf("vfdType: %s\n", vfdType);
			}
			if (strcmp(Data.identifier, "systemPumpsNumber") == 0
					&& atoi(Data.value) != systemPumpsNumber)
			{
				systemPumpsNumber = atoi(Data.value);
				//printf("systemPumpsNumber: %d\n", systemPumpsNumber);
			}

			if (strcmp(Data.identifier, "minimumPressure") == 0
					&& atof(Data.value) != minimumPressure)
			{
				minimumPressure = atof(Data.value);
				//printf("minimumPressure: %.3f\n", minimumPressure);
			}
			if (strcmp(Data.identifier, "MaximumPressure") == 0
					&& atof(Data.value) != MaximumPressure)
			{
				MaximumPressure = atof(Data.value);
				//printf("MaximumPressure: %.3f\n", MaximumPressure);
			}

			if (strcmp(Data.identifier, "enableDryRun") == 0
					&& atoi(Data.value) != enableDryRun)
			{
				enableDryRun = atoi(Data.value);
				//	printf("enableDryRun: %d\n", enableDryRun);
			}

			if (strcmp(Data.identifier, "enablePID") == 0
					&& atoi(Data.value) != enablePID)
			{
				enablePID = atoi(Data.value);
				//printf("enablePID: %d\n", enablePID);
			}

			if (strcmp(Data.identifier, "enableScheduler") == 0
					&& atoi(Data.value) != enableScheduler)
			{
				enableScheduler = atoi(Data.value);
				//printf("enableScheduler: %d\n", enableScheduler);
			}

			if (strcmp(Data.identifier, "currentOfTheFirstReading") == 0)
			{
				currentOfTheFirstReading = atof(Data.value);
				//printf("currentOfTheFirstReading: %.8f\n",
				//	currentOfTheFirstReading);
				firstCurrentReadingRecived = 1;
			}

			if (strcmp(Data.identifier, "currentOfTheSecondReading") == 0)
			{
				currentOfTheSecondReading = atof(Data.value);
				//printf("currentOfTheSecondReading: %.8f\n",
				//currentOfTheSecondReading);
				secondCurrentReadingRecived = 1;
			}

			if (strcmp(Data.identifier, "coffA") == 0
					&& atof(Data.value) != coffA)
			{
				coffA = atof(Data.value);
				//printf("coffA: %.8f\n", coffA);
			}

			if (strcmp(Data.identifier, "coffB") == 0
					&& atof(Data.value) != coffB)
			{
				coffB = atof(Data.value);
				//printf("coffB: %.8f\n", coffB);
			}

			if (strcmp(Data.identifier, "changeOverTime") == 0
					&& atoi(Data.value) != changeOverTime)
			{

				changeOverTime = atoi(Data.value);
				//printf("changeOverTime: %d\n", changeOverTime);

			}

			if (strcmp(Data.identifier, "KP") == 0 && atof(Data.value) != Kp)
			{
				Kp = atof(Data.value);
				//printf("kp: %.8f\n", Kp);
			}

			if (strcmp(Data.identifier, "KI") == 0 && atof(Data.value) != Ki)
			{
				Ki = atof(Data.value);
				KIreceived = 1;
				//printf("ki: %.8f\n", Ki);
			}

			if (strcmp(Data.identifier, "KD") == 0 && atof(Data.value) != Kd)
			{
				Kd = atof(Data.value);
				//printf("kd: %.8f\n", Kd);
			}

			if (strcmp(Data.identifier, "save") == 0)
			{
				save = atof(Data.value);
				//printf("save: %d\n", save);
			}

			if (strcmp(Data.identifier, "getFactoryData") == 0)
			{
				getFactoryData = atof(Data.value);
				//printf("getFactoryData: %d\n", getFactoryData);
			}

			//end factory page data



			if (strcmp(Data.identifier, "ID") == 0
					&& atoi(Data.value) != Slave_ID)
			{
				Slave_ID = atoi(Data.value);
				//printf("the id of the slave: %d\n", Slave_ID);
			}
			if (strcmp(Data.identifier, "functionNumber") == 0
					&& atoi(Data.value) != functionNumber)
			{
				functionNumber = atoi(Data.value);
				//printf("the function number : %d\n", functionNumber);
			}

			if (strcmp(Data.identifier, "slaveAddress") == 0
					&& atoi(Data.value) != slaveAddress)
			{
				slaveAddress = atoi(Data.value);
				//printf("the slave address : %d\n", slaveAddress);
			}

			if (strcmp(Data.identifier, "dataToSend") == 0
					&& atoi(Data.value) != dataToSend)
			{
				dataToSend = atoi(Data.value);
				//printf("the data to send : %d\n", dataToSend);
			}

			if (strcmp(Data.identifier, "dataToSend") == 0
					&& atoi(Data.value) != dataToSend)
			{
				dataToSend = atoi(Data.value);
				//printf("the data to send : %d\n", dataToSend);
			}
			if (strcmp(Data.identifier, "operationMode") == 0)
			{
				operationMode = atoi(Data.value);

				//printf("the data to send : %d\n", dataToSend);
			}

			if (operationMode == MANUAL_MODE)
			{
				if (strcmp(Data.identifier, "speedPercentageOfPump1") == 0)
				{
					manualPumpID = 1;
					manualPumpSpeed = (atoi(Data.value)*DanfossVfdMaxSpeed)/100;

				}
				if (strcmp(Data.identifier, "speedPercentageOfPump2") == 0)
				{
					manualPumpID = 2;
					manualPumpSpeed = (atoi(Data.value)*DanfossVfdMaxSpeed)/100;
				}
				if (strcmp(Data.identifier, "speedPercentageOfPump3") == 0)
				{
					manualPumpID = 3;
					manualPumpSpeed = (atoi(Data.value)*DanfossVfdMaxSpeed)/100;
				}
				if (strcmp(Data.identifier, "speedPercentageOfPump4") == 0)
				{
					manualPumpID = 4;
					manualPumpSpeed = (atoi(Data.value)*DanfossVfdMaxSpeed)/100;
				}
				if (strcmp(Data.identifier, "speedPercentageOfPump5") == 0)
				{
					manualPumpID = 5;
					manualPumpSpeed = (atoi(Data.value)*DanfossVfdMaxSpeed)/100;
				}
				if (strcmp(Data.identifier, "speedPercentageOfPump6") == 0)
				{
					manualPumpID = 6;
					manualPumpSpeed = (atoi(Data.value)*DanfossVfdMaxSpeed)/100;
				}
				if (strcmp(Data.identifier, "speedPercentageOfPump7") == 0)
				{
					manualPumpID = 7;
					manualPumpSpeed = (atoi(Data.value)*DanfossVfdMaxSpeed)/100;
				}
				if (strcmp(Data.identifier, "speedPercentageOfPump8") == 0)
				{
					manualPumpID = 8;
					manualPumpSpeed = (atoi(Data.value)*DanfossVfdMaxSpeed)/100;
				}
			}

			if (strcmp(Data.identifier, "changeRTC") == 0)
			{
				// Ensure that Data.value is null-terminated and doesn't overrun RTCString
				strncpy(RTCString, Data.value, sizeof(RTCString) - 1);
//				printf("%s\n",Data.value);

				// Null-terminate RTCString in case Data.value was too long
				RTCString[sizeof(RTCString) - 1] = '\0';
				setRTC = 1;
			}

			if (strcmp(Data.identifier, "sendNow") == 0)
			{

				sendNow = 1;
			}

			if (strcmp(Data.identifier, "getSchedularData") == 0)
			{
				sendRTC = 1;
				sendSchedData = 1;
				printf("send rtc\n");
			}
			if (strcmp(Data.identifier, "getAlarmsdss") == 0)
			{
				if (Data.value[0] == '1')
				{
					sendAlarms = 1;
					sendAlarmRecords = 1;

				}
				else if (Data.value[0] == '0')
				{
					sendAlarms = 0;
					sendAlarmRecords = 0;
				}

			}

			if (strcmp(Data.identifier, "DryRunActive") == 0)
			{
				enableDryRun = atoi(Data.value);

			}
			if (strcmp(Data.identifier, "DryRunSysStop") == 0)
			{

				dryRunSysStop = atoi(Data.value);

			}
			if (strcmp(Data.identifier, "pressureActive") == 0)
			{
				pressureAlarmEnable = atoi(Data.value);
			}
			if (strcmp(Data.identifier, "pressureSysStop") == 0)
			{
				pressureSysStop = atoi(Data.value);
				store_alarm_data(); //put this on the last alarm to save the values sended from the app
			}

			if (strcmp(Data.identifier, "startTime") == 0)
			{

				strncpy(startTime, Data.value, sizeof(startTime) - 1);
				printf("starttime %s\n", startTime);

			}

			if (strcmp(Data.identifier, "weeklyRecord") == 0)
			{

				strncpy(weeklyRecord, Data.value, sizeof(weeklyRecord) - 1);
				printf("weeklyRecord %s\n", weeklyRecord);
				weeklyRecordFlag = 1;

			}
			if (strcmp(Data.identifier, "stopTime") == 0)
			{
				strncpy(stopTime, Data.value, strlen(Data.value));

				// Clear fullDatedSchedRecord
				memset(fullDatedSchedRecord, 0, sizeof(fullDatedSchedRecord));

				// Copy startTime into fullDatedSchedRecord
				strncpy(fullDatedSchedRecord, startTime, strlen(startTime));
				// Concatenate a comma into fullDatedSchedRecord
				strncat(fullDatedSchedRecord, ",", 1);
				// Concatenate stopTime into fullDatedSchedRecord
				strncat(fullDatedSchedRecord, stopTime, strlen(stopTime));

				// printf("stopTime %s\n", stopTime);
				//   printf("fullDatedSchedRecord %s\n", fullDatedSchedRecord);
				check_specific_record(numberOfSchedulerRecords, startTime,
						stopTime);

				stopTimeFlag = 1;
			}

			if (strcmp(Data.identifier, "removeSchedRecord") == 0)
			{

				indexOfRecordToRemove = atoi(Data.value);
				printf("sched record to delete with id  %d\n",
						indexOfRecordToRemove);
				removeRecordFlag = 1;

			}
			if (strcmp(Data.identifier, "removeWeeklySchedRecord") == 0)
			{

				indexOfRecordToRemove = atoi(Data.value);
				printf("weekly sched record to delete with id  %d\n",
						indexOfRecordToRemove);
				removeWeeklyRecordFlag = 1;

			}
			if (strcmp(Data.identifier, "inHome") == 0
					&& atoi(Data.value) != inHomePage)
			{
				inHomePage = atoi(Data.value);
				if(inHomePage==1)
				{
					sendOperationMode=1;
				}
				printf("inHome: %d\n", inHomePage);
			}

			if (strcmp(Data.identifier, "removeAllRecords") == 0
					&& atoi(Data.value) == 1)
			{
				printf("erase all alarms\n");
				eraseAlarmsRecordsFlag = 1;
			}

			if (strcmp(Data.identifier, "removeAllRecords") == 0
					&& atoi(Data.value) == 1)
			{
				printf("erase all alarms\n");
				eraseAlarmsRecordsFlag = 1;
			}

			if (strcmp(Data.identifier, "disableTimeController") == 0
					&& atoi(Data.value) != date_week_sched_disabled)
			{
				date_week_sched_disabled = atoi(Data.value);
				printf("date_week_sched_disabled: %d\n",
						date_week_sched_disabled);

				if (matchedRecordCount == 0 && matchedWeeklyRecordCount == 0
						&& date_week_sched_disabled == 0)
				{
					enablePID = 0;
					schedDisabledSystem = 1;
					printf("i am if \n");
				}
				else
				{
					enablePID = 1;
					schedDisabledSystem = 0;
					printf("i am else \n");
				}

			}

		}
		osDelay(5);
	}
	/* USER CODE END UART1_task */
}

/* USER CODE BEGIN Header_ADC_4_20mApolling */
/**
 * @brief Function implementing the Task4_20mAscan thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ADC_4_20mApolling */
void ADC_4_20mApolling(void *argument)
{
	/* USER CODE BEGIN ADC_4_20mApolling */
	uint8_t firstCurrentRecived = 0;
	uint8_t secondCurrentRecived = 0;

	for (;;)
	{
		// Start the ADC conversion process for ADC1. This initiates the ADC to begin converting the analog signal to a digital value.
		HAL_ADC_Start(&hadc1);

		// Poll the ADC1 to complete the conversion. HAL_MAX_DELAY is used to wait indefinitely until the ADC conversion is complete.
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

		// Retrieve the converted digital value from ADC1. This value is the digital representation of the analog signal read by ADC1.
		uint32_t adcValue = HAL_ADC_GetValue(&hadc1);

		//the system is monitoring 4-20 ml industrial sensor and  the the lowest reading of it
		//is 4 mA so below that with a certain amount considered faulty sensor or the sensor it not connected
		//so set the flag that will stop the pid and send the alarm
		if (adcValue < 5000)
		{
			pressureSensorError = 1;
		}
		else
		{
			pressureSensorError = 0;
		}
		//printf("Raw ADC Value = %u\n", adcValue);
		//if the user send the first calibarion reading from the app record it
		if (firstCurrentReadingRecived)
		{
			firstADCCalibrationReading = adcValue;

			firstCurrentRecived = 1;
			firstCurrentReadingRecived = 0;
			//printf("First ADC Calibration Reading = %u\n",
					//firstADCCalibrationReading);
			//printf("Current of the First Reading = %.8f\n",
				//	currentOfTheFirstReading);
		}
//if the user sent the second 4-20mA reading record it
		if (secondCurrentReadingRecived)
		{
			secondADCCalibrationReading = adcValue;

			secondCurrentRecived = 1;
			//printf("Second ADC Calibration Reading = %u\n",
					//secondADCCalibrationReading);
			//printf("Current of the Second Reading = %.8f\n",
				//	currentOfTheSecondReading);
			secondCurrentReadingRecived = 0;
		}
//if both both readings was received start calibartion the coff A and B
		if (firstCurrentRecived && secondCurrentRecived)
		{
			CalibrateADC();
			firstCurrentRecived = 0;
			secondCurrentRecived = 0;

		//	printf("Calibration done.\n");
		}

//get the calibrated current
		float calibratedCurrent = coffA * adcValue + coffB;
		//printf("Calibrated Current = %.8f\n", calibratedCurrent);


//calculate the current pressure
		currentPressure = (calibratedCurrent - 4) * (MaximumPressure / 16);
		//printf("Current Pressure = %.8f\n", currentPressure);

		//printf("\n");

		osDelay(50);
	}
	/* USER CODE END ADC_4_20mApolling */
}

/* USER CODE BEGIN Header_StartTaskMaster */
/**
 * @brief Function implementing the myTaskMaster thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskMaster */
void StartTaskMaster(void *argument)
{
	/* USER CODE BEGIN StartTaskMaster */
	telegram[0].u16CoilsNo = 1; // number of elements (coils or registers) to read
	uint8_t counter = 0;
	telegram[0].u8id = 1;
	uint8_t totalTripPumps = 0;

	/* Infinite loop */
	for (;;)
	{

				if (sendNow == 1)
				{
					//send custom command coming from the app
					if (functionNumber > 0 && functionNumber < 5)
					{
						telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
						ModbusRequest(&responseData, Slave_ID, functionNumber,
								slaveAddress, &telegram[1], osThreadGetId(),
								TimeoutValue);
						sendNow = 0;
						thereIsData = 1;

					}
					else if (functionNumber == 5 || functionNumber == 6)
					{
						telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
						ModbusRequest(&dataToSend, Slave_ID, functionNumber,
								slaveAddress, &telegram[1], osThreadGetId(),
								TimeoutValue);
						sendNow = 0;

					}
				}


				else if (enablePID == 1)
		{
			if (testIFNoVfdResponse)
			{
				for (int i = 0; i < systemPumpsNumber; i++)
				{
					//set the pump speed to 2 to indicate trip reading the speed from the vfd
					//and the application will respond to the 2 and know it is trip so it will yellow the pump shapes
					//if it remains 2 for certain amount of time
					speedPercentageOfPump[i] = 2;
					testIFNoVfdResponse = 0;

				}
			}

			for (int i = 0; i < systemPumpsNumber; i++)
			{
				if (speedPercentageOfPump[i] == 2)
				{
					totalTripPumps++;
				}
			}
			if (totalTripPumps == systemPumpsNumber)
			{
				totalTripPumps = 0;
				fullSystemTrip = 1;
			}
			else
			{
				totalTripPumps = 0;
				fullSystemTrip = 0;
			}

			osDelay(50);

		}
		else
		{
			osDelay(500);
		}

	}
	/* USER CODE END StartTaskMaster */
}

/* USER CODE BEGIN Header_external_flash */
/**
 * @brief Function implementing the ExternalFlash thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_external_flash */
void external_flash(void *argument)
{
	/* USER CODE BEGIN external_flash */

	volatile uint8_t debugging_flag_test_store_alarms = 0;
	volatile uint8_t enableMemoryMappedFlag;

//
	// Call the store_alarm function for dryRun alarm

	/* Infinite loop */
	for (;;)
	{

		if (debugging_flag_test_store_alarms == 1)
		{

			HAL_StatusTypeDef statusDryRun = store_alarm(dryRun,
			ALARMS_START_ADDRESS);
			printf("Store alarm (dryRun) status: %d\n", statusDryRun);

			// Call the store_alarm function for noPressureSensor alarm
			HAL_StatusTypeDef statusNoPressureSensor = store_alarm(
					noPressureSensor, ALARMS_START_ADDRESS);
			printf("Store alarm (noPressureSensor) status: %d\n",
					statusNoPressureSensor);

			// Call the read_alarm_records function
			HAL_StatusTypeDef statusReadAlarms = read_alarm_records(
			ALARMS_START_ADDRESS, alarmsArray, &storedAlarmsNumber);
			printf("Read alarm records status: %d\n", statusReadAlarms);
			printf("Number of alarm records read: %lu\n", storedAlarmsNumber);

			for (uint32_t i = 0; i < storedAlarmsNumber; i++)
			{
				printf("Alarm Record %lu: %s\n", i + 1, alarmsArray[i]);
			}
			debugging_flag_test_store_alarms = 0;
		}

		if (eraseDatedRecordsFlag)
		{
			S25FL256S_WriteEnable(&hqspi, S25FL256S_SPI_MODE);
			flStatus = S25FL256S_BlockErase(&hqspi, S25FL256S_SPI_MODE,
			DATED_SCHEDULAR_RECORDS_START_ADDRESS, S25FL256S_ERASE_64K);
			printf(" Erase, status = %ld\r\n", flStatus);
			if (flStatus == S25FL256S_OK)
			{
				write_backup_register(datedSchedRegisterNumber, 1);
				numberOfSchedulerRecords = 1;
			}

			eraseDatedRecordsFlag = 0;
		}

		if (eraseWeeklyRecordsFlag)
		{
			S25FL256S_WriteEnable(&hqspi, S25FL256S_SPI_MODE);
			flStatus = S25FL256S_BlockErase(&hqspi, S25FL256S_SPI_MODE,
			WEEKLY_SCHEDULAR_RECORDS_START_ADDRESS, S25FL256S_ERASE_64K);
			printf(" Erase, status = %ld\r\n", flStatus);
			if (flStatus == S25FL256S_OK)
			{

				write_backup_register(numberOfWeeklyRecordsRegNum, 1);
				numberOfWeeklyRecords = 1;
			}
			eraseWeeklyRecordsFlag = 0;
		}

		if (eraseAlarmsRecordsFlag)
		{
			S25FL256S_WriteEnable(&hqspi, S25FL256S_SPI_MODE);
			flStatus = S25FL256S_BlockErase(&hqspi, S25FL256S_SPI_MODE,
			ALARMS_START_ADDRESS, S25FL256S_ERASE_64K);
			printf(" Erase, status = %ld\r\n", flStatus);
			if (flStatus == S25FL256S_OK)
			{

				write_backup_register(numberOfAlarmsRegNum, 1);
				storedAlarmsNumber = 0;
			}
			eraseAlarmsRecordsFlag = 0;
		}
		if (eraseEntireChipFlag)
		{

			uint8_t flagStatus = S25FL256S_WriteEnable(&hqspi,
					S25FL256S_SPI_MODE);
			flStatus = S25FL256S_BlockErase(&hqspi, S25FL256S_SPI_MODE, 0,
					S25FL256S_ERASE_CHIP);
			printf(" Erase, status = %ld\r\n", flStatus);
			//wait 130ms for sector erase

			if (flagStatus == S25FL256S_OK)
			{
				printf("erased the entire chip successfully\n");

				write_backup_register(datedSchedRegisterNumber, 1);

				write_backup_register(numberOfWeeklyRecordsRegNum, 1);
				write_backup_register(numberOfAlarmsRegNum, 0);

			}
			else if (flagStatus == S25FL256S_ERROR)
			{
				printf("faild to erased the entire chip \n");
			}

			eraseEntireChipFlag = 0;
		}

		if (eraseExternalFlashFlag == 1)
		{

			S25FL256S_WriteEnable(&hqspi, S25FL256S_SPI_MODE);
			flStatus = S25FL256S_BlockErase(&hqspi, S25FL256S_SPI_MODE, 0,
					S25FL256S_ERASE_64K);
			printf(" Erase, status = %ld\r\n", flStatus);
			//wait 130ms for sector erase

			eraseExternalFlashFlag = 0;
		}

		if (enableMemoryMappedFlag)
		{
			flStatus = S25FL256S_EnableMemoryMappedModeSTR(&hqspi,
					S25FL256S_SPI_1I4O_MODE);

			enableMemoryMappedFlag = 0;
		}

		if (removeRecordFlag)
		{
			DeleteRecordByIndex(&hqspi, indexOfRecordToRemove,
			DATED_SCHEDULAR_RECORDS_START_ADDRESS, DATED_RECORD_SIZE);

			removeRecordFlag = 0;

			ExtractRecords(&hqspi);
			compare_rtc();
			sendMatchedIndices(matchedRecordIndices, &matchedRecordCount,
					"matchDatedIndex");

			if (matchedRecordCount == 0 && matchedWeeklyRecordCount == 0
					&& date_week_sched_disabled == 0)
			{
				enablePID = 0;
				schedDisabledSystem = 1;
			}
			else
			{
				enablePID = 1;
				schedDisabledSystem = 0;
			}

			osDelay(5);
		}

		if (removeWeeklyRecordFlag)
		{
			DeleteRecordByIndex(&hqspi, indexOfRecordToRemove,
			WEEKLY_SCHEDULAR_RECORDS_START_ADDRESS, WEEKLY_RECORD_SIZE);
			ExtractWeeklyRecords(&hqspi);
			compare_weekly_rtc();

			// For sending weekly indices
			sendMatchedIndices(matchedWeeklyRecordIndices,
					&matchedWeeklyRecordCount, "matchedWeeklyIndex");

			if (matchedRecordCount == 0 && matchedWeeklyRecordCount == 0
					&& date_week_sched_disabled == 0)
			{
				enablePID = 0;
				schedDisabledSystem = 1;
			}
			else
			{
				enablePID = 1;
				schedDisabledSystem = 0;
			}
			removeWeeklyRecordFlag = 0;

			osDelay(5);
		}

		if (stopTimeFlag)
		{
			externalFlashWrite(DATED_SCHEDULAR_RECORDS_START_ADDRESS,
					fullDatedSchedRecord, &numberOfSchedulerRecords,
					DATED_RECORD_SIZE, datedSchedRegisterNumber);

			ExtractRecords(&hqspi);

			compare_rtc();
			sendMatchedIndices(matchedRecordIndices, &matchedRecordCount,
					"matchDatedIndex");

			if (matchedRecordCount == 0 && matchedWeeklyRecordCount == 0
					&& date_week_sched_disabled == 0)
			{
				enablePID = 0;
				schedDisabledSystem = 1;
			}
			else
			{
				enablePID = 1;
				schedDisabledSystem = 0;
			}
			stopTimeFlag = 0;

		}

		if (weeklyRecordFlag)
		{

			externalFlashWrite(WEEKLY_SCHEDULAR_RECORDS_START_ADDRESS,
					weeklyRecord, &numberOfWeeklyRecords, WEEKLY_RECORD_SIZE,
					numberOfWeeklyRecordsRegNum);
			ExtractWeeklyRecords(&hqspi);

			// For sending weekly indices

			compare_weekly_rtc();
			sendMatchedIndices(matchedWeeklyRecordIndices,
					&matchedWeeklyRecordCount, "matchedWeeklyIndex");

			if (matchedRecordCount == 0 && matchedWeeklyRecordCount == 0
					&& date_week_sched_disabled == 0)
			{
				enablePID = 0;
				schedDisabledSystem = 1;
			}
			else
			{
				enablePID = 1;
				schedDisabledSystem = 0;
			}
			weeklyRecordFlag = 0;
		}
		osDelay(10);
	}

	/* USER CODE END external_flash */
}

/* USER CODE BEGIN Header_UART_Send */
/**
 * @brief Implements the UART_Send thread for transmitting data to external devices via UART1.
 *        This task is responsible for serially transmitting various system parameters, alarms,
 *        and operational states to an external interface (like an app). It covers a broad range
 *        of data including current pressure, system errors, calibration settings, and real-time
 *        clock values. Adhering to MISRA C:2012 guidelines, the task ensures safety and reliability
 *        in data transmission within an embedded system context.
 *
 *        Primary Operations:
 *        - Periodic data transmission based on system states and user interface interactions.
 *        - Formatting and sending system parameters, error states, and configuration settings.
 *        - Handling conditional logic for data transmission based on system and user interface states.
 *
 *        The task plays a crucial role in maintaining real-time communication with external interfaces,
 *        providing necessary feedback and system status updates for user interaction and system monitoring.
 *
 * @param argument: Not used, included for RTOS task compatibility.
 * @retval None
 */
/* USER CODE END Header_UART_Send */
void UART_Send(void *argument)
{
	/* USER CODE BEGIN UART_Send */
	char value[15];
	char *DataSend;
	int index = 0;
	uint8_t index1 = 0;

	uint8_t prevPressureSensorError = 0;

	uint16_t dryRunSendCounter = 99;

	/* Infinite loop */
	for (;;)
	{
//if the user in homepage and pid is enabled start sending the data of this page
		if (enablePID == 1 && inHomePage)
		{
			switch (index)
			{
			case 0:
				sprintf(value, "%.2f", currentPressure);
				DataSend = PrepareData("currentPressure", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
				break;

			case 1:
				if (prevPressureSensorError != pressureSensorError
						|| sendAlarms)
				{
					sprintf(value, "%d", pressureSensorError);
					DataSend = PrepareData("pressureError", value);
					HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
							strlen(DataSend), 10);
					prevPressureSensorError = pressureSensorError;

				}
				break;

			case 2:

				sprintf(value, "%f", setPointRequired);
				DataSend = PrepareData("setPointRequired", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);

				break;
			default:
				break;

			}
			index++;
			if (index > 3)
			{
				index = 0; // Reset the index to start from the beginning again
			}

			//send the speed of each pump and the current drawn by each pump.
			for (int i = 0; i < systemPumpsNumber; i++)
			{

				sprintf(value, "%d", speedPercentageOfPump[i]);
				char key[50];
				sprintf(key, "speedPercentageOfPump%d", i + 1);
				DataSend = PrepareData(key, value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 20);
				osDelay(20);

				sprintf(value, "%d", pumpCurrent[i]);
				char key1[50];
				sprintf(key1, "pump%dCurrent", i + 1);
				DataSend = PrepareData(key1, value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 20);

				osDelay(20);

			}

		}


//if the user enterd the factory page data which contain the system setting send the saved data to the app
		//to be shown in the app
		if (getFactoryData == 1)
		{

			switch (index1)
			{
			case 0:
				sprintf(value, "%f", setPointRequired);
				DataSend = PrepareData("setPointOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("1\n");
				break;
			case 1:
				sprintf(value, "%f", Kp);
				DataSend = PrepareData("KpOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("2\n");
				break;
			case 2:
				sprintf(value, "%f", Ki);
				DataSend = PrepareData("KiOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("3\n");
				break;

			case 3:
				sprintf(value, "%f", Kd);
				DataSend = PrepareData("KdOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("4\n");
				break;

			case 4:
				sprintf(value, "%d", offset);
				DataSend = PrepareData("offsetOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("5\n");
				break;

			case 5:
				sprintf(value, "%f", cutIn);
				DataSend = PrepareData("cutInOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("17\n");
				break;
			case 6:
				sprintf(value, "%f", cutOff);
				DataSend = PrepareData("cutOffOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("18\n");
				break;
			case 7:
				sprintf(value, "%d", changeOverTime);
				DataSend = PrepareData("changeOverTimeOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("6\n");
				break;
			case 8:
				sprintf(value, "%d", systemPumpsNumber);
				DataSend = PrepareData("systemPumpsNumberOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("7\n");
				break;
			case 9:
				sprintf(value, "%f", currentOfTheFirstReading);
				DataSend = PrepareData("currentOfTheFirstReadingOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("8\n");
				break;
			case 10:
				sprintf(value, "%f", currentOfTheSecondReading);
				DataSend = PrepareData("currentOfTheSecondReadingOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("9\n");
				break;
			case 11:
				sprintf(value, "%f", coffA);
				DataSend = PrepareData("coffAOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("10\n");
				break;
			case 12:
				sprintf(value, "%f", coffB);
				DataSend = PrepareData("coffBOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("11\n");
				break;
			case 13:
				sprintf(value, "%f", minimumPressure);
				DataSend = PrepareData("LowestSensorOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("12\n");
				break;
			case 14:
				sprintf(value, "%f", MaximumPressure);
				DataSend = PrepareData("HigestSensorOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("13\n");
				break;
			case 15:
				sprintf(value, "%d", enableDryRun);
				DataSend = PrepareData("enableDryRunOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("14\n");
				break;
			case 16:
				sprintf(value, "%d", enablePID);
				DataSend = PrepareData("enablePIDOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("15\n");
				break;
			case 17:
				sprintf(value, "%d", enableScheduler);
				DataSend = PrepareData("enableSchedulerOfStm", value);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
//				printf("16\n");
				break;

			default:
				break;
			}
			index1++;
			if (index1 > 17)
			{
				index1 = 0;
				getFactoryData = 0;
			}

		}
		//send the state of dryrun state to the system
		if (enablePID == 1 && inHomePage)
		{
			if (dryRunSendCounter >= 200)
			{
				for (uint8_t i = 0; i < 2; i++)
				{
					sprintf(value, "%d", dryRunDetected);
					DataSend = PrepareData("dryRunDetected", value);
					HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
							strlen(DataSend), 10);
					osDelay(10);
				}
				dryRunSendCounter = 0;

			}
		}
		else
		{
			if (dryRunSendCounter >= 1000)
			{
				for (uint8_t i = 0; i < 2; i++)
				{
					sprintf(value, "%d", dryRunDetected);
					DataSend = PrepareData("dryRunDetected", value);
					HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
							strlen(DataSend), 10);
					osDelay(10);
				}
				dryRunSendCounter = 0;

			}
		}

		dryRunSendCounter++;
// when the user enter the scheduler page on the app send the saved records to the app to display it
		if (sendRTC)
		{
			Get_RTC_Values();
			char formatted_datetime[20]; // Buffer to hold the formatted date and time string

			sprintf(formatted_datetime, "%02d:%02d %02d/%02d/%04d", sTime.Hours,
					sTime.Minutes, sDate.Date, sDate.Month, 2000 + sDate.Year); // Assuming the year is in the 2000s
			DataSend = PrepareData("RTC", formatted_datetime);
			HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend),
					10);
			osDelay(10);
			if (sendSchedData)
			{

				char value3[15];
				sprintf(value3, "%d", !date_week_sched_disabled);
				DataSend = PrepareData("TimeController", value3);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
				osDelay(10); // Adding a delay to ensure there's no data overlap

				// Loop to send each original_record from the valid_records array
				for (int i = 0; i < valid_record_count; i++)
				{
					char *currentRecord = valid_records[i].original_record;
					DataSend = PrepareData("schedRecords", currentRecord);
					HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
							strlen(DataSend), 10);
					osDelay(10); // Adding a delay to ensure there's no data overlap
				}
				// Loop to send each original_record from the valid_records array
				for (int i = 0; i < valid_weekly_record_count; i++)
				{
					char *currentRecord =
							valid_weekly_records[i].original_record;
					DataSend = PrepareData("weeklyschedRecords", currentRecord);
					HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
							strlen(DataSend), 10);
					osDelay(10); // Adding a delay to ensure there's no data overlap
				}
				char value1[15];
				sprintf(value1, "%lu", numberOfSchedulerRecords);
				DataSend = PrepareData("datedRecordIndex", value1);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
				osDelay(10); // Adding a delay to ensure there's no data overlap
				char value2[15];
				sprintf(value2, "%lu", numberOfWeeklyRecords);
				DataSend = PrepareData("weeklyRecordIndex", value2);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
				osDelay(10); // Adding a delay to ensure there's no data overlap
				compare_rtc();
				compare_weekly_rtc();

				// For sending dated indices
				sendMatchedIndices(matchedRecordIndices, &matchedRecordCount,
						"matchDatedIndex");

				// For sending weekly indices
				sendMatchedIndices(matchedWeeklyRecordIndices,
						&matchedWeeklyRecordCount, "matchedWeeklyIndex");

				sendSchedData = 0;
			}

			sendRTC = 0;

		}
//if the user enterd the alarm and warning page send the alarm history records to this page .
		if (sendAlarmRecords)
		{
			for (uint32_t i = 0; i < storedAlarmsNumber; i++)
			{

				DataSend = PrepareData("alarmRecord", alarmsArray[i]);
				HAL_UART_Transmit(&huart1, (uint8_t*) DataSend,
						strlen(DataSend), 10);
				osDelay(10); // Adding a delay to ensure there's no data overlap
				printf("Alarm Record %lu: %s\n", i + 1, alarmsArray[i]);
			}
			sendAlarmRecords = 0;
		}
		//send the current state of possible alarms in the system and weither each alarm will stop the system
		if (sendAlarms == 1)
		{

			sprintf(value, "%d", dryRunDetected);
			DataSend = PrepareData("dryRunDetected", value);
			HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend),
					10);
			osDelay(10);

			sprintf(value, "%d", pressureSensorError);
			DataSend = PrepareData("pressureSensorError", value);
			HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend),
					10);
			osDelay(10);

			sprintf(value, "%d", enableDryRun);
			DataSend = PrepareData("enableDryRun", value);
			HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend),
					10);
			osDelay(10);

			sprintf(value, "%d", dryRunSysStop);
			DataSend = PrepareData("dryRunSysStop", value);
			HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend),
					10);
			osDelay(10);

			sprintf(value, "%d", pressureAlarmEnable);
			DataSend = PrepareData("pressureAlarmEnable", value);
			HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend),
					10);

			sprintf(value, "%d", pressureSysStop);
			DataSend = PrepareData("pressureSysStop", value);
			HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend),
					10);
			osDelay(10);
			osDelay(100);
			sendAlarms++;
			if (sendAlarms >= 4)
			{
				sendAlarms = 0;
			}
		}
		//when the user enter the home page send the current mode (manual or automatic) to display the approprate graphics on the app
		if(sendOperationMode==1)
		{
			sprintf(value, "%d", operationMode);
			DataSend = PrepareData("operationMode", value);
			HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend),
					10);

			HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend),
					10);
			HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend),
					10);
			sendOperationMode=0;
		}
		osDelay(10);
	}
	/* USER CODE END UART_Send */
}

/* USER CODE BEGIN Header_PIDControl */
/**
 * @brief Function implementing the PID thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PIDControl */
#if defined(MULTI_VFD)
void PIDControl(void *argument)
{

	/* USER CODE BEGIN PIDControl */
	//monitor the current speed of the pump that is risisng up
	uint16_t tempPumpToAwakeSpeed = 0;
	uint8_t startPumpsFlag = 0;

	telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
	telegram[1].u8id = 1;

	int16_t schedulerPumpToShutDownSpeed = 0;

	//delete_internal_system_data_from_backup_domain_registers();
	OldKI = Ki;
	/* Infinite loop */
	for (;;)
	{
		// if the user choosed automatic mocd from the app
		if (operationMode == AUTO_MODE)
		{
			//if the scheduler not changing the order of pumps and the pid is enabled
			if (arrangeChanged == 0 && enablePID == 1)
			{
				//if the pressure below cutin start increasing pumps speed
				if (currentPressure <= (setPointRequired - cutIn)
						&& startPumpsFlag == 0 && activePumps == 0)
				{
					startPumpsFlag = 1;
				}
				//if the pressure reached cutoff stop the pumps and pid
				else if (currentPressure >= (setPointRequired + cutOff)
						&& startPumpsFlag == 1)
				{
					startPumpsFlag = 0;
					accumulatedError = 0;
				}
				//calculating the paramters of pid to apply it to the current speed of each pump
				float error = setPointRequired - currentPressure;

				float Pout = Kp * error;
				if (KIreceived == 1)
				{
					// Scale the accumulated error
					if (OldKI != 0)
					{
						accumulatedError *= (OldKI / Ki);
					}
					else
					{
						accumulatedError = 0;
					}
					// Reset the flag and update oldKi
					KIreceived = 0;
					OldKI = Ki;
				}
				accumulatedError += error * 200; // deltaTime is the time step, e.g., 30ms

				float Iout = Ki * accumulatedError;

				// Derivative term calculation
				float deltaError = error - lastError;

				float Dout = Kd * deltaError; // deltaTime is the time step, e.g., 30ms

				if (startPumpsFlag == 1)
				{
					controlAction = Pout + Iout + Dout;
					if (controlAction < DanfossVfdMaxSpeed * .5 && controlAction > 0)
					{
						controlAction = DanfossVfdMaxSpeed * .5;
					}
				}
				else
				{
					controlAction = 0;
				}

				//dry run detection and shutting pumps down mechanism

				//dry run end

//			printf("delta error %f \n",deltaError);
//			printf("dout %f \n",Dout);
//			printf("error %d \n ",error);
//			printf("last error %f \n",lastError);
//			printf("control Action %f \n",controlAction);
//			printf("\n");

				lastError = error; // Save the current error for next loop iteration
				//	printf("pout %f \n",Pout);

				if (accumulatedError < 0)
				{
					accumulatedError = 0;

				}
				if (controlAction <= 0)
				{
					controlAction = 0;
					startPumpsFlag = 0;
					accumulatedError = 0;
				}
//limit the range of change so the system can react to pid changes normally
				if (controlAction > DanfossVfdMaxSpeed * systemPumpsNumber)
				{
					controlAction = DanfossVfdMaxSpeed * systemPumpsNumber;
					accumulatedError -= error * 200;
				}
//store the current values of pid so if system is reset the pid will continue from the point it stopped
				store_internal_system_data();
				controlActionDifference = controlAction - prevControlAction;

				prevControlAction = controlAction;
				if (controlActionDifference > 0)
				{
					// Calculate number of pumps to be activated based on the control action
					activePumps = (controlAction + DanfossVfdMaxSpeed
							+ offset / 2 - 1)
							/ (DanfossVfdMaxSpeed + offset / 2);

				}
				else if (controlActionDifference < 0
						&& desiredSpeedPerPump
								<= (DanfossVfdMaxSpeed * 88 / 100))
				{

					// Calculate number of pumps to be activated based on the control action
					activePumps = (controlAction + DanfossVfdMaxSpeed
							- (offset / 2) - 1)
							/ (DanfossVfdMaxSpeed - (offset / 2));

				}
				if (activePumps > systemPumpsNumber) // make sure it doesn't exceed the total number of pumps available
					activePumps = systemPumpsNumber;

				else if (controlActionDifference == 0)
				{
					activePumps = prevActivePumps;
				}

				///**************************start calculate the desired speed per pump*********************************/

				if (controlAction >= offset && activePumps == 0)
				{
					desiredSpeedPerPump = controlAction;
				}
				else if (controlAction < offset)
				{
					desiredSpeedPerPump = 0;
					activePumps = 0;
				}
				else
				{
					desiredSpeedPerPump = controlAction / activePumps;
				}

				///**************************end calculate desired speed per pump*********************************/

				prevActivePumps = activePumps;

				// Calculate desired speed per pump

				if (desiredSpeedPerPump > DanfossVfdMaxSpeed)
					desiredSpeedPerPump = DanfossVfdMaxSpeed;

//			printf("controlActionDifference %d \n", controlActionDifference);
//			printf("controlAction %f \n", controlAction);
//	     	printf("desiredSpeedPerPump %d \n", desiredSpeedPerPump);
//			printf("activePumps%d \n", activePumps);
//			printf("accumlated error%f \n",accumulatedError);
//			printf("\n");

				// Apply the desired speed to the active pumps and ensure others are turned off
				//by sending modbus request to each pump
				for (uint8_t i = 0; i < systemPumpsNumber; i++)
				{

					if (i < activePumps)
					{

						//the control speed is now separated equally among all the pumps

						if (pumpInfoArray[i].isRunning == 0)
						{
							uint16_t tempHolder1 = 0;
							Update_Working_Time(pumpInfoArray[i].id,
									START_COUNTING);

							ModbusRequest(&requestResponseHolder,
									pumpInfoArray[i].id,
									MB_FC_WRITE_HOLDING_REGISTER,
									danfoss101ControlWord, &telegram[1],
									osThreadGetId(), TimeoutValue);

							ModbusRequest(&tempHolder1, pumpInfoArray[i].id,
									MB_FC_READ_HOLDING_REGISTERS,
									danfoss101ControlWord, &telegram[1],
									osThreadGetId(), TimeoutValue);

							if (tempHolder1 == danfoss101_runForward)
							{
								pumpInfoArray[i].isRunning = 1;
								Update_Working_Time(pumpInfoArray[i].id,
										START_COUNTING);

							}
							else
							{
								ModbusRequest(&requestResponseHolder,
										pumpInfoArray[i].id,
										MB_FC_WRITE_HOLDING_REGISTER,
										danfoss101ControlWord, &telegram[1],
										osThreadGetId(), TimeoutValue);

							}

						}

						ModbusRequest(&desiredSpeedPerPump, pumpInfoArray[i].id,
								MB_FC_WRITE_HOLDING_REGISTER, danfoss101Speed,
								&telegram[1], osThreadGetId(), TimeoutValue);

						pumpInfoArray[i].currentSpeed = desiredSpeedPerPump;
//			printf("desiredSpeedPerPump of pump %d %d \n", i,
//					desiredSpeedPerPump);

					}
					else
					{
						// Stop the pump
						if (arrangeChanged == 0)
						{
							ModbusRequest(&desiredSpeedPerPump,
									pumpInfoArray[i].id,
									MB_FC_WRITE_HOLDING_REGISTER,
									danfoss101Speed, &telegram[1],
									osThreadGetId(), TimeoutValue);

							requestResponseHolder1 = danfoss101_stop;
							ModbusRequest(&requestResponseHolder1,
									pumpInfoArray[i].id,
									MB_FC_WRITE_HOLDING_REGISTER,
									danfoss101ControlWord, &telegram[1],
									osThreadGetId(), TimeoutValue);
							//osDelay(200);
							Update_Working_Time(pumpInfoArray[i].id,
									STOP_COUNTING);

							pumpInfoArray[i].currentSpeed = 0;
							pumpInfoArray[i].isRunning = 0;
						}

					}
				}

			}

//if scheduler is enabled and the pumps are working start check for changes on wroking hours of pumps and sort them
			else if (enableScheduler && desiredSpeedPerPump != 0
					&& arrangeChanged)
			{
//			printf("schedulerPumpToRiseIndex %d \n", schedulerPumpToRiseIndex);
//			printf("schedulerPumpToShutDownIndex %d \n",
//					schedulerPumpToShutDownIndex);
//			printf("schedulerPumpToRiseSpeed %d \n", schedulerPumpToRiseSpeed);
//			printf("schedulerPumpToShutDownSpeed %d \n",
//					schedulerPumpToShutDownSpeed);
//			printf("the arrange %d %d %d %d %d %d %d %d \n",
//					pumpInfoArray[0].id, pumpInfoArray[1].id,
//					pumpInfoArray[2].id, pumpInfoArray[3].id,
//					pumpInfoArray[4].id, pumpInfoArray[5].id,
//					pumpInfoArray[6].id, pumpInfoArray[7].id);
//			printf("the running %d %d %d %d %d %d %d %d \n",
//					pumpInfoArray[0].isRunning, pumpInfoArray[1].isRunning,
//					pumpInfoArray[2].isRunning, pumpInfoArray[3].isRunning,
//					pumpInfoArray[4].isRunning, pumpInfoArray[5].isRunning,
//					pumpInfoArray[6].isRunning, pumpInfoArray[7].isRunning);
//			printf("schedulerTargetSpeed %d \n", schedulerTargetSpeed);
//			printf("\n");

				if (pumpInfoArray[schedulerPumpToRiseIndex].isRunning == 0)
				{
					uint16_t tempHolder = 0;
					requestResponseHolder = danfoss101_runForward;

					ModbusRequest(&requestResponseHolder,
							pumpInfoArray[schedulerPumpToRiseIndex].id,
							MB_FC_WRITE_HOLDING_REGISTER, danfoss101ControlWord,
							&telegram[1], osThreadGetId(), TimeoutValue);
					//osDelay(100);

					ModbusRequest(&tempHolder,
							pumpInfoArray[schedulerPumpToRiseIndex].id,
							MB_FC_READ_HOLDING_REGISTERS, danfoss101ControlWord,
							&telegram[1], osThreadGetId(), TimeoutValue);
					//osDelay(100);

					if (tempHolder == danfoss101_runForward)
					{
						pumpInfoArray[schedulerPumpToRiseIndex].isRunning = 1;
						Update_Working_Time(
								pumpInfoArray[schedulerPumpToRiseIndex].id,
								START_COUNTING);

					}
					else
					{
						ModbusRequest(&tempHolder,
								pumpInfoArray[schedulerPumpToRiseIndex].id,
								MB_FC_READ_HOLDING_REGISTERS,
								danfoss101ControlWord, &telegram[1],
								osThreadGetId(), TimeoutValue);
						//osDelay(100);
					}

				}
				else
				{

					ModbusRequest(&desiredSpeedPerPump,
							pumpInfoArray[schedulerPumpToRiseIndex].id,
							MB_FC_WRITE_HOLDING_REGISTER, danfoss101Speed,
							&telegram[1], osThreadGetId(), TimeoutValue);

					//osDelay(100);
					ModbusRequest(&tempPumpToAwakeSpeed,
							pumpInfoArray[schedulerPumpToRiseIndex].id,
							MB_FC_READ_HOLDING_REGISTERS, danfoss101Speed,
							&telegram[1], osThreadGetId(), TimeoutValue);
					if (tempPumpToAwakeSpeed != 0)
					{

						schedulerPumpToShutDownSpeed = 0;
						ModbusRequest(&schedulerPumpToShutDownSpeed,
								pumpInfoArray[schedulerPumpToShutDownIndex].id,
								MB_FC_WRITE_HOLDING_REGISTER, danfoss101Speed,
								&telegram[1], osThreadGetId(), TimeoutValue);
						arrangeChanged = 0;
						pumpInfoArray[schedulerPumpToShutDownIndex].isRunning =
								0;
						pumpInfoArray[schedulerPumpToShutDownIndex].currentSpeed =
								0;
						schedulerPumpToShutDownIndex = 20;
						schedulerPumpToRiseIndex = 20;
						Update_Working_Time(
								pumpInfoArray[schedulerPumpToShutDownIndex].id,
								STOP_COUNTING);

						//osDelay(300);

					}

					else
					{
						ModbusRequest(&desiredSpeedPerPump,
								pumpInfoArray[schedulerPumpToRiseIndex].id,
								MB_FC_WRITE_HOLDING_REGISTER, danfoss101Speed,
								&telegram[1], osThreadGetId(), TimeoutValue);
					}
				}
			}

			else
			{
				arrangeChanged = 0;
			}
		}

		// if the user on manual mode start sending the changes he make on the app to the vfd through modbus requests
		else if (operationMode == MANUAL_MODE)
		{
			if (manualPumpID != 0)
			{
				if (manualPumpSpeed > 0)
				{
					requestResponseHolder = danfoss101_runForward;
					ModbusRequest(&requestResponseHolder, manualPumpID,
							MB_FC_WRITE_HOLDING_REGISTER, danfoss101ControlWord,
							&telegram[1], osThreadGetId(), TimeoutValue);
					ModbusRequest(&manualPumpSpeed, manualPumpID,
							MB_FC_WRITE_HOLDING_REGISTER, danfoss101Speed,
							&telegram[1], osThreadGetId(), TimeoutValue);

				}
				else if (manualPumpSpeed == 0)
				{
					requestResponseHolder = danfoss101_stop;
					ModbusRequest(&requestResponseHolder, manualPumpID,
							MB_FC_WRITE_HOLDING_REGISTER, danfoss101ControlWord,
							&telegram[1], osThreadGetId(), TimeoutValue);
					ModbusRequest(&manualPumpSpeed, manualPumpID,
							MB_FC_WRITE_HOLDING_REGISTER, danfoss101Speed,
							&telegram[1], osThreadGetId(), TimeoutValue);
				}
				manualPumpSpeed = 0;
				manualPumpID = 0;
			}
		}
			// get the readings of the speed and the current from the vfd of each pump
			for (int i = 0; i < systemPumpsNumber; i++)
			{
				ModbusRequest(&speedPercentageOfPump[i], i + 1,
						MB_FC_READ_HOLDING_REGISTERS, danfoss101Speed,
						&telegram[1], osThreadGetId(), TimeoutValue);
//				osDelay(100);
				//	printf("read speed  of pump %d %d \n",i,speedPercentageOfPump[i]);

			}

			// get the readings of the speed and the current from the vfd of each pump
			for (int i = 0; i < systemPumpsNumber; i++)
			{
				ModbusRequest(&pumpCurrent[i], i + 1,
						MB_FC_READ_HOLDING_REGISTERS, danfoss101Current_MA_Read
						, &telegram[1], osThreadGetId(), TimeoutValue);
//				osDelay(100);

			}

			if (save == 1)
			{
				store_factory_page_data();
				save = 0;
			}
			osDelay(100);
		}

		/* USER CODE END PIDControl */
	}
#elif defined(SINGLE_VFD)

/* USER CODE BEGIN Header_PIDControl */
/**
 * @brief Implements the PIDControl thread for a single VFD system, specifically designed to control a base load pump
 *        and other directly connected pumps. The task manages the PID control for the base load VFD pump and
 *        directly controls the on/off state of other pumps. It ensures optimal operation by adjusting the base load pump
 *        speed while running other pumps at full efficiency to achieve the desired setpoint. The task complies with
 *        MISRA C:2012 guidelines, ensuring reliability and safety in an embedded control system context.
 *
 *        Key Operations:
 *        - PID control of the base load pump VFD for precise speed adjustment.
 *        - Direct control of other pumps for full efficiency operation.
 *        - Monitoring and updating of system parameters like pressure and pump status.
 *        - Management of pump states based on process requirements and control algorithm.
 *
 *        This function is crucial in systems with a single VFD and multiple direct online pumps, commonly used in
 *        industrial and process control applications for efficient and synchronized operation.
 *
 * @param argument: Not used, included for RTOS task compatibility.
 * @retval None
 */
/* USER CODE END Header_PIDControl */
void PIDControl(void *argument) {

	/* USER CODE BEGIN PIDControl */

	uint8_t startPumpsFlag = 0;
	uint16_t baseLoadPumpSpeed = 0;
	telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
	telegram[1].u8id = 1;
	//delete_internal_system_data_from_backup_domain_registers();
	OldKI = Ki;
	/* Infinite loop */
	for (;;) {

		if (arrangeChanged == 0 && enablePID == 1 ) {
			//below cutin phase stop start the pid and pumps
			if (currentPressure <= (setPointRequired - cutIn)
					&& startPumpsFlag == 0 && activePumps == 0) {
				startPumpsFlag = 1;
			}
			//above cutIn phase stop the pumps and pid
			else if (currentPressure >= (setPointRequired + cutOff)
					&& startPumpsFlag == 1) {
				startPumpsFlag = 0;
				accumulatedError = 0;
			}
//calculation of pid parameters
			float error = setPointRequired - currentPressure;

			float Pout = Kp * error;
			if (KIreceived == 1) {
				// Scale the accumulated error
				if (OldKI != 0) {
					accumulatedError *= (OldKI / Ki);
				} else {
					accumulatedError = 0;
				}
				// Reset the flag and update oldKi
				KIreceived = 0;
				OldKI = Ki;
			}
			accumulatedError += error * 200; // deltaTime is the time step, e.g., 30ms

			float Iout = Ki * accumulatedError;

			// Derivative term calculation
			float deltaError = error - lastError;

			float Dout = Kd * deltaError; // deltaTime is the time step, e.g., 30ms

			if (startPumpsFlag == 1) {
				controlAction = Pout + Iout + Dout;
				//make sure pump start at 50%
				if (controlAction < DanfossVfdMaxSpeed * .3 && controlAction > 0) {
					controlAction = DanfossVfdMaxSpeed * .3;
				}
			} else {
				controlAction = 0;
			}

//			printf("delta error %f \n",deltaError);
//			printf("dout %f \n",Dout);
//			printf("error %d \n ",error);
//			printf("last error %f \n",lastError);
//			printf("control Action %f \n",controlAction);
//			printf("\n");

			lastError = error; // Save the current error for next loop iteration
			//	printf("pout %f \n",Pout);

			if (accumulatedError < 0) {
				accumulatedError = 0;

			}
			if (controlAction <= 0) {
				controlAction = 0;
				startPumpsFlag = 0;
				accumulatedError = 0;
			}

			if (controlAction > DanfossVfdMaxSpeed * systemPumpsNumber) {
				controlAction = DanfossVfdMaxSpeed * systemPumpsNumber;
				accumulatedError -= error * 200;
			}
			//store the current pump speed and state so if the system restarts it continue from the point it stopped at .
			store_internal_system__data();

			//measure the change from the PID output from the previous iteration.
			controlActionDifference = controlAction - prevControlAction;

			prevControlAction = controlAction;

			//prevent hysteresis logic
			if (controlActionDifference > 0) {
				// Calculate number of pumps to be activated based on the new control action
				activePumps = (controlAction + DanfossVfdMaxSpeed + offset / 2
						- 1) / (DanfossVfdMaxSpeed + offset / 2);

			} else if (controlActionDifference < 0) {

				// Calculate number of pumps to be activated based on the control action
				activePumps = (controlAction + DanfossVfdMaxSpeed - (offset / 2)
						- 1) / (DanfossVfdMaxSpeed - (offset / 2));

			}

			else if (controlActionDifference == 0) {
				activePumps = prevActivePumps;
			}

			///**************************start calculate the speed of base load pump*********************************/


				baseLoadPumpSpeed = (uint32_t)controlAction % (DanfossVfdMaxSpeed+1);
				if (baseLoadPumpSpeed < (DanfossVfdMaxSpeed / 3)&&controlAction>0) {
					baseLoadPumpSpeed = DanfossVfdMaxSpeed / 3;

	}
				else if (controlAction==0)
				{
					baseLoadPumpSpeed=0;
				}

			///**************************end calculate the speed of base load pump*********************************/

			prevActivePumps = activePumps;

			if (activePumps > systemPumpsNumber) // make sure it doesn't exceed the total number of pumps available
				activePumps = systemPumpsNumber;

			printf("controlActionDifference %d \n", controlActionDifference);
			printf("controlAction %f \n", controlAction);
	     	printf("baseLoadPumpSpeed %d \n", baseLoadPumpSpeed);
			printf("activePumps%d \n", activePumps);
				printf("accumlated error%f \n",accumulatedError);
				printf("setPoint: %.3f\n", setPointRequired);
				  printf("Current Pressure = %.8f\n", currentPressure);
			printf("\n");

			// Apply the desired speed to the active pumps and ensure others are turned off
			for (uint8_t i = 0; i < systemPumpsNumber; i++) {

				if (i < activePumps) {

					//if base load pump which lay in the start of the pump info array
					if (i == 0) {
						// if the base load pump is not running start it
						if (pumpInfoArray[i].isRunning == 0) {
							uint16_t tempHolder1 = 0;

							Update_Working_Time(pumpInfoArray[i].id,
									START_COUNTING);
							//send start order to the pump three times to make sure it is started
							requestResponseHolder = danfoss101_runForward;
							ModbusRequest(&requestResponseHolder,
									pumpInfoArray[i].id,
									MB_FC_WRITE_HOLDING_REGISTER,
									danfoss101ControlWord, &telegram[1], osThreadGetId(), TimeoutValue);
							//osDelay(100);
							ModbusRequest(&requestResponseHolder,
									pumpInfoArray[i].id,
									MB_FC_WRITE_HOLDING_REGISTER,
									danfoss101ControlWord, &telegram[1], osThreadGetId(), TimeoutValue);
							//osDelay(100);
							ModbusRequest(&requestResponseHolder,
									pumpInfoArray[i].id,
									MB_FC_WRITE_HOLDING_REGISTER,
									danfoss101ControlWord, &telegram[1], osThreadGetId(), TimeoutValue);
							//osDelay(100);
							// read the pump state to check if it is started
							ModbusRequest(&tempHolder1, pumpInfoArray[i].id,
									MB_FC_READ_HOLDING_REGISTERS,
									danfoss101ControlWord, &telegram[1], osThreadGetId(), TimeoutValue);
							//osDelay(200);
							//if the pump started successfully
							if (1) {
								pumpInfoArray[i].isRunning = 1;
								Update_Working_Time(pumpInfoArray[i].id,
										START_COUNTING);
								ModbusRequest(&baseLoadPumpSpeed,
										pumpInfoArray[i].id,
										MB_FC_WRITE_HOLDING_REGISTER,
										danfoss101Speed, &telegram[1], osThreadGetId(), TimeoutValue);
								baseLoadPumpWokring=1;


							} else {
								//send the request again
								ModbusRequest(&requestResponseHolder,
										pumpInfoArray[i].id,
										MB_FC_WRITE_HOLDING_REGISTER,
										danfoss101ControlWord, &telegram[1], osThreadGetId(), TimeoutValue);
								//osDelay(100);
							}

						} else {
							// if the base load pump is already running send the desired speed to it
							ModbusRequest(&baseLoadPumpSpeed,
									pumpInfoArray[i].id,
									MB_FC_WRITE_HOLDING_REGISTER,
									danfoss101Speed, &telegram[1], osThreadGetId(), TimeoutValue);
							pumpInfoArray[i].currentSpeed = baseLoadPumpSpeed;
						}



					} else
						//if the pump should run and it is one of the direct run pumps
						//make it run at 100% speed by writing high on the pid which it is connected to
						{

						if (pumpInfoArray[i].isRunning == 0) {
//							HAL_GPIO_WritePin(pumpInfoArray[i].port,
//									pumpInfoArray[i].pinNumber, GPIO_PIN_SET);
							pumpInfoArray[i].isRunning = 1;
							Update_Working_Time(pumpInfoArray[i].id,
									START_COUNTING);
							pumpInfoArray[i].currentSpeed = 100;
						}
					}

			}
				//if it is one of stop pumps stop by by writing low on the pind that this pump is connected to
				else{
					if (pumpInfoArray[i].isRunning == 1) {
//						HAL_GPIO_WritePin(pumpInfoArray[i].port,
//								pumpInfoArray[i].pinNumber, GPIO_PIN_RESET);
						pumpInfoArray[i].isRunning = 0;
						Update_Working_Time(pumpInfoArray[i].id,
								STOP_COUNTING);
						pumpInfoArray[i].currentSpeed = 0;
					}
				}

			}

			//if the PID should stop the base load pump .
				if(activePumps==0){
					if (pumpInfoArray[0].isRunning == 1) {
											uint16_t tempHolder1 = 0;


											//send start order to the pump three times to make sure it is started
											requestResponseHolder = danfoss101_stop;
											ModbusRequest(&requestResponseHolder,
													pumpInfoArray[0].id,
													MB_FC_WRITE_HOLDING_REGISTER,
													danfoss101ControlWord, &telegram[1], osThreadGetId(), TimeoutValue);
											//osDelay(100);
											ModbusRequest(&requestResponseHolder,
													pumpInfoArray[0].id,
													MB_FC_WRITE_HOLDING_REGISTER,
													danfoss101ControlWord, &telegram[1], osThreadGetId(), TimeoutValue);
											//osDelay(100);
											ModbusRequest(&requestResponseHolder,
													pumpInfoArray[0].id,
													MB_FC_WRITE_HOLDING_REGISTER,
													danfoss101ControlWord, &telegram[1], osThreadGetId(), TimeoutValue);
											//osDelay(100);
											// read the pump state to check if it is started
											ModbusRequest(&tempHolder1, pumpInfoArray[0].id,
													MB_FC_READ_HOLDING_REGISTERS,
													danfoss101ControlWord, &telegram[1], osThreadGetId(), TimeoutValue);
											//osDelay(200);
											//if the pump started successfully
											if (1) {
												pumpInfoArray[0].isRunning = 0;
												Update_Working_Time(pumpInfoArray[0].id,
														STOP_COUNTING);
												ModbusRequest(&baseLoadPumpSpeed,
														pumpInfoArray[0].id,
														MB_FC_WRITE_HOLDING_REGISTER,
														danfoss101Speed, &telegram[1], osThreadGetId(), TimeoutValue);
												pumpInfoArray[0].isRunning = 0;
												pumpInfoArray[0].currentSpeed=0;
												baseLoadPumpWokring=0;
											} else {
												//send the request again
												ModbusRequest(&requestResponseHolder,
														pumpInfoArray[0].id,
														MB_FC_WRITE_HOLDING_REGISTER,
														danfoss101ControlWord, &telegram[1], osThreadGetId(), TimeoutValue);
												//osDelay(100);
											}


										}
				}
		}
//if scheduler is enabled and there is some pumps working and the arrange of pumps changed .
			else if (enableScheduler && desiredSpeedPerPump != 0
					&& arrangeChanged) {
//			printf("schedulerPumpToRiseIndex %d \n", schedulerPumpToRiseIndex);
//			printf("schedulerPumpToShutDownIndex %d \n",
//					schedulerPumpToShutDownIndex);
//			printf("schedulerPumpToRiseSpeed %d \n", schedulerPumpToRiseSpeed);
//			printf("schedulerPumpToShutDownSpeed %d \n",
//					schedulerPumpToShutDownSpeed);
//			printf("the arrange %d %d %d %d %d %d %d %d \n",
//					pumpInfoArray[0].id, pumpInfoArray[1].id,
//					pumpInfoArray[2].id, pumpInfoArray[3].id,
//					pumpInfoArray[4].id, pumpInfoArray[5].id,
//					pumpInfoArray[6].id, pumpInfoArray[7].id);
//			printf("the running %d %d %d %d %d %d %d %d \n",
//					pumpInfoArray[0].isRunning, pumpInfoArray[1].isRunning,
//					pumpInfoArray[2].isRunning, pumpInfoArray[3].isRunning,
//					pumpInfoArray[4].isRunning, pumpInfoArray[5].isRunning,
//					pumpInfoArray[6].isRunning, pumpInfoArray[7].isRunning);
//			printf("schedulerTargetSpeed %d \n", schedulerTargetSpeed);
//			printf("\n");
				//wake the new pump that with lowest working hours
				if (pumpInfoArray[schedulerPumpToRiseIndex].isRunning == 0) {
//						HAL_GPIO_WritePin(pumpInfoArray[schedulerPumpToRiseIndex].port,
//								pumpInfoArray[schedulerPumpToRiseIndex].pinNumber, GPIO_PIN_SET);
						pumpInfoArray[schedulerPumpToRiseIndex].isRunning = 1;
						Update_Working_Time(pumpInfoArray[schedulerPumpToRiseIndex].id,
								START_COUNTING);
						pumpInfoArray[schedulerPumpToRiseIndex].currentSpeed = 100;

				}
				if (	pumpInfoArray[schedulerPumpToShutDownIndex].isRunning==1)
				{
//					HAL_GPIO_WritePin(pumpInfoArray[schedulerPumpToShutDownIndex].port,
//							pumpInfoArray[schedulerPumpToShutDownIndex].pinNumber, GPIO_PIN_RESET);
					pumpInfoArray[schedulerPumpToShutDownIndex].isRunning = 0;
					Update_Working_Time(pumpInfoArray[schedulerPumpToShutDownIndex].id,
							STOP_COUNTING);
					pumpInfoArray[schedulerPumpToShutDownIndex].currentSpeed = 0;

				}
			} else {
				arrangeChanged = 0;
			}

			if (save == 1) {
				//if the user requested to save the data permanently
				store_factory_page_data();
				save = 0;
			}
			osDelay(100);
		}

		/* USER CODE END PIDControl */

	}


#elif defined(NO_VFD)
/**
 * @brief Implements the PIDControl thread for a direct online pump system. In this configuration,
 *        the system is connected directly to multiple pumps, each activated at specific pressure set points.
 *        The task manages the on/off state of each pump based on current system pressure and predefined set points,
 *        ensuring efficient operation and maintaining the desired pressure. The task adheres to MISRA C:2012 guidelines,
 *        promoting safety and reliability in an embedded system designed for process control.
 *
 *        Key Operations:
 *        - Managing on/off state of pumps based on pressure set points.
 *        - Monitoring current system pressure and adjusting pump states accordingly.
 *        - Ensuring pumps with the lowest working hours are prioritized for activation.
 *        - Maintaining efficient operation while achieving pressure control objectives.
 *
 *        The function is crucial in systems where direct online pump control is preferred for simplicity and efficiency,
 *        commonly used in applications requiring straightforward pressure management with multiple pumps.
 *
 * @param argument: Not used, included for RTOS task compatibility.
 * @retval None
 */
void PIDControl(void *argument)
{
	// set each set point that will make addtional pump start
	pumpsSetPoints[0]=8;
	pumpsSetPoints[1]=5.5;
	pumpsSetPoints[2]=3.5;
	pumpsSetPoints[3]=1.5;

/* USER CODE BEGIN PIDControl */
for(;;) {
	float error = setPointRequired - currentPressure;
	printf("error %f\n",error);
	printf("\n");
	for(uint8_t i=0;i<systemPumpsNumber;i++) {
		if((pumpsSetPoints[i]+error)>setPointRequired)
		{
			if(pumpInfoArray[i].isRunning==0)
			{
			Update_Working_Time(
					pumpInfoArray[i].id,
					START_COUNTING);
//			HAL_GPIO_WritePin(pumpInfoArray[i].port, pumpInfoArray[i].pinNumber, GPIO_PIN_SET);
			pumpInfoArray[i].isRunning=1;
		}
		}
		else
		{
			if(pumpInfoArray[i].isRunning==1)
			{
			// Set the first pump's pin low
//			HAL_GPIO_WritePin(pumpInfoArray[i].port, pumpInfoArray[i].pinNumber, GPIO_PIN_RESET);
			Update_Working_Time(
					pumpInfoArray[i].id,
					STOP_COUNTING);
			pumpInfoArray[i].isRunning=0;
			}
		}
	}
	osDelay(1000);
}
/* USER CODE END PIDControl */
}
#endif

/**
 * @brief Implements the Pumps_Scheduler thread for dynamically managing pump operation in a system with multiple pumps.
 *        This task continuously evaluates the working hours of each pump and rearranges their operation based on
 *        accumulated working time to balance usage. It activates pumps with lower working hours while deactivating
 *        those with higher usage, adhering to MISRA C:2012 guidelines for safety and reliability in embedded control systems.
 *
 *        Key Operations:
 *        - Monitoring and updating working time for each pump.
 *        - Sorting pumps based on working hours for balanced usage.
 *        - Dynamically controlling pump operation based on scheduler settings and working time.
 *        - Maintaining system efficiency and extending pump life through balanced operation.
 *
 *        This task is crucial in systems where multiple pumps are used and wear balancing is needed to ensure
 *        longevity and efficiency, commonly seen in industrial and environmental control applications.
 *
 * @param argument: Not used, included for RTOS task compatibility.
 * @retval None
 */
	/* USER CODE END Header_Pumps_Scheduler */
	void Pumps_Scheduler(void *argument)
	{
		/* USER CODE BEGIN Pumps_Scheduler */

		/* Infinite loop */
		for (;;)
		{

// Get current time (if needed)

			osDelay(60000);
			//if the scheduler of the app is enabled and there is no scheduler record
			// match the current date and time stop the pid and the pumps
			if (matchedRecordCount == 0 && matchedWeeklyRecordCount == 0
					&& date_week_sched_disabled == 0)
			{
				enablePID = 0;
				schedDisabledSystem = 1;
			}
			else
			{
				enablePID = 1;
				schedDisabledSystem = 0;

			}

			sendRTC = 1;
//store the values of the pumpinfoArray in the temp array to compare with it later
			for (int i = 0; i < systemPumpsNumber; i++)
			{
				tempPumpInfoArray[i].id = pumpInfoArray[i].id;
			}
			// get current date and time to start working with it
			Get_RTC_Values();
			//update the working time of each pump
			for (uint8_t i = 1; i <= systemPumpsNumber; i++)
			{
				Update_Working_Time(i, UPDATE_TIME);
			}
//sort the pumps according ascending according to the working time

			if (!arrangeChanged && enableScheduler)
			{

				Sort_Pumps_By_changeOverTime(pumpInfoArray, systemPumpsNumber,
						changeOverTime);
				write_pump_data_to_backup_registers();
			}

//check for change in the order of the pumps
			for (int i = 0; i < systemPumpsNumber; i++)
			{
				//if there is a change in the order of the working pumps
				if (tempPumpInfoArray[i].id != pumpInfoArray[i].id)

				{
					//sotre the new sorted array
					for (int j = 0; j < systemPumpsNumber; j++)
					{
						tempPumpInfoArray[j].id = pumpInfoArray[j].id;
					}
					//get the pump that should work
					for (int j = 0; j < systemPumpsNumber; j++)
					{
						if (pumpInfoArray[j].isRunning == 0)
						{
							schedulerPumpToRiseIndex = j;
							arrangeChanged = 1;
							break;
						}
					}

					if (arrangeChanged)
					{
						//get the pump that should shutdown
						for (int j = systemPumpsNumber - 1; j >= 0; j--)
						{
							if (pumpInfoArray[j].isRunning == 1)
							{
								schedulerPumpToShutDownIndex = j;
								break;
							}
						}
					}

					break;
				}
			}
		}
		/* USER CODE END Pumps_Scheduler */
	}

	/* Private application code --------------------------------------------------*/
	/* USER CODE BEGIN Application */

	/* USER CODE END Application */

