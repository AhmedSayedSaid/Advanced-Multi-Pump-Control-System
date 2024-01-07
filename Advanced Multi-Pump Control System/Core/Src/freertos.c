/***************************************************************************************
 * NOTE TO THE READER:
 ***************************************************************************************
 * This file contains only the headers and documentation for various functions.
 * The implementation of these functions has been deliberately removed in order to
 * protect the copyright and prevent unauthorized copying of the code.
 *
 * Each function is accompanied by a brief description, including details about
 * its intended purpose, parameters, and expected return values. However, the
 * actual code within each function is intentionally omitted.
 *
 * This approach is taken to share the structure and design of the code while
 * preserving the integrity and proprietary nature of the actual implementation.
 ***************************************************************************************/

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
// #define SINGLE_VFD
// #define NO_VFD

#if defined(SINGLE_VFD)
uint8_t baseLoadPumpWokring = 0;
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
// to show weither it we are on manual mode
// or PID when entering the home page to show the appropriate settings
uint8_t sendOperationMode = 0;

// the pump id and speed that the user changed from the APP when
// the app is on the manual mode
uint8_t manualPumpID = 0;
uint16_t manualPumpSpeed = 0;

// current operaton mode
uint8_t operationMode = MANUAL_MODE;
/*flags to erase sections of the external flash while the code running by me
 * by editing them while debugging*/
volatile uint8_t eraseEntireChipFlag = 0;
volatile uint8_t eraseDatedRecordsFlag = 0;
volatile uint8_t eraseWeeklyRecordsFlag = 0;
volatile uint8_t eraseAlarmsRecordsFlag = 0;

// Flag when the user enter the alarm page on the app this will be 1 (alarms page)
uint8_t sendAlarmRecords = 0;
// Flag when the user enter the dated scheduler  page on the app this will be 1 (Scheduler page)
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
// start timer variables
uint8_t KIreceived = 0;
// end timer variables
// flag to stop the pid if no single vfd response at all
volatile uint8_t fullSystemTrip = 0;
// number of working currently working pumps on the system
uint8_t activePumps = 0;
// helpers to pid algortim variables
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
// rgb led variables
uint16_t redValue = 64000;
uint16_t greenValue = 32000;
uint16_t blueValue = 32000;

//(global between task)
// holder of modbus request
uint16_t requestResponseHolder1 = danfoss101_stop;
uint16_t requestResponseHolder = 0;
// current speed of each pump on the system
uint16_t speedPercentageOfPump[8] =
	{0, 0, 0, 0, 0, 0, 0, 0};
// current pump current of each pump on the system
uint16_t pumpCurrent[8] =
	{0, 0, 0, 0, 0, 0, 0, 0};
// flag to indicate weither we are on home page or not to start or stop sending the pid data
uint8_t inHomePage = 0;
// flag to check if there is dry run condition to stop the pumps in this case
uint8_t dryRunDetected = 0;
// alarm array which contain all the alarms of the system
extern char alarmsArray[MAX_ALARMS][RECORD_SIZE];
uint16_t DanfossVfdMaxSpeed = 16384;
// if we enter the scheduler page this will be one to send the rtc date and time
uint8_t setRTC = 0;
//  hold current info about each pump on the array
PumpInfo pumpInfoArray[MAX_NUM_PUMPS];
// time out for modebus request if not response on this time (modbus configuration)
uint32_t TimeoutValue = 65; // minimum for a successful communication is 35ms
// test the case of connectvity with the vfd and this is every around 5 seconds
uint8_t testIFNoVfdResponse = 1;
// if the pressure sensor error is not connected probelry this flag will be 1
uint8_t pressureSensorError = 0;
// handler of  configurations of modbus
modbus_t telegram[2];

//(Factory Configuration page)
// used to to maintain the current value of pid ouput when hte user change the ki constant
float OldKI = 0;
float setPointRequired = 5;
uint8_t getFactoryData = 0;
// pack those three varibles on 4 bytes of the back domain
uint16_t offset = 1800;
float cutIn = 1.5;
float cutOff = 1.5;
uint8_t VFD_type = 0;
uint8_t systemPumpsNumber = 4;
// pack those two variables on one 32bit register of the backup domain
uint16_t firstADCCalibrationReading = 7743;
uint16_t secondADCCalibrationReading = 64196;
char RTCString[25]; // Replace this line with the actual received string
float Kp = 1000;
float Ki = .2;
float Kd = 0;
float currentOfTheFirstReading = 4.00000192;
float currentOfTheSecondReading = 20.5100198;
float_t minimumPressure = 0;
float MaximumPressure = 16;
uint16_t changeOverTime = 20;
// the flags try pack them in one byte
uint8_t enableScheduler = 0;
uint8_t enablePID = 0;
uint8_t save = 0;
uint8_t sendRTC = 1;
//	first current received from the setting page to calibrate the adc (Factory Configuration page)
uint8_t firstCurrentReadingRecived = 0;
//	second current received from the setting page to calibrate the adc
uint8_t secondCurrentReadingRecived = 0;
float coffA = 0.00029245f;	// Calibration multiplier
float coffB = 1.735515392f; // Calibration offset
char vfdType[101] =
	{0};
// enable dry run switch
uint8_t enableDryRun = 0;

//(no vfd pid)
// used to hold each set point the system will start a new pump on it
float pumpsSetPoints[MAX_NUM_PUMPS];

//(Pumps_Scheduler task)
PumpInfo tempPumpInfoArray[MAX_NUM_PUMPS];
// if one of hte pumps excedded change over time it this flag will be set to one to make the
// scheduler task stop it and start one with less wokring hours
volatile uint8_t arrangeChanged = 0;

// modbus handlers and buffers and in our application it it allowed to use two instance of modbus communication
extern modbusHandler_t ModbusH;
extern uint16_t ModbusDATA[128];
extern modbusHandler_t ModbusH2;
extern uint16_t ModbusDATA2[128];

// varible that the rtc will fill with the current date and time
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

// if the user in the app enter the alarms page this flag will be 1
uint8_t sendAlarms = 0;
// current presusre that is being get from the sensor connected to the board
float currentPressure;
// end ADC TASK VARibles

//(alarm and warning page)
// the user choosed from the alarms page that if there is a dry run it will trip the system
uint8_t dryRunSysStop = 0;
// the user enabled the pressure sensor not detected erro
uint8_t pressureAlarmEnable = 0;
// the user choose from alarm page that the pressure sensor not detected will trip the system
uint8_t pressureSysStop = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes =
	{
		.name = "defaultTask",
		.stack_size = 256 * 4,
		.priority =
			(osPriority_t)osPriorityNormal,
};
/* Definitions for UART1Task */
osThreadId_t UART1TaskHandle;
const osThreadAttr_t UART1Task_attributes =
	{
		.name = "UART1Task",
		.stack_size = 512 * 4,
		.priority =
			(osPriority_t)osPriorityLow,
};
/* Definitions for Task4_20mAscan */
osThreadId_t Task4_20mAscanHandle;
const osThreadAttr_t Task4_20mAscan_attributes =
	{
		.name = "Task4_20mAscan",
		.stack_size = 256 * 4,
		.priority =
			(osPriority_t)osPriorityLow,
};
/* Definitions for myTaskMaster */
osThreadId_t myTaskMasterHandle;
const osThreadAttr_t myTaskMaster_attributes =
	{
		.name = "myTaskMaster",
		.stack_size = 256 * 4,
		.priority =
			(osPriority_t)osPriorityLow,
};
/* Definitions for ExternalFlash */
osThreadId_t ExternalFlashHandle;
const osThreadAttr_t ExternalFlash_attributes =
	{
		.name = "ExternalFlash",
		.stack_size = 512 * 4,
		.priority =
			(osPriority_t)osPriorityLow,
};
/* Definitions for UARTSend */
osThreadId_t UARTSendHandle;
const osThreadAttr_t UARTSend_attributes =
	{
		.name = "UARTSend",
		.stack_size = 512 * 4,
		.priority =
			(osPriority_t)osPriorityLow,
};
/* Definitions for PID */
osThreadId_t PIDHandle;
const osThreadAttr_t PID_attributes =
	{
		.name = "PID",
		.stack_size = 512 * 4,
		.priority =
			(osPriority_t)osPriorityLow,
};
/* Definitions for PumpsScheduler */
osThreadId_t PumpsSchedulerHandle;
const osThreadAttr_t PumpsScheduler_attributes =
	{
		.name = "PumpsScheduler",
		.stack_size = 256 * 4,
		.priority =
			(osPriority_t)osPriorityLow,
};

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
}

/**
 * @brief UART1 Task Function for FreeRTOS
 *
 * Overview:
 * Function Name: UART1_task
 * Purpose: The `UART1_task` function is designed as a FreeRTOS task, responsible for managing UART (Universal Asynchronous Receiver-Transmitter) communications in an embedded system environment. This task handles communications through UART1, particularly focusing on interactions with external devices such as an ESP32 module, and processes data for the main board microcontroller, STM32H750.
 *
 * Key Functionalities:
 * 1. UART Data Reception:
 *    - Initiates UART reception in interrupt mode to efficiently handle asynchronous data reception.
 *    - Continuously monitors for new data received from the connected ESP32 device.
 *
 * 2. Data Parsing and Processing:
 *    - Upon new data arrival, parses the data to extract relevant commands or settings.
 *    - Converts received string data into appropriate data formats (integers, floats) as required by the system.
 *
 * 3. System Parameter Updates:
 *    - Updates various system parameters based on the received data, including:
 *      a. RGB LED color intensity adjustments.
 *      b. System setpoints such as pressure thresholds.
 *      c. Operational modes and control of pump speeds.
 *      d. Real-time clock (RTC) adjustments.
 *      e. Scheduler and alarm configuration settings.
 *
 * @param argument A pointer to the argument passed to the function.
 *                 This parameter is not used in the current implementation.
 *
 * @remarks The function is structured to comply with embedded system standards and is designed to be robust and efficient in handling UART communications. It is part of a larger system that relies on accurate and timely data communication for operational effectiveness.
 *
 * @warning This function is intended to run as a FreeRTOS task. It contains an infinite loop and must be carefully managed to avoid any potential system blocking or deadlocks in a real-time operating system (RTOS) environment.
 */
void UART1_task(void *argument)
{
}

/* USER CODE BEGIN Header_external_flash */
/**
 * @brief this task is responsable for operation of the external flash on the board
 * 			like storing alarms ,date of the operation of the system on the correct section
 * 			of the external flash and responsable for deleting the data when the user of the app request that .

 *
 * @param argument: Not used, included for RTOS task compatibility.
 * @retval None
 */
/* USER CODE END Header_external_flash */
void external_flash(void *argument)
{

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

	/* USER CODE END UART_Send */
}

/* USER CODE BEGIN Header_PIDControl */
/**
 * @brief Function implementing the PID control which send the approprate orders to the vfds using modbus communcation protocol to maintian the required set point .
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PIDControl */
#if defined(MULTI_VFD)
void PIDControl(void *argument)
{
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
void PIDControl(void *argument)
{
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
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
