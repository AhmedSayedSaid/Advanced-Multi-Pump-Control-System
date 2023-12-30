/*
 * external_flash.h
 *
 *  Created on: Nov 6, 2023
 *      Author: ahmed sayed
 */

#ifndef INC_EXTERNAL_FLASH_H_
#define INC_EXTERNAL_FLASH_H_

#include "rtc.h"
#include "backup_resgisters_operations.h"
#include "s25fl256s.h"
#include "quadspi.h"
#include "rtc.h"
#include <string.h>
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"


// Constants and macros:
#define DATED_SCHEDULAR_RECORDS_START_ADDRESS 15000000
#define WEEKLY_SCHEDULAR_RECORDS_START_ADDRESS 20000000
#define ALARMS_START_ADDRESS 25000000

#define MAX_RECORDS 1000
#define MAX_DAYS 8
#define DATED_RECORD_SIZE 48
#define WEEKLY_RECORD_SIZE 32
#define MAX_RECORD_LENGTH 100
#define MAX_VALID_RECORDS 1000
#define MAX_VALID_WEEKLY_RECORDS 1000
#define RECORD_BUFFER_SIZE 10
#define MAX_RETRIES 3
#define ALARM_RECORD_SIZE 32
#define RECORD_SIZE 32
#define MAX_ALARMS 100


// varible which is used to help the store and restore from the external flash memory
extern uint8_t datedSchedRegisterNumber;
extern uint8_t numberOfWeeklyRecordsRegNum;
extern char records[1000][48];
extern uint32_t schedulerRecordsStartAddress;
extern uint32_t schedulerRecordsCurrentAddress;
extern uint8_t eraseExternalFlashFlag;
extern uint8_t stopTimeFlag;
extern volatile uint8_t removeRecordFlag;
extern volatile uint8_t removeWeeklyRecordFlag;

extern int matchedRecordCount;
extern int matchedWeeklyRecordCount;
extern int valid_record_count;
extern uint32_t valid_weekly_record_count;
extern int matchingRecordIndex;

extern int matchedRecordIndices[MAX_VALID_RECORDS];
extern int matchedWeeklyRecordIndices[MAX_VALID_WEEKLY_RECORDS];

extern int32_t flStatus;
extern uint8_t firstTimeRequest;
extern uint8_t date_week_sched_disabled;
extern uint8_t schedDisabledSystem;
extern char startTime[30];
extern char stopTime[30];
extern char fullDatedSchedRecord[48];
extern char weeklyRecord[30];
extern uint8_t weeklyRecordFlag;
extern uint16_t indexOfRecordToRemove;
extern char stopTimeRemove[30];
extern uint8_t addschedData;
extern uint8_t removeschedData;

extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern uint32_t numberOfSchedulerRecords;
extern uint32_t numberOfWeeklyRecords;
extern uint8_t numberOfAlarmsRegNum;


extern uint32_t numberOfSchedulerRecords;
extern uint32_t numberOfWeeklyRecords;
extern uint32_t storedAlarmsNumber;


// Type definitions:
typedef struct
{
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} STM32_RTC;

typedef struct
{
	int index;
	STM32_RTC start_time;
	STM32_RTC end_time;
	char original_record[MAX_RECORD_LENGTH];
} Record;


//weekly record struct
typedef struct
{
	int index;
	STM32_RTC start_time;
	STM32_RTC end_time;
	char original_record[MAX_RECORD_LENGTH];
	uint8_t days[MAX_DAYS];
} weekRecord;


//types of alarms exsisted on the system
typedef enum{
	dryRun=0,
	noPressureSensor,
}alarmType;

extern Record valid_records[MAX_RECORDS];
extern weekRecord valid_weekly_records[MAX_RECORDS];

// Function Prototypes

/**
 * @brief Stores an alarm record on the external flash memory.
 *
 * Attempts to write the alarm data to the external flash memory up to three times.
 * If a write attempt fails, it marks the record as deleted and tries to write the record again at the next address.
 *
 * @param alarm The type of alarm to store.
 * @param baseAddress The base address in the memory for the alarm section.
 * @return HAL status indicating the result of the operation.
 */
HAL_StatusTypeDef store_alarm(alarmType alarm, uint32_t baseAddress);

/**
 * @brief Reads alarm records from the external flash memory.
 *
 * Retrieves stored alarm records from a specified base address into a provided array.
 *
 * @param baseAddress The base address in memory to start reading from.
 * @param alarmsArray Array to store read alarm records.
 * @param numRecords Pointer to the number of records to read.
 * @return HAL status indicating the result of the operation.
 */
HAL_StatusTypeDef read_alarm_records(uint32_t baseAddress, char (*alarmsArray)[RECORD_SIZE], uint32_t *numRecords);

/**
 * @brief Compares the current RTC time with stored weekly schedules.
 *
 * Checks whether the current time falls within any of the user-defined weekly schedules.
 * Updates global variables with matched record indices if any matches are found.
 */
void compare_weekly_rtc(void);

/**
 * @brief Compares the current RTC time with stored dated schedules.
 *
 * Checks whether the current time falls within any of the user-defined dated schedules.
 * Updates global variables with matched record indices if any matches are found.
 */
void compare_rtc(void);

/**
 * @brief Extracts weekly records from the external flash memory.
 *
 * Reads and stores weekly schedule records from the external flash into an array.
 *
 * @param Ctx Pointer to the QSPI handle.
 */
void ExtractWeeklyRecords(QSPI_HandleTypeDef *Ctx);

/**
 * @brief Checks if the current RTC time matches a specific record.
 *
 * Compares the current RTC time with the start and stop times of a specific record.
 *
 * @param recordIndex Index of the record to compare with.
 * @param startTime Start time of the record.
 * @param stopTime Stop time of the record.
 */
void check_specific_record(uint32_t recordIndex, const char *startTime, const char *stopTime);

/**
 * @brief Extracts dated records from the external flash memory.
 *
 * Reads and stores dated schedule records from the external flash into an array.
 *
 * @param Ctx Pointer to the QSPI handle.
 */
void ExtractRecords(QSPI_HandleTypeDef *Ctx);

/**
 * @brief Sends indices of matched records to an external application.
 *
 * Formats and transmits the indices of matched records over UART.
 *
 * @param matchedIndices Array of indices of matched records.
 * @param matchedCount Pointer to the count of matched records.
 * @param id Identifier to be sent along with the indices.
 */
void sendMatchedIndices(int *matchedIndices, int *matchedCount, const char *id);

/**
 * @brief Prints valid records stored in the system.
 *
 * Iterates through the array of valid records and prints their details.
 */
void PrintValidRecords(void);

/**
 * @brief Prints valid weekly records stored in the system.
 *
 * Iterates through the array of valid weekly records and prints their details.
 */
void PrintWeeklyValidRecords(void);

/**
 * @brief Deletes a record in the external flash memory by its index.
 *
 * Overwrites the record at the specified index with zeros, effectively marking it as deleted.
 *
 * @param hqspi Pointer to the QSPI handle.
 * @param record_index Index of the record to delete.
 * @param base_address Base address in the memory for the records section.
 * @param memory_record_size Size of each record in memory.
 */
void DeleteRecordByIndex(QSPI_HandleTypeDef *hqspi, int record_index, uint32_t base_address, uint32_t memory_record_size);

/**
 * @brief Writes a record to the external flash memory.
 *
 * Writes a specified data record to the external flash, verifying its correctness post-write.
 * Attempts multiple writes in case of failure.
 *
 * @param Baseaddress Base address in memory to start writing to.
 * @param dataToWrite Pointer to the data to be written.
 * @param index Pointer to the index where the data will be written.
 * @param writeMemorySize Size of the memory to write.
 * @param saveRegisterNumber Register number to update after successful write.
 */
void externalFlashWrite(uint32_t Baseaddress, char *dataToWrite, uint32_t *index, uint8_t writeMemorySize, uint8_t saveRegisterNumber);


#endif /* INC_EXTERNAL_FLASH_H_ */
