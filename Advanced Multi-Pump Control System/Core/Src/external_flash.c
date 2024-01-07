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

#include "external_flash.h"

uint8_t datedSchedRegisterNumber = 29;
uint8_t numberOfWeeklyRecordsRegNum = 30;
uint8_t numberOfAlarmsRegNum = 31;

char records[1000][48];
uint32_t schedulerRecordsStartAddress = 0;
uint32_t schedulerRecordsCurrentAddress = 0;
uint8_t eraseExternalFlashFlag = 0;

uint8_t stopTimeFlag = 0;
volatile uint8_t removeRecordFlag = 0;
volatile uint8_t removeWeeklyRecordFlag = 0;

int matchedRecordCount = -1;
int matchedWeeklyRecordCount = -1;
int valid_record_count = 0;
uint32_t valid_weekly_record_count = 0;
int matchingRecordIndex = -1;

int matchedRecordIndices[MAX_VALID_RECORDS];
int matchedWeeklyRecordIndices[MAX_VALID_WEEKLY_RECORDS];

Record valid_records[MAX_RECORDS];
weekRecord valid_weekly_records[MAX_RECORDS];

int32_t flStatus;
uint8_t firstTimeRequest = 0;
uint8_t date_week_sched_disabled = 1;
uint8_t schedDisabledSystem = 0;
char startTime[30];
char stopTime[30];
char fullDatedSchedRecord[48];
char weeklyRecord[30];
uint8_t weeklyRecordFlag = 0;
uint16_t indexOfRecordToRemove;
char stopTimeRemove[30];
uint8_t addschedData = 0;
uint8_t removeschedData = 0;
// Define the global variables:

uint32_t numberOfSchedulerRecords = 1;
uint32_t numberOfWeeklyRecords = 1;
uint32_t storedAlarmsNumber = 0;

extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

// this funciton is used to store alarm of the system on the external flah whenever alarm happen
// it tried to write the data to external flash 3 tries and when a try failed that mains that the data written is not correclty written
// so we delete it index by changing it to 0 so with this way it is marked as deleted and try to write the record again on the address after it
// and this happen by reading the data after writeing it and comparing it with what should be written on the external flahs
//  and it takes the alarm type to store from the and base address on the memory for alarm seciotn
HAL_StatusTypeDef store_alarm(alarmType alarm, uint32_t baseAddress)
{
}

// this function is used to read the alarms hitstory of the system from the flash memroy
// it takes the base address to read from and the array to store the alarms on it
HAL_StatusTypeDef read_alarm_records(uint32_t baseAddress, char (*alarmsArray)[RECORD_SIZE], uint32_t *numRecords)
{
}

// this funciton compare the current date and time with the by user stored weekly days
// that the system PID should work automattaclly on and extract the date and time that the current date and time are within it
void compare_weekly_rtc()
{
}
// this funciton compare the current date and time with the by user stored dated records
// that the system PID should work automatically on and extract the date and time that the current date and time are within it
void compare_rtc()
{
}
// this function is used to extract weekly record from the external flash memroy and store them
//  in array
void ExtractWeeklyRecords(QSPI_HandleTypeDef *Ctx)
{
}

// this funciton compare the current rtc date and time with a specific record
// to show wether we are in the middle of this record or not
void check_specific_record(uint32_t recordIndex, const char *startTime,
						   const char *stopTime)
{
}
// this function extract the dated records from the external flash and store them and every boot of the board
void ExtractRecords(QSPI_HandleTypeDef *Ctx)
{
}
// this function is used to send the given record to the app to highlight them in green on the scheduler records
void sendMatchedIndices(int *matchedIndices, int *matchedCount, const char *id)
{
}

void PrintValidRecords()
{
}
// this function is used to print avaible weekly reocrds that was extracted from the external flahs memory
void PrintWeeklyValidRecords()
{
}

// this funciton is used to delete any reocrd in the flash memory using the index of it
void DeleteRecordByIndex(QSPI_HandleTypeDef *hqspi, int record_index,
						 uint32_t base_address, uint32_t memroy_record_size)
{
}

#define MAX_RETRIES 3
// this funciton is used to write a record to the flash memory
void externalFlashWrite(uint32_t Baseaddress, char *dataToWrite,
						uint32_t *index, uint8_t writeMemorySize, uint8_t saveRegisterNumber)
{
}
