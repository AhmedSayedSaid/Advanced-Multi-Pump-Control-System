#include "external_flash.h"

uint8_t datedSchedRegisterNumber = 29;
uint8_t numberOfWeeklyRecordsRegNum = 30;
uint8_t numberOfAlarmsRegNum=31;


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

//this funciton is used to store alarm of the system on the external flah whenever alarm happen
//it tried to write the data to external flash 3 tries and when a try failed that mains that the data written is not correclty written
//so we delete it index by changing it to 0 so with this way it is marked as deleted and try to write the record again on the address after it
//and this happen by reading the data after writeing it and comparing it with what should be written on the external flahs
// and it takes the alarm type to store from the and base address on the memory for alarm seciotn
HAL_StatusTypeDef store_alarm(alarmType alarm, uint32_t baseAddress)
{
    uint8_t flStatus;
    uint8_t readbuf[32]; // Buffer to read data back from flash
    char dataToWrite[32];
    HAL_StatusTypeDef status = HAL_ERROR;
    int record_index = (alarm == dryRun) ? 1 : (alarm == noPressureSensor) ? 2 : 0;

    // Reinitialize the QSPI to exit the memory mapped mode if the external memory is on it
    HAL_QSPI_DeInit(&hqspi);
    HAL_QSPI_MspDeInit(&hqspi);
    MX_QUADSPI_Init();

    // This function will get the date and time of the system RTC and put them in the sDate, sTime.
    Get_RTC_Values();

    // Format the string with the date and time, considering the alarm type
    snprintf(dataToWrite, sizeof(dataToWrite), "%d_%04d_%02d_%02d,%02d:%02d\0",
             record_index, sDate.Year + 2000, sDate.Month, sDate.Date,
             sTime.Hours, sTime.Minutes);

    uint32_t addressToWriteInto = baseAddress + storedAlarmsNumber * ALARM_RECORD_SIZE;

    for (int attempt = 0; attempt < 3; attempt++)
    {
        // Enable writing to the S25FL256S
        flStatus = S25FL256S_WriteEnable(&hqspi, S25FL256S_SPI_MODE);

        if (flStatus != S25FL256S_OK)
        {
            printf("Error: Write Enable failed!\n");
            continue; // Attempt to enable writing again
        }

        // Write the data to the calculated address
        flStatus = S25FL256S_PageProgram(&hqspi, S25FL256S_SPI_MODE, dataToWrite,
                                         addressToWriteInto,  strlen(dataToWrite) + 1);
    	osDelay(5);
        if (flStatus != S25FL256S_OK)
        {
            printf("Attempt %d: Error writing to the external memory!\n", attempt + 1);
            status = HAL_ERROR;
            continue; // Skip the rest of the loop if write failed
        }
		flStatus = S25FL256S_ReadSTR(&hqspi, S25FL256S_SPI_1I4O_MODE, readbuf,
				addressToWriteInto, strlen(dataToWrite) + 1);
		osDelay(5);

        if (flStatus != S25FL256S_OK)
        {
            printf("Attempt %d: Error reading back the data!\n", attempt + 1);
            status = HAL_ERROR;
            continue; // Skip the rest of the loop if read failed
        }

		readbuf[strlen(dataToWrite)] = '\0'; // Set null terminator





        // Print the data that was intended to be written and the data that was read back as strings
        printf("Data to write: %s\n", dataToWrite);
        printf("Data read back: %s\n", readbuf);


        // Check if the read data matches the expected data
        if (strcmp(dataToWrite, readbuf) == 0)
        {
            printf("Verification succeeded after attempt %d.\n", attempt + 1);
            status = HAL_OK;
            storedAlarmsNumber++;
            write_backup_register(numberOfAlarmsRegNum, storedAlarmsNumber);
            break; // Data verified, exit the loop
        }
        else
        {

            printf("Attempt %d: Verification failed, deleting corrupted  record.\n", attempt + 1);
            DeleteRecordByIndex(&hqspi, storedAlarmsNumber, baseAddress, ALARM_RECORD_SIZE);
            // Increment the number of alarms and calculate the new address for the next attempt
            storedAlarmsNumber++;
            write_backup_register(numberOfAlarmsRegNum, storedAlarmsNumber);
            addressToWriteInto = baseAddress + storedAlarmsNumber * ALARM_RECORD_SIZE;
            // If this was the last attempt, set status to HAL_ERROR
            if (attempt == 2) status = HAL_ERROR;
        }
    }

    return status;
}




//this function is used to read the alarms hitstory of the system from the flash memroy
//it takes the base address to read from and the array to store the alarms on it
HAL_StatusTypeDef read_alarm_records(uint32_t baseAddress, char (*alarmsArray)[RECORD_SIZE], uint32_t *numRecords) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t readbuf[64]; // Buffer to read data from flash
    uint32_t addressToRead = baseAddress;
    uint32_t tempNumberOfRecords=0;

    // Start the timeout timer
    uint32_t startTick = HAL_GetTick(); // Get the start tick count

    for (uint32_t recordNum = 0; recordNum <= *numRecords; recordNum++) {
        // Check for timeout
        if ((HAL_GetTick() - startTick) > 2000) { // 2000 ms or 2 seconds
            printf("Timeout while reading alarm records.\n");
            status = HAL_TIMEOUT;
            break;
        }

        // Read a block of memory from the flash
        if (S25FL256S_ReadSTR(&hqspi, S25FL256S_SPI_1I4O_MODE, readbuf, addressToRead, sizeof(readbuf)) != S25FL256S_OK) {
            printf("Error reading from flash at address %lu\n", addressToRead);
            status = HAL_ERROR;
            break;
        }

        // Check if this is a block of 0xFFs indicating the end of the records
        int is_end = 1;
        for (int i = 0; i < 64; i++) {
            if (readbuf[i] != 0xFF) {
                is_end = 0;
                break;
            }
        }

        if (is_end) {
            printf("End of records reached at address %lu\n", addressToRead);
            break; // Break out of loop if end of records is found
        }

        // If the index is not 0, copy the record to the alarms array
        if (readbuf[0] != '0') {
            memcpy(alarmsArray[tempNumberOfRecords], readbuf, RECORD_SIZE);
            tempNumberOfRecords++;
        }

        // Move to the next record address
        addressToRead += RECORD_SIZE;
    }

    return status;
}

// this funciton compare the current date and time with the by user stored weekly days
// that the system PID should work automattaclly on and extract the date and time that the current date and time are within it
void compare_weekly_rtc()
{

	matchedWeeklyRecordCount = 0;  // Reset matchedWeeklyRecordCount to zero

	Get_RTC_Values(); // gets the  current sDate and sTime.

	char currentTime[6];  // HH:MM format
	sprintf(currentTime, "%02d:%02d", sTime.Hours, sTime.Minutes);

	uint8_t currentDay = sDate.WeekDay; //  this gives current day, where 1 = Monday, 2 = Tuesday, and so on.

	int matchingRecordIndex = -1;

	for (int i = 0; i < valid_weekly_record_count; i++)
	{
		char startTime[6];
		char endTime[6];
		uint8_t days[MAX_DAYS]; // assuming the length of days array

		// Extracting the values using sscanf
		if (sscanf(valid_weekly_records[i].original_record,
				"%d_%5[^,],%5[^,],%s", &matchingRecordIndex, startTime, endTime,
				days) < 3)
		{
			continue;  // If extraction failed, skip this iteration.
		}

		// Check if currentDay exists in days.
		int dayExists = 0;
		for (int j = 0; j < strlen((char*) days); j++)
		{
			if (days[j] - '0' == currentDay)  // Convert char to int and check
			{
				dayExists = 1;
				break;
			}
		}

		if (!dayExists)
			continue; // If current day doesn't exist in the record's days, skip to the next record.

		// Compare currentTime with startTime and endTime
		if (strcmp(currentTime, startTime) >= 0
				&& strcmp(currentTime, endTime) <= 0)
		{
			// If currentTime is in between the start and end times

			matchedWeeklyRecordIndices[matchedWeeklyRecordCount++] =
					matchingRecordIndex;

		}
	}
	for (int i = 0; i < matchedWeeklyRecordCount; i++)
	{
		printf("Matching weekly record found with index: %d\n",
				matchedWeeklyRecordIndices[i]);
	}
	printf("\n");

}
// this funciton compare the current date and time with the by user stored dated records
// that the system PID should work automatically on and extract the date and time that the current date and time are within it
void compare_rtc()
{
	matchedRecordCount = -1; // Initialize to -1 to indicate that no records have been matched yet

	Get_RTC_Values();

	char systemRTC[20];  // YYYY-MM-DDTHH:MM:SS format
	sprintf(systemRTC, "%04d-%02d-%02dT%02d:%02d:%02d", 2000 + sDate.Year,
			sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

	for (int i = 0; i < valid_record_count; i++)
	{
		int index;
		char startDate[20];
		char endDate[20];

		if (sscanf(valid_records[i].original_record, "%d_%19[^,],%19s", &index,
				startDate, endDate) != 3)
		{
			continue; // If sscanf doesn't return 3, the extraction failed. Skip this iteration.
		}

		if (strcmp(systemRTC, startDate) >= 0
				&& strcmp(systemRTC, endDate) <= 0)
		{
			if (matchedRecordCount == -1)
			{
				matchedRecordCount = 0; // Found the first record, so initialize to 0
			}
			matchedRecordIndices[matchedRecordCount++] = index;
		}
	}

	if (matchedRecordCount == -1)
	{
		matchedRecordCount = 0;  // No records found, set count to 0
		printf("No matching record found for the current system RTC.\n");
	}
	else
	{
		for (int i = 0; i < matchedRecordCount; i++)
		{
			printf("Matching record found with index: %d\n",
					matchedRecordIndices[i]);
		}
	}
}
//this function is used to extract weekly record from the external flash memroy and store them
// in array
void ExtractWeeklyRecords(QSPI_HandleTypeDef *Ctx)
{
	uint8_t readbuf[WEEKLY_RECORD_SIZE];
	uint8_t readbufback[WEEKLY_RECORD_SIZE];
	uint32_t address = WEEKLY_SCHEDULAR_RECORDS_START_ADDRESS;
	valid_weekly_record_count = 0;

	if (firstTimeRequest == 0)
	{
		//switch to QSPI
		flStatus = S25FL256S_Enter4LinesDataMode(&hqspi, S25FL256S_SPI_MODE);
		printf("Entering to QSPI, status = %ld\r\n", flStatus);
		printf("SysTick2: %ld\r\n", HAL_GetTick());
		firstTimeRequest = 1;
	}
	else
	{
		HAL_QSPI_DeInit(&hqspi);
		HAL_QSPI_MspDeInit(&hqspi);
		MX_QUADSPI_Init();
	}

	while (1)
	{
		// Reading from flash
		S25FL256S_ReadSTR(Ctx, S25FL256S_SPI_1I4O_MODE, readbuf, address,
		WEEKLY_RECORD_SIZE);
		osDelay(5);

		// Copying the content of readbuf to readbufback
		memcpy(readbufback, readbuf, WEEKLY_RECORD_SIZE);

		// Check if buffer contains all 0xFF (or ÿ in ASCII)
		int is_end = 1;
		for (int i = 0; i < 25; i++)
		{
			if (readbuf[i] != 0xFF)
			{
				is_end = 0;
				break;
			}
		}

		if (is_end)
			break;

		// Extract the record index using strtok
		char *token = strtok((char*) readbuf, "_");
		int index = atoi(token);

		char str_index_check[10];
		sprintf(str_index_check, "%d", index);

		if (index != 0 && strcmp(token, str_index_check) == 0)
		{
			strncpy(
					valid_weekly_records[valid_weekly_record_count].original_record,
					(char*) readbufback, sizeof(readbufback));

			char *start_time_str = strtok(NULL, ",");
			char *stop_time_str = strtok(NULL, ",");
			char *days = strtok(NULL, "\0");

			int start_hour, start_minute, stop_hour, stop_minute;
			sscanf(start_time_str, "%2d:%2d", &start_hour, &start_minute);
			sscanf(stop_time_str, "%2d:%2d", &stop_hour, &stop_minute);

			STM32_RTC start_time =
			{ .hour = start_hour, .minute = start_minute, .second = 0 };
			STM32_RTC stop_time =
			{ .hour = stop_hour, .minute = stop_minute, .second = 0 };

			valid_weekly_records[valid_weekly_record_count].index = index;
			valid_weekly_records[valid_weekly_record_count].start_time =
					start_time;
			valid_weekly_records[valid_weekly_record_count].end_time =
					stop_time;

			for (int i = 0; i < strlen(days); i++)
			{
				valid_weekly_records[valid_weekly_record_count].days[i] =
						days[i] - '0';
			}

			valid_weekly_record_count++;

			if (valid_weekly_record_count >= MAX_RECORDS)
				break;
		}

		address += WEEKLY_RECORD_SIZE;
	}
}

// this funciton compare the current rtc date and time with a specific record
// to show wether we are in the middle of this record or not
void check_specific_record(uint32_t recordIndex, const char *startTime,
		const char *stopTime)
{
	Get_RTC_Values();

	// Convert the system's RTC to a comparable string format
	char systemRTC[20];  // Assuming YYYY-MM-DDTHH:MM:SS format
	sprintf(systemRTC, "%04d-%02d-%02dT%02d:%02d:%02d", 2000 + sDate.Year,
			sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

	// Compare systemRTC with startTime and stopTime
	if (strcmp(systemRTC, startTime) >= 0 && strcmp(systemRTC, stopTime) <= 0)
	{
		// If systemRTC is in between the start and end date-time
		printf("Current RTC matches with the given index %d range: %s to %s\n",
				recordIndex, startTime, stopTime);
	}
	else
	{
		printf(
				"Current RTC does NOT match with the given index %d range: %s to %s\n",
				recordIndex, startTime, stopTime);
	}
}
// this function extract the dated records from the external flash and store them and every boot of the board
void ExtractRecords(QSPI_HandleTypeDef *Ctx)
{
	uint8_t readbuf[DATED_RECORD_SIZE];
	uint8_t readbufback[DATED_RECORD_SIZE];
	uint32_t address = DATED_SCHEDULAR_RECORDS_START_ADDRESS;
	valid_record_count = 0;
	if (firstTimeRequest == 0)
	{
		//switch to QSPI
		flStatus = S25FL256S_Enter4LinesDataMode(&hqspi, S25FL256S_SPI_MODE);
		printf("Entering to QSPI, status = %ld\r\n", flStatus);

		printf("SysTick2: %ld\r\n", HAL_GetTick());

		firstTimeRequest = 1;
	}
	else
	{
		HAL_QSPI_DeInit(&hqspi);
		HAL_QSPI_MspDeInit(&hqspi);
		MX_QUADSPI_Init();
	}
	while (1)
	{
		// Reading from flash
		S25FL256S_ReadSTR(Ctx, S25FL256S_SPI_1I4O_MODE, readbuf, address,
		DATED_RECORD_SIZE);
		osDelay(5);
		// Copying the content of readbuf to readbufback
		memcpy(readbufback, readbuf, DATED_RECORD_SIZE);
		//printf("readbuf %s \n", readbuf);
		// Check if buffer contains all 0xFF (or ÿ in ASCII)
		int is_end = 1;
		for (int i = 0; i < 25; i++)
		{
			if (readbuf[i] != 0xFF)
			{
				is_end = 0;
				break;
			}
		}

		if (is_end)
		{
			break;  // Break out of loop if end of records is found
		}

		// Extract the record index using strtok
		char *token = strtok((char*) readbuf, "_");
		int index = atoi(token);

		// Check for leading zeros
		char str_index_check[10];
		sprintf(str_index_check, "%d", index);

		if (index != 0 && strcmp(token, str_index_check) == 0)
		{
			// Save the original record
			strncpy(valid_records[valid_record_count].original_record,
					(char*) readbufback, sizeof(readbufback));

			// Parse the start date and time
			char *start_time_str = strtok(NULL, ",");

			// Parse the end date and time
			char *end_time_str = strtok(NULL, "\0");

			// Parse the start and end times
			int year, month, day, hour, minute, second;

			sscanf(start_time_str, "%4d-%2d-%2dT%2d:%2d:%2d", &year, &month,
					&day, &hour, &minute, &second);

			STM32_RTC start_time;
			start_time.year = year;
			start_time.month = month;
			start_time.day = day;
			start_time.hour = hour;
			start_time.minute = minute;
			start_time.second = second;

			sscanf(end_time_str, "%4d-%2d-%2dT%2d:%2d:%2d", &year, &month, &day,
					&hour, &minute, &second);

			STM32_RTC end_time;
			end_time.year = year;
			end_time.month = month;
			end_time.day = day;
			end_time.hour = hour;
			end_time.minute = minute;
			end_time.second = second;

			// Save to valid records array
			valid_records[valid_record_count].index = index;
			valid_records[valid_record_count].start_time = start_time;
			valid_records[valid_record_count].end_time = end_time;

			valid_record_count++;

			// Safety check to avoid storing beyond the buffer size
			if (valid_record_count >= MAX_RECORDS)
			{
				break;
			}
		}

		// Move to the next record
		address += DATED_RECORD_SIZE;
	}

}
// this function is used to send the given record to the app to highlight them in green on the scheduler records
void sendMatchedIndices(int *matchedIndices, int *matchedCount, const char *id)
{
	char value[10];  // Buffer to hold the string representation of the index
	char *DataSend;

	for (int i = 0; i < *matchedCount; i++)
	{
		sprintf(value, "%d", matchedIndices[i]);
		DataSend = PrepareData(id, value);
		HAL_UART_Transmit(&huart1, (uint8_t*) DataSend, strlen(DataSend), 10);
	}

}

void PrintValidRecords()
{
	for (int i = 0; i < RECORD_BUFFER_SIZE; i++)
	{
		if (valid_records[i].index != 0)
		{  // Check if record is valid based on index
			printf("Record Index: %d\n", valid_records[i].index);
			printf("print Start Time: %04d-%02d-%02dT%02d:%02d:%02d\n",
					valid_records[i].start_time.year,
					valid_records[i].start_time.month,
					valid_records[i].start_time.day,
					valid_records[i].start_time.hour,
					valid_records[i].start_time.minute,
					valid_records[i].start_time.second);
			printf("Assigned End Time: %04d-%02d-%02dT%02d:%02d:%02d\n",
					valid_records[i].end_time.year,
					valid_records[i].end_time.month,
					valid_records[i].end_time.day,
					valid_records[i].end_time.hour,
					valid_records[i].end_time.minute,
					valid_records[i].end_time.second);
		}
	}
}
// this function is used to print avaible weekly reocrds that was extracted from the external flahs memory
void PrintWeeklyValidRecords()
{
	for (int i = 0; i < valid_weekly_record_count; i++)
	{
		if (valid_weekly_records[i].index != 0)
		{
			printf("Record Index: %d\n", valid_weekly_records[i].index);
			printf("Start Time: %02d:%02d\n",
					valid_weekly_records[i].start_time.hour,
					valid_weekly_records[i].start_time.minute);
			printf("End Time: %02d:%02d\n",
					valid_weekly_records[i].end_time.hour,
					valid_weekly_records[i].end_time.minute);

			printf("Days: ");
			for (int j = 0;
					j < MAX_DAYS && valid_weekly_records[i].days[j] != 0xFF;
					j++)
			{
				printf("%d ", valid_weekly_records[i].days[j]);
			}
			printf("\n");
		}
	}
}

// this funciton is used to delete any reocrd in the flash memory using the index of it
void DeleteRecordByIndex(QSPI_HandleTypeDef *hqspi, int record_index,
		uint32_t base_address, uint32_t memroy_record_size)
{

	if (firstTimeRequest == 0)
	{
		flStatus = S25FL256S_Enter4LinesDataMode(&hqspi, S25FL256S_SPI_MODE);
		printf("Entering to QSPI, status = %ld\r\n", flStatus);

		firstTimeRequest = 1;

	}
	else
	{
		HAL_QSPI_DeInit(hqspi);
		HAL_QSPI_MspDeInit(hqspi);
		MX_QUADSPI_Init();
	}
	// Calculate the address of the desired record
	uint32_t address = base_address + (record_index - 1) * memroy_record_size;
	osDelay(5);
	// Read the record to identify the size of the index
	uint8_t readBuffer[memroy_record_size];
	S25FL256S_ReadSTR(hqspi, S25FL256S_SPI_1I4O_MODE, readBuffer, address,
			memroy_record_size);

	osDelay(5);
	printf("the delete fucntion buffer %s \n", readBuffer);
	printf("the address of the record to delete %X \n", address);
	// Locate the '_' character to determine the number of digits in the index
	int indexSize = 0;
	for (int i = 0; i < memroy_record_size; i++)
	{
		if (readBuffer[i] == '_')
		{
			indexSize = i;

			break;
		}
	}

	// Prepare data to overwrite the index with zeros
	uint8_t dataToWrite[indexSize];
	memset(dataToWrite, '0', indexSize);  // Setting all characters to '0'

	// Enable writing to the S25FL256S
	S25FL256S_WriteEnable(hqspi, S25FL256S_SPI_MODE);

	// Write the data to the calculated address to overwrite the index
	flStatus = S25FL256S_PageProgram(hqspi, S25FL256S_SPI_MODE, dataToWrite,
			address, indexSize);
	osDelay(5);
	if (flStatus != S25FL256S_OK)
	{
		printf("Error writing to the external memory!\n");
	}
	else
	{
		printf("deleted record with index %d \n", record_index);
	}

}

#define MAX_RETRIES 3
// this funciton is used to write a record to the flash memory
void externalFlashWrite(uint32_t Baseaddress, char *dataToWrite,
		uint32_t *index, uint8_t writeMemorySize, uint8_t saveRegisterNumber)
{
	uint8_t readbuf[DATED_RECORD_SIZE];
	uint32_t lastWrittenAddress = 0;
	char dataWithIndex[DATED_RECORD_SIZE];
	uint8_t exitFlag = 0;

	if (firstTimeRequest == 0)
	{
		flStatus = S25FL256S_Enter4LinesDataMode(&hqspi, S25FL256S_SPI_MODE);
		printf("Entering to QSPI, status = %ld\r\n", flStatus);
		firstTimeRequest = 1;
	}
	else
	{
		HAL_QSPI_DeInit(&hqspi);
		HAL_QSPI_MspDeInit(&hqspi);
		MX_QUADSPI_Init();
	}

	while (exitFlag < MAX_RETRIES)
	{
		sprintf(dataWithIndex, "%d_%s", *index, dataToWrite);

		lastWrittenAddress = ((*index - 1) * writeMemorySize) + Baseaddress;


		S25FL256S_WriteEnable(&hqspi, S25FL256S_SPI_MODE);
		flStatus = S25FL256S_PageProgram(&hqspi, S25FL256S_SPI_MODE,
				dataWithIndex, lastWrittenAddress, strlen(dataWithIndex) + 1);


		if (flStatus != S25FL256S_OK)
		{
			printf("Failed to write data. Status = %ld\r\n", flStatus);

		}

		osDelay(8);

		flStatus = S25FL256S_ReadSTR(&hqspi, S25FL256S_SPI_1I4O_MODE, readbuf,
				lastWrittenAddress, strlen(dataWithIndex) + 1);

		if (flStatus != S25FL256S_OK)
		{
			printf("Failed to read data. Status = %ld\r\n", flStatus);

		}

		readbuf[strlen(dataWithIndex)] = '\0'; // Set null terminator
		osDelay(8);

		printf("dataWithIndex: %s\n", dataWithIndex);
		printf("readbuf: %s\n", readbuf);

		if (strcmp(dataWithIndex, readbuf) == 0)
		{
			printf("the data record written successfully \n");
			(*index)++;
			write_backup_register(saveRegisterNumber, *index);
			exitFlag = MAX_RETRIES;
		}
		else
		{
			printf("failed to write the data \n");
			DeleteRecordByIndex(&hqspi, *index,
			DATED_SCHEDULAR_RECORDS_START_ADDRESS, DATED_RECORD_SIZE);
			printf("deleted FAULTY RECORD INDEX %d \n", *index);
			lastWrittenAddress = (*index) * writeMemorySize;
			(*index)++;
			write_backup_register(saveRegisterNumber, *index);
			printf("the new index %i \n", *index);
			exitFlag++;
		}
	}

}
