#include "scheduler.h"
#include "backup_resgisters_operations.h"

RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;

// Array storing the number of days in each month. Index 0 is unused.
static int daysInMonth[] =
{ 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

// Function to determine if a given year is a leap year.
static bool isLeapYear(int year)
{
	// Returns true if the year is divisible by 4 and not divisible by 100,
	// or if it's divisible by 400.
	return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

// Assuming daysInMonth is an array where January is daysInMonth[0], February is daysInMonth[1], etc.
static int32_t calculateElapsedDays(RTC_DateTypeDef startDate,
		RTC_DateTypeDef endDate)
{
	int32_t elapsedDays = 0;
	int startYear = 2000 + startDate.Year;
	int endYear = 2000 + endDate.Year;

	// Handle same-day start and end first
	if (startYear == endYear && startDate.Month == endDate.Month
			&& startDate.Date == endDate.Date)
	{
		return 0;
	}

	for (int year = startYear; year <= endYear; year++)
	{
		int startMonth = (year == startYear) ? startDate.Month : 1;
		int endMonth = (year == endYear) ? endDate.Month : 12;

		for (int month = startMonth; month <= endMonth; month++)
		{
			if (year == startYear && month == startDate.Month)
			{
				elapsedDays += daysInMonth[month - 1] - startDate.Date;
			}
			else if (year == endYear && month == endDate.Month)
			{
				elapsedDays += endDate.Date;
			}
			else
			{
				elapsedDays += daysInMonth[month - 1];  // General case
			}

			// Add an extra day if it's a leap year and the month is February
			if (month == 2 && isLeapYear(year))
			{
				elapsedDays += 1;
			}
		}
	}
	return elapsedDays;
}

// Function to update pump working time information.
static void updatePumpInfo(PumpInfo *pInfo, uint32_t elapsedDays,
		RTC_TimeTypeDef startTime, RTC_TimeTypeDef endTime, CountAction action)
{
	// Calculate the total elapsed hours from the elapsed days and the difference in start and end time.
	uint32_t elapsedHours = elapsedDays * 24
			+ (endTime.Hours - startTime.Hours);

	// Calculate the elapsed minutes between end time and start time.
	int32_t elapsedMinutes = endTime.Minutes - startTime.Minutes;

//	printf("id %d \n", pInfo->id);
//	// Print the details of the elapsedMinutes, startTime, and endTime variables.
//	printf("Elapsed minutes: %d\n", elapsedMinutes);
//	printf("Start time: %02d:%02d:%02d\n", startTime.Hours, startTime.Minutes,
//			startTime.Seconds);
//	printf("End time: %02d:%02d:%02d\n", endTime.Hours, endTime.Minutes,
//			endTime.Seconds);
//	printf("\n");

	// If elapsedMinutes is negative, make the necessary adjustments.
	if (elapsedMinutes < 0)
	{
		elapsedMinutes += 60;
		elapsedHours -= 1;
	}

	// Update the pump information with the newly calculated elapsed hours and minutes.
	pInfo->workingHours += elapsedHours;
	pInfo->workingMinutes += elapsedMinutes;

	if (action == UPDATE_TIME)
	{
		// Store the current time and date as the starting time and date.
		pInfo->startTime = currentTime;
		pInfo->startDate = currentDate;
	}

	// If workingMinutes crosses 60, convert it into hours and update workingHours.
	if (pInfo->workingMinutes >= 60)
	{
		pInfo->workingMinutes -= 60;
		pInfo->workingHours += 1;
	}

}

//Loop thorugh array to find the id passed to the function

static PumpInfo* findPumpById(uint8_t id)
{
	for (int i = 0; i < MAX_NUM_PUMPS; i++)
	{
		if (pumpInfoArray[i].id == id)
		{
			return &pumpInfoArray[i];
		}
	}

	return NULL;
}

// After sorting, the pump with the lowest changeOverTime Units will be at the beginning of the array,
// and the pump with the changeOverTime Units will be at the end of the array.

//void Sort_Pumps_By_changeOverTime(PumpInfo *array, uint8_t size, uint8_t changeOverTime)
//{
//    // Using the bubble sort algorithm for sorting.
//    // The outer loop iterates through each element of the array.
//    for (uint8_t i = 0; i < size; i++)
//    {
//        // The inner loop iterates through the remaining elements in the array after the i-th element.
//        for (uint8_t j = i + 1; j < size; j++)
//        {
//            uint8_t pump1ChangeOverUnits = ((array[i].workingHours * 60) + array[i].workingMinutes) / changeOverTime;
//            uint8_t pump2ChangeOverUnits = ((array[j].workingHours * 60) + array[j].workingMinutes) / changeOverTime;
//
//            if (pump1ChangeOverUnits > pump2ChangeOverUnits)
//            {
//                PumpInfo temp = array[i];
//                array[i] = array[j];
//                array[j] = temp;
//
//            }
//
//        }
//    }
//}

void Sort_Pumps_By_changeOverTime(PumpInfo *array, uint8_t size,
		uint8_t changeOverTime)
{
	int8_t maxIndex = -1;
	uint8_t maxPumpChangeOverUnits = 0;
	uint8_t changeOrderTrigger = 0;
	for (uint8_t i = 0; i < size; i++)
	{
		PumpInfo temp = array[i];
		uint8_t pumpChangeOverUnits = ((temp.workingHours * 60)
				+ temp.workingMinutes) / changeOverTime;

		if (pumpChangeOverUnits > maxPumpChangeOverUnits)
		{
			maxPumpChangeOverUnits = pumpChangeOverUnits;
			maxIndex = i;

		}
	}

	if (maxIndex >= 0)
	{
		//check that all the pumps after it not equal to it
		if (maxIndex < (size-1))
		{
			for (uint8_t i = maxIndex + 1; i < size; i++)
			{

				PumpInfo temp = array[i];
				uint8_t ChangeOverUnits = ((temp.workingHours * 60)
						+ temp.workingMinutes) / changeOverTime;

				if (ChangeOverUnits < maxPumpChangeOverUnits)
				{
					changeOrderTrigger = 1;
					break;
				}
			}
		}
		if (changeOrderTrigger)
		{
			PumpInfo temp = array[maxIndex];
			for (uint8_t i = maxIndex; i < size - 1; i++)
			{
				array[i] = array[i + 1];
			}
			array[size - 1] = temp;
		}
	}
}

void Sort_Pumps_By_Working_Hours_and_Minutes(PumpInfo *array, uint8_t size)
{
	// Using the bubble sort algorithm for sorting.
	// The outer loop iterates through each element of the array.
	for (uint8_t i = 0; i < size; i++)
	{
		// The inner loop iterates through the remaining elements in the array after the i-th element.
		for (uint8_t j = i + 1; j < size; j++)
		{
			// Compare the workingHours of the two PumpInfo objects.
			// Sorting is primarily based on 'workingHours' in ascending order.
			if (array[i].workingHours > array[j].workingHours)
			{
				// Swap array[i] and array[j] if the workingHours of array[i] is greater than array[j].
				PumpInfo temp = array[i];
				array[i] = array[j];
				array[j] = temp;
			}
			// If the workingHours are the same, then consider 'workingMinutes' for sorting.
			else if (array[i].workingHours == array[j].workingHours)
			{
				// Sorting is based on 'workingMinutes' in ascending order if 'workingHours' are equal.
				// Swap array[i] and array[j] if the workingMinutes of array[i] is greater than array[j].
				if (array[i].workingMinutes > array[j].workingMinutes)
				{
					PumpInfo temp = array[i];
					array[i] = array[j];
					array[j] = temp;
				}
			}
		}
	}
}

void Sort_Pumps_By_Working_Hours(PumpInfo *array, uint8_t size)
{
	// Basic bubble sort
	for (uint8_t i = 0; i < size; i++)
	{
		for (uint8_t j = i + 1; j < size; j++)
		{
			if (array[i].workingHours > array[j].workingHours)
			{
				PumpInfo temp = array[i];
				array[i] = array[j];
				array[j] = temp;
			}
		}
	}
}


// This function updates the working time of a pump based on its ID and the action (START_COUNTING or STOP_COUNTING).
void Update_Working_Time(uint8_t id, CountAction action)
{

	// Get the current time and date from the RTC (Real-Time Clock) module and store them in currentTime and currentDate.
	HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
//	printf("Current time: %02d:%02d:%02d\n", currentTime.Hours, currentTime.Minutes, currentTime.Seconds);
//	printf("Current date: %02d/%02d/%04d\n", currentDate.Date, currentDate.Month, 2000 + currentDate.Year);
//	printf("\n");
	// Check if the pump ID is within the valid range of pump IDs.
	if (id >= 1 && id <= MAX_NUM_PUMPS)
	{
		// Use the findPumpById function to get the PumpInfo structure associated with the given pump ID.
		PumpInfo *pInfo = findPumpById(id);

		// Check if pInfo is NULL, which means the pump ID was not found.
		if (pInfo == NULL)
		{
			// Handle the error for pump ID not found (this is an infinite loop for now; you may want better error handling).
			//	while (1)
			;
		}

		// Check if the action is to start counting time.
		if (action == START_COUNTING)
		{
			// Store the current time and date as the starting time and date.
			pInfo->startTime = currentTime;
			pInfo->startDate = currentDate;
			pInfo->isRunning = true; // Indicate that the pump is currently working.
		}
		// Check if the action is to stop counting time.
		else if (action == STOP_COUNTING)
		{
			// Only proceed if the pump is actually counting (i.e., it was previously started).
			if (pInfo->isRunning)
			{
				// Calculate the number of elapsed days between the start and stop dates.
				int32_t elapsedDays = calculateElapsedDays(pInfo->startDate,
						currentDate);

				// Update the working hours and minutes for the pump.
				updatePumpInfo(pInfo, elapsedDays, pInfo->startTime,
						currentTime, STOP_COUNTING);
				pInfo->isRunning = false;
			}
			else
			{
				// Handle the error where STOP_COUNTING is called without a preceding START_COUNTING (infinite loop for now).
				//  while(1);
			}
		}
		else if (action == UPDATE_TIME)
		{
			if (pInfo->isRunning)
			{
				// Calculate the number of elapsed days between the start and stop dates.
				int32_t elapsedDays = calculateElapsedDays(pInfo->startDate,
						currentDate);

				// Update the working hours and minutes for the pump.
				updatePumpInfo(pInfo, elapsedDays, pInfo->startTime,
						currentTime, UPDATE_TIME);
			}
			else
			{

			}
		}
	}
	else
	{
		// Handle the error for invalid pump ID (infinite loop for now; better error handling may be needed).
		//while (1)
		;
	}
}
