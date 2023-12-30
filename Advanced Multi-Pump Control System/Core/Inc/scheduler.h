/*
 * scheduler.h
 *
 *  Created on: Sep 10, 2023
 *      Author: ahmed elsayed
 */


#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include <stdint.h>  // for uint8_t, uint32_t, int32_t
#include "rtc.h"
#include "stm32h7xx_hal.h"
#include "stdbool.h"


// Declare currentTime and currentDate variables to hold real-time clock time and date.

#define MAX_NUM_PUMPS 8
typedef enum {
    START_COUNTING,
    STOP_COUNTING,
	UPDATE_TIME
} CountAction;

typedef enum
{
	stopped=0,
	toRise,
	toShutdown,
	underPIDControl,
}pumpCurrentState;

typedef struct {
    uint8_t id;
    pumpCurrentState state;
    uint16_t currentSpeed;
    uint32_t workingHours;
    uint32_t workingMinutes;
    RTC_TimeTypeDef startTime;
    RTC_DateTypeDef startDate;
    bool isRunning;
} PumpInfo;




// Array of PumpInfo, externally defined with a maximum number of pumps.
extern PumpInfo pumpInfoArray[MAX_NUM_PUMPS];
extern uint8_t schedulerPumpToShutDownID;
extern uint8_t schedulerPumpToRiseID;
extern uint16_t speedPercentageOfPump[8] ;


/**
 * @brief Sorts an array of PumpInfo structures based on working hours and minutes in ascending order.
 * @param array Pointer to the first element of the PumpInfo array.
 * @param size Number of elements in the array.
 */
void Sort_Pumps_By_Working_Hours_and_Minutes(PumpInfo *array, uint8_t size);

/**
 * @brief Sorts an array of PumpInfo structures based on calculated change-over time units in ascending order.
 * @param array Pointer to the first element of the PumpInfo array.
 * @param size Number of elements in the array.
 * @param changeOverTime Change-over time in minutes used to calculate the change-over time units for each pump.
 */
void Sort_Pumps_By_changeOverTime(PumpInfo *array, uint8_t size, uint8_t changeOverTime);

/**
 * @brief Updates the working time of a specific pump.
 * @param id ID of the pump to update.
 * @param action Action to perform (START_COUNTING, STOP_COUNTING, or UPDATE_TIME).
 */
void Update_Working_Time(uint8_t id, CountAction action);

/**
 * @brief Sorts an array of PumpInfo structures based solely on working hours in ascending order.
 * @param array Pointer to the first element of the PumpInfo array.
 * @param size Number of elements in the array.
 */
void Sort_Pumps_By_Working_Hours(PumpInfo *array, uint8_t size);

#endif /* INC_SCHEDULER_H_ */
