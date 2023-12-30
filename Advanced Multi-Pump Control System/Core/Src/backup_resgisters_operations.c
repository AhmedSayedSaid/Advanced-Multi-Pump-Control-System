/*
 * backup_resgisters_operations.c
 *
 *  Created on: Sep 10, 2023
 *      Author: GENIUS 1
 */


#define MAX_NUM_PUMPS 8
#define NUM_REGISTERS_PER_PUMP 1
#define PUMPS_FLAG_REGISTER_INDEX (MAX_NUM_PUMPS * NUM_REGISTERS_PER_PUMP)
#define FLAG_VALUE 0x12345678
#define DELETED_FLAG_VALUE 0XFFFFFFFF
#define FACTORY_FLAG_REGISTER_INDEX 22
#define INTERNAL_SYSTEM_DATA_FALG_INDEX 25
#define ALARM_SAVING_REGISTER 28




#include "scheduler.h"
#include "backup_resgisters_operations.h"

// Write a value to a backup register
 void write_backup_register(uint32_t register_index, uint32_t value)
{
	HAL_PWR_EnableBkUpAccess(); // Enable access to the backup domain
	HAL_RTCEx_BKUPWrite(&hrtc, register_index, value); // Write the value to the specified backup register
	HAL_PWR_DisableBkUpAccess(); // Disable access to the backup domain
}

// Read a value from a backup register
 uint32_t read_backup_register(uint32_t register_index)
{
	HAL_PWR_EnableBkUpAccess(); // Enable access to the backup domain
	uint32_t value = HAL_RTCEx_BKUPRead(&hrtc, register_index); // Read the value from the specified backup register
	HAL_PWR_DisableBkUpAccess(); // Disable access to the backup domain
	return value;
}


// Function to write the system pumps data to the backup registers
void write_pump_data_to_backup_registers(void)
{
	for (int i = 0; i < MAX_NUM_PUMPS; i++)
	{
		// Pack the id, workingHours, and workingMinutes fields into a single 32-bit value
		uint32_t packed_value = ((uint32_t) pumpInfoArray[i].id << 24)
				| ((uint32_t) pumpInfoArray[i].workingHours << 8)
				| pumpInfoArray[i].workingMinutes;
		// Write the packed value to a backup register
		write_backup_register(i * NUM_REGISTERS_PER_PUMP, packed_value);
	}
	// Write the flag value to the flag register
	write_backup_register(PUMPS_FLAG_REGISTER_INDEX, FLAG_VALUE);
}


// Function to store the pump data to the backup registers
void delete_pump_data_from_backup_registers(void)
{
	// Write the flag value to the flag register
	write_backup_register(PUMPS_FLAG_REGISTER_INDEX, DELETED_FLAG_VALUE);
}





// Function to restore the pumps data from the backup registers
void restore_pump_data(void)
{
//	 Check if the flag value is set
    if (read_backup_register(PUMPS_FLAG_REGISTER_INDEX) == FLAG_VALUE) {
        // Read the pump data from the backup registers
        for (int i = 0; i < MAX_NUM_PUMPS; i++) {
            // Read the packed value from a backup register
            uint32_t packed_value = read_backup_register(i * NUM_REGISTERS_PER_PUMP);
            // Unpack the id, workingHours, and workingMinutes fields from the packed 32-bit value
            pumpInfoArray[i].id = packed_value >> 24;
            pumpInfoArray[i].workingHours = (packed_value >> 8) & 0xFFFF;
            pumpInfoArray[i].workingMinutes = packed_value & 0xFF;
        }
    } else {
	// Initialize the id field of each PumpInfo struct
	for (uint8_t i = 0; i < MAX_NUM_PUMPS; i++)
	{
		pumpInfoArray[i].id = (i + 1);
	}
    }
}


//function to resotre the saved data of the the application factory page which is saved by the user
void store_factory_page_data(void)
{

    // Pack the offset, VFD_type, and systemPumpsNumber variables into a single 32-bit value
    uint32_t packed_value = ((uint32_t) offset << 16)
            | ((uint32_t) VFD_type << 8)
            | systemPumpsNumber;
    // Write the packed value to the backup register
    write_backup_register(9, packed_value);

    // Pack the firstADCCalibrationReading and secondADCCalibrationReading variables into a single 32-bit value
    packed_value = ((uint32_t) firstADCCalibrationReading << 16)
            | secondADCCalibrationReading;
    // Write the packed value to the backup register
    write_backup_register(10, packed_value);

    // Write the setPointRequired variable to the backup register
    write_backup_register(11, *((uint32_t*)&setPointRequired));

    // Write the Kp variable to the backup register
    write_backup_register(12, *((uint32_t*)&Kp));

    // Write the Ki variable to the backup register
    write_backup_register(13, *((uint32_t*)&Ki));

    // Write the Kd variable to the backup register
    write_backup_register(14, *((uint32_t*)&Kd));

    // Write the currentOfTheFirstReading variable to the backup register
    write_backup_register(15, *((uint32_t*)&currentOfTheFirstReading));

    // Write the currentOfTheSecondReading variable to the backup register
    write_backup_register(16, *((uint32_t*)&currentOfTheSecondReading));

    // Write the minimumPressure variable to the backup register
    write_backup_register(17, *((uint32_t*)&minimumPressure));

    // Write the MaximumPressure variable to the backup register
    write_backup_register(18, *((uint32_t*)&MaximumPressure));

    // Pack the enableDryRun, enableScheduler, and enablePID flags into a single byte
    uint8_t packed_flags = (enableDryRun << 2) | (enableScheduler << 1) | enablePID;
    // Write the packed flags to the backup register
    write_backup_register(19, packed_flags);

    // Write the coffA variable to the backup register
    write_backup_register(20, *((uint32_t*)&coffA));

    // Write the coffB variable to the backup register
    write_backup_register(21, *((uint32_t*)&coffB));

    // Write the cutIn variable to the backup register it is not in order with others
    write_backup_register(26, *((uint32_t*)&cutIn));

    // Write the cutOff variable to the backup register it is not in order with others
    write_backup_register(27, *((uint32_t*)&cutOff));

    // Write the value of the flag that will indicate that the data is saved
	write_backup_register(FACTORY_FLAG_REGISTER_INDEX, FLAG_VALUE);

}

// fucntion to delete the saved data from the backup register so after the system restart it will
//work with initial values
void delete_factory_data_from_backup_registers(void)
{
	// Write the flag value to the flag register
	write_backup_register(FACTORY_FLAG_REGISTER_INDEX, DELETED_FLAG_VALUE);
}

//function to resotre the factory page data after reboting so the system will work with the
//saved data which is saved by the user
void restore_factory_page_data(void)
{
    if (read_backup_register(FACTORY_FLAG_REGISTER_INDEX) == FLAG_VALUE) {
    // Read the packed value from the backup register
    uint32_t packed_value = read_backup_register(9);
    // Unpack the offset, VFD_type, and systemPumpsNumber variables from the packed 32-bit value
    offset = packed_value >> 16;
    VFD_type = (packed_value >> 8) & 0xFF;
    systemPumpsNumber = packed_value & 0xFF;

    // Read the packed value from the backup register
    packed_value = read_backup_register(10);
    // Unpack the firstADCCalibrationReading and secondADCCalibrationReading variables from the packed 32-bit value
    firstADCCalibrationReading = packed_value >> 16;
    secondADCCalibrationReading = packed_value & 0xFFFF;

    // Read the setPointRequired variable from the backup register
    uint32_t temp = read_backup_register(11);
    setPointRequired = *((float*)&temp);

    // Repeat for other float variables...
    temp = read_backup_register(12);
    Kp = *((float*)&temp);

    temp = read_backup_register(13);
    Ki = *((float*)&temp);

    temp = read_backup_register(14);
    Kd = *((float*)&temp);

    temp = read_backup_register(15);
    currentOfTheFirstReading = *((float*)&temp);

    temp = read_backup_register(16);
    currentOfTheSecondReading = *((float*)&temp);

    temp = read_backup_register(17);
    minimumPressure = *((float*)&temp);

    temp = read_backup_register(18);
    MaximumPressure = *((float*)&temp);

    // Read the packed flags from the backup register
    uint8_t packed_flags = read_backup_register(19);
    // Unpack the enableDryRun, enableScheduler, and enablePID flags from the single byte
    enableDryRun = (packed_flags >> 2) & 0x1;
    enableScheduler = (packed_flags >> 1) & 0x1;
    enablePID = packed_flags & 0x1;

    temp = read_backup_register(20);
    coffA =*((float*)&temp);

    temp = read_backup_register(21);
    coffB =*((float*)&temp);

    temp = read_backup_register(26);
    cutIn =*((float*)&temp);

    temp = read_backup_register(27);
    cutOff =*((float*)&temp);

    }
    else{
    	//do nothing and that will make the code with the inital values of the varibles
    }
}

//store the current pid data is if a system reset happens it will continue from the point it stopped on
//instead of reseting all the pumps to 0 then start again
void store_internal_system_data(void)
{
    // Write the accumulated error variable to the backup register
    write_backup_register(23, *((uint32_t*)&accumulatedError));
    // Write the activePumps variable to the backup register
    write_backup_register(24, *((uint32_t*)&prevActivePumps));
    // Write the value of the flag that will indicate that the data is saved
	write_backup_register(INTERNAL_SYSTEM_DATA_FALG_INDEX, FLAG_VALUE);

}

//restore the current pid data is if a system reset happens it will continue from the point it stopped on
//instead of reseting all the pumps to 0 then start again
void restore_internal_system_data(void)
{
    if (read_backup_register(INTERNAL_SYSTEM_DATA_FALG_INDEX) == FLAG_VALUE) {
    	uint32_t temp = read_backup_register(23);
        accumulatedError =*((float*)&temp);
    	 temp = read_backup_register(24);
        prevActivePumps =*((uint8_t*)&temp);
    }
    else{
    	//start pid from 0
    }
}


// Function to delete the pid current state from the backup register so it will start from the
//begging after reset or normal start
void delete_internal_system_data_from_backup_domain_registers(void)
{
	// Write the flag value to the flag register
	write_backup_register(INTERNAL_SYSTEM_DATA_FALG_INDEX, DELETED_FLAG_VALUE);
}

//fucntion is used to save the data coming from alarm that is saved by the user and warning page of the app
//to resotre it when the system reboot
void store_alarm_data(void)
{
    // Pack the four 1-bit variables into a single 4-bit value
    uint32_t packed_value = ((uint32_t)(enableDryRun << 3)
                            | (dryRunSysStop << 2)
                            | (pressureAlarmEnable << 1)
                            | pressureSysStop)
                            | (1 << 31);  // Set the last bit to 1 as data stored flag

    // Write the packed value to the backup register
    write_backup_register(ALARM_SAVING_REGISTER, packed_value);
}

//function to restore the data of the alarm and warning page after restarting the board
void restore_alarm_data(void)
{
    // Read the packed value from the backup register
    uint32_t packed_value = read_backup_register(ALARM_SAVING_REGISTER);

    // Check if the last bit is set to 1 (data stored flag)
    if ((packed_value >> 31) & 0x1) {
        // Unpack the four 1-bit variables from the single 4-bit value
        enableDryRun = (packed_value >> 3) & 0x1;
        dryRunSysStop = (packed_value >> 2) & 0x1;
        pressureAlarmEnable = (packed_value >> 1) & 0x1;
        pressureSysStop = packed_value & 0x1;
    }
    else {
        // Do nothing and that will make the code with the initial values of the variables
    }
}



