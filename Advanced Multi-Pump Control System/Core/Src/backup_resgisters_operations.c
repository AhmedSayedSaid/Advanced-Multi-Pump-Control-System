/*
 * backup_resgisters_operations.c
 *
 *  Created on: Sep 10, 2023
 *      Author: Ahmed sayed
 * note the code is
 */

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
    // Enable access to the backup domain
    // Write the value to the specified backup register
    // Disable access to the backup domain
}

// Read a value from a backup register
uint32_t read_backup_register(uint32_t register_index)
{
    // Enable access to the backup domain
    // Read the value from the specified backup register
    // Disable access to the backup domain
}

// Function to write the system pumps data to the backup registers
void write_pump_data_to_backup_registers(void)
{

    // Pack the id, workingHours, and workingMinutes fields into a single 32-bit value

    // Write the packed value to a backup register
}
// Write the flag value to the flag register
}

// Function to store the pump data to the backup registers
void delete_pump_data_from_backup_registers(void)
{
    // Write the flag value to the flag register
}

// Function to restore the pumps data from the backup registers
void restore_pump_data(void)
{
    //	 Check if the flag value is set

    // Read the pump data from the backup registers

    // Read the packed value from a backup register

    // Unpack the id, workingHours, and workingMinutes fields from the packed 32-bit value
}
}
else
{
    // Initialize the id field of each PumpInfo struct
}
}

// function to resotre the saved data of the the application factory page which is saved by the user
void store_factory_page_data(void)
{

    // Pack the offset, VFD_type, and systemPumpsNumber variables into a single 32-bit value

    // Write the packed value to the backup register

    // Pack the firstADCCalibrationReading and secondADCCalibrationReading variables into a single 32-bit value

    // Write the packed value to the backup register

    // Write the setPointRequired variable to the backup register

    // Write the Kp variable to the backup register

    // Write the Ki variable to the backup register

    // Write the Kd variable to the backup register

    // Write the currentOfTheFirstReading variable to the backup register

    // Write the currentOfTheSecondReading variable to the backup register

    // Write the minimumPressure variable to the backup register

    // Write the MaximumPressure variable to the backup register

    // Pack the enableDryRun, enableScheduler, and enablePID flags into a single byte

    // Write the packed flags to the backup register

    // Write the coffA variable to the backup register

    // Write the coffB variable to the backup register

    // Write the cutIn variable to the backup register it is not in order with others

    // Write the cutOff variable to the backup register it is not in order with others

    // Write the value of the flag that will indicate that the data is saved
}

// fucntion to delete the saved data from the backup register so after the system restart it will
// work with initial values
void delete_factory_data_from_backup_registers(void)
{
    // Write the flag value to the flag register
}

// function to resotre the factory page data after reboting so the system will work with the
// saved data which is saved by the user
void restore_factory_page_data(void)
{

    // Read the packed value from the backup register

    // Unpack the offset, VFD_type, and systemPumpsNumber variables from the packed 32-bit value

    // Read the packed value from the backup register

    // Unpack the firstADCCalibrationReading and secondADCCalibrationReading variables from the packed 32-bit value

    // Read the setPointRequired variable from the backup register

    // Repeat for other float variables...

    // Read the packed flags from the backup register
    uint8_t packed_flags = read_backup_register(19);
    // Unpack the enableDryRun, enableScheduler, and enablePID flags from the single byte
}
else
{
    // do nothing and that will make the code with the inital values of the varibles
}
}

// store the current pid data is if a system reset happens it will continue from the point it stopped on
// instead of reseting all the pumps to 0 then start again
void store_internal_system_data(void)
{
    // Write the accumulated error variable to the backup register

    // Write the activePumps variable to the backup register

    // Write the value of the flag that will indicate that the data is saved
}

// restore the current pid data is if a system reset happens it will continue from the point it stopped on
// instead of reseting all the pumps to 0 then start again
void restore_internal_system_data(void)
{
    if (read_backup_register(INTERNAL_SYSTEM_DATA_FALG_INDEX) == FLAG_VALUE)
    {
    }
    else
    {
        // start pid from 0
    }
}

// Function to delete the pid current state from the backup register so it will start from the
// begging after reset or normal start
void delete_internal_system_data_from_backup_domain_registers(void)
{
    // Write the flag value to the flag register
}

// fucntion is used to save the data coming from alarm that is saved by the user and warning page of the app
// to resotre it when the system reboot
void store_alarm_data(void)
{
    // Pack the four 1-bit variables into a single 4-bit value

    // Write the packed value to the backup register
}

// function to restore the data of the alarm and warning page after restarting the board
void restore_alarm_data(void)
{
    // Read the packed value from the backup register

    // Check if the last bit is set to 1 (data stored flag)
    if ((packed_value >> 31) & 0x1)
    {
        // Unpack the four 1-bit variables from the single 4-bit value
    }
    else
    {
        // Do nothing and that will make the code with the initial values of the variables
    }
}
