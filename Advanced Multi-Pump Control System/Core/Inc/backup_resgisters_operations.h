/*
 * backup_resgisters_operations.h
 *
 *  Created on: Sep 10, 2023
 *      Author: ahmed elsayed
 */

#ifndef INC_BACKUP_RESGISTERS_OPERATIONS_H_
#define INC_BACKUP_RESGISTERS_OPERATIONS_H_

// varible of the pages of the app that will be stored and resotred from the backregister
extern float setPointRequired;
extern uint16_t offset;
extern uint8_t VFD_type;
extern uint8_t systemPumpsNumber;
extern uint16_t firstADCCalibrationReading;
extern uint16_t secondADCCalibrationReading;
extern float Kp;
extern float Ki;
extern float Kd;
extern float currentOfTheFirstReading;
extern float currentOfTheSecondReading;
extern float minimumPressure;
extern float MaximumPressure;
extern uint8_t enableDryRun;
extern uint8_t enableScheduler;
extern uint8_t enablePID;
extern float coffA;
extern float coffB;
extern float cutIn;
extern float cutOff;

extern float accumulatedError;
extern uint8_t prevActivePumps;
extern uint8_t dryRunSysStop;
extern uint8_t pressureAlarmEnable;
extern uint8_t pressureSysStop;


/**
 * @brief Deletes pump data from backup registers.
 *
 * Writes a specific flag value (DELETED_FLAG_VALUE) to the pump's flag register to indicate that the pump data has been logically deleted.
 *
 * Preconditions:
 * - Backup registers must be initialized and accessible.
 *
 * Postconditions:
 * - The pump data in the backup registers is marked as deleted.
 */
void delete_pump_data_from_backup_registers(void);

/**
 * @brief Deletes factory settings data from backup registers.
 *
 * Writes a specific flag value (DELETED_FLAG_VALUE) to the factory settings flag register to indicate that the factory data has been logically deleted.
 *
 * Preconditions:
 * - Backup registers must be initialized and accessible.
 *
 * Postconditions:
 * - The factory settings data in the backup registers is marked as deleted.
 */
void delete_factory_data_from_backup_registers(void);

/**
 * @brief Restores factory page data from backup registers after a system reboot.
 *
 * Reads the factory settings data from the backup registers and updates the system's configuration accordingly if the data is valid.
 *
 * Preconditions:
 * - Backup registers must be initialized and contain valid factory settings data.
 *
 * Postconditions:
 * - The system configuration is updated with the factory settings data from the backup registers.
 */
void restore_factory_page_data(void);

/**
 * @brief Stores factory page data to backup registers.
 *
 * Packs and writes various factory settings data, such as offsets, types, and system pump numbers, into the backup registers.
 *
 * Preconditions:
 * - The relevant factory settings variables must be initialized and hold valid data.
 *
 * Postconditions:
 * - Factory settings data is written to the backup registers.
 */
void store_factory_page_data(void);

/**
 * @brief Restores pump data from backup registers.
 *
 * Reads pump data from the backup registers and updates the pumpInfoArray if the data is valid.
 *
 * Preconditions:
 * - Backup registers must be initialized and contain valid pump data.
 *
 * Postconditions:
 * - pumpInfoArray is updated with the data read from the backup registers.
 */
void restore_pump_data(void);

/**
 * @brief Writes pump data to backup registers.
 *
 * Iterates through the pumpInfoArray and writes the data of each pump, including id, workingHours,
 * and workingMinutes, into the backup registers.
 *
 * Preconditions:
 * - pumpInfoArray must be initialized and populated with valid data.
 *
 * Postconditions:
 * - Pump data is written to the backup registers.
 */
void write_pump_data_to_backup_registers(void);

/**
 * @brief Stores internal system data (PID settings) to backup registers.
 *
 * Writes the current PID control system data, such as accumulated error and active pumps, to the backup registers.
 *
 * Preconditions:
 * - PID control system data must be initialized and hold valid data.
 *
 * Postconditions:
 * - PID control system data is written to the backup registers.
 */
void store_internal_system_data(void);

/**
 * @brief Restores internal system data (PID settings) from backup registers.
 *
 * Reads the PID control system data from the backup registers and updates the system's PID configuration if the data is valid.
 *
 * Preconditions:
 * - Backup registers must be initialized and contain valid PID control system data.
 *
 * Postconditions:
 * - The system's PID configuration is updated with the data read from the backup registers.
 */
void restore_internal_system_data(void);

/**
 * @brief Deletes internal system data (PID settings) from backup registers.
 *
 * Writes a specific flag value (DELETED_FLAG_VALUE) to the PID data flag register to indicate that the PID data has been logically deleted.
 *
 * Preconditions:
 * - Backup registers must be initialized and accessible.
 *
 * Postconditions:
 * - The PID data in the backup registers is marked as deleted.
 */
void delete_internal_system_data_from_backup_domain_registers(void);

/**
 * @brief Stores alarm configuration data to backup registers.
 *
 * Writes the current alarm configuration, such as enabled states and system stop settings, to the backup registers.
 *
 * Preconditions:
 * - Alarm configuration data must be initialized and hold valid data.
 *
 * Postconditions:
 * - Alarm configuration data is written to the backup registers.
 */
void store_alarm_data(void);

/**
 * @brief Writes a value to a specified backup register.
 *
 * Writes the given value to the specified backup register index after enabling access to the backup domain.
 *
 * @param register_index Index of the register to write to.
 * @param value Value to be written.
 *
 * Preconditions:
 * - Backup register at the given index must be initialized and accessible.
 *
 * Postconditions:
 * - The specified backup register contains the provided value.
 */
void write_backup_register(uint32_t register_index, uint32_t value);

/**
 * @brief Reads a value from a specified backup register.
 *
 * Reads and returns the value from the specified backup register index after enabling access to the backup domain.
 *
 * @param register_index Index of the register to read from.
 * @return uint32_t Value read from the register.
 *
 * Preconditions:
 * - Backup register at the given index must be initialized and accessible.
 *
 * Postconditions:
 * - Returns the value from the specified backup register.
 */
uint32_t read_backup_register(uint32_t register_index);

/**
 * @brief Restores alarm configuration data from backup registers.
 *
 * Reads the alarm configuration data from the backup registers and updates the system's alarm configuration if the data is valid.
 *
 * Preconditions:
 * - Backup registers must be initialized and contain valid alarm configuration data.
 *
 * Postconditions:
 * - The system's alarm configuration is updated with the data read from the backup registers.
 */
void restore_alarm_data(void);


#endif /* INC_BACKUP_RESGISTERS_OPERATIONS_H_ */
