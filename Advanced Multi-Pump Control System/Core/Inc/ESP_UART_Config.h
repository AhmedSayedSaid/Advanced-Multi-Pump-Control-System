/**
  ******************************************************************************
  * @file    ESP_UART_Config.h
  * @brief   Header file for ESP_UART_Config.
  ******************************************************************************
  * @attention
  *
  * This file contains the configuration api for the ESP STM UART connection.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ESP_UART_CONFIG_H
#define __ESP_UART_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
#define MAX_DATA_LENGTH 70

typedef struct {
    char value[MAX_DATA_LENGTH];
    char identifier[MAX_DATA_LENGTH];
} returned_ST_Data;
extern volatile uint8_t usart1HaveNewData ;

extern uint8_t getBuffer[128];
#include <stdio.h>
#include <stdlib.h>
#include <string.h> /* memset */
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Checks and extracts data from a received USART string based on a CRC-like format.
 *
 * This function parses the received data to extract 'value' and 'identifier' parts if they match
 * the expected format: 'value*?ID*?length of data + value*first char of value*first char of ID*'.
 * It validates the format by comparing the actual and expected lengths and the first characters.
 *
 * @param Data Pointer to the received data string.
 * @return The extracted data and identifier in a returned_ST_Data struct if the data matches the format.
 *         Returns default values ('0' for both fields) if the data doesn't match.
 *
 * Preconditions:
 * - The Data pointer must not be NULL and should point to a null-terminated string.
 *
 * Postconditions:
 * - Returns a struct with extracted 'value' and 'identifier' if format matches, or default values otherwise.
 */
returned_ST_Data CheckData_received(unsigned char* Data);

/**
 * @brief Initializes data fields and processes received USART data.
 *
 * Initializes the identifier and value fields of the returned_ST_Data structure to avoid garbage values.
 * It then calls CheckData_received to process the data received via USART and zeroes the flag 'usart1HaveNewData'.
 *
 * @return The processed data in a returned_ST_Data struct after CRC check.
 *
 * Preconditions:
 * - USART data buffer (getBuffer) should be ready with the received data.
 * - usart1HaveNewData flag should be set to indicate new data is available.
 *
 * Postconditions:
 * - Returns a returned_ST_Data structure with processed data.
 * - Resets the usart1HaveNewData flag.
 */
returned_ST_Data ReceiveData(void);

/**
 * @brief Prepares data to be sent over USART with a specific CRC-like format.
 *
 * Formats the given 'value' and 'identifier' into a string following the format:
 * 'value*?ID*?length of data + value*first char of value*first char of ID*'.
 * The formatted string is intended for transmission over USART.
 *
 * @param identifier Pointer to the identifier string.
 * @param value Pointer to the value string.
 * @return Pointer to a static buffer containing the formatted data.
 *
 * Preconditions:
 * - 'identifier' and 'value' should not be NULL and must be null-terminated strings.
 *
 * Postconditions:
 * - Returns a pointer to a statically allocated string containing the formatted data.
 * - The returned pointer points to a static buffer, so it should be used or copied before the next call to PrepareData.
 */
char* PrepareData(char* identifier, char* value);



#ifdef __cplusplus
}
#endif

#endif /* __ESP_UART_CONFIG_H */
