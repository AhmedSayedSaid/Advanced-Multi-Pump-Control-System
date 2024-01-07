/**
 ******************************************************************************
 * @file    ESP_UART_Config.c
 * @brief   Source file for ESP_UART_Config.
 ******************************************************************************
 * @attention
 *
 * This file contains the implementation of the ESP UART library.
 *
 ******************************************************************************
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

/* Includes ------------------------------------------------------------------*/
#include "ESP_UART_Config.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  Check the received data which it matched the this formula of crc or not
 * value*?ID*?lenght of data +value *frist char of value*first char of ID*.
 * @param  Data: The received data.
 * @retval the extracted data and id in case the coming data matched the format of crc.
 */

returned_ST_Data CheckData_received(unsigned char *Data)
{
}

/**
 * @brief  initalize the data id and value array to avoid garbage values
 * and recive the after crc checking and zero usart1HaveNewData
 * to make it ready for new data
 *
 * @param  uartType: The type of UART to receive data from.
 * @retval The received data.
 */
returned_ST_Data ReceiveData()
{
}

/**
 * @brief  Prepare data to be sent over UART with the
 * value*?ID*?lenght of data +value *frist char of value*first char of ID* crc .
 * @param  value: The value to be sent.
 * @param  identifier: The identifier to be sent.
 * @retval A pointer to a buffer containing the prepared data.
 */

char *PrepareData(char *identifier, char *value)
{
}
