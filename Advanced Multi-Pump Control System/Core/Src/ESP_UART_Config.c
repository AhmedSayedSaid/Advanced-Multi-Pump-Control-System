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



returned_ST_Data CheckData_received(unsigned char* Data) {
    returned_ST_Data D;
    char value[MAX_DATA_LENGTH];
    char identifier[MAX_DATA_LENGTH];
    char DataLength[MAX_DATA_LENGTH];
    int checkLen;
    char firatChValue;
    char firatChIdentifier;
    short i = 0, j = 0, flag = 0, flag2 = 0;
    while (Data[i] != '\n') {
        if (Data[i] == '*' && Data[i + 1] == '?') {
            if (flag == 0) {
                strncpy(value, (const char *)(Data + j), i - j);
                value[i - j] = '\0';
                flag++;
            } else if (flag == 1) {
                strncpy(identifier, (const char *)(Data + j), i - j);
                identifier[i - j] = '\0';
                flag++;
            }
            i++;
            j = i + 1;
        } else if (Data[i] == '*' && Data[i + 1] != '?') {
            if (flag2 == 0) {
                strncpy(DataLength, (const char *)(Data + j), i - j);
                DataLength[i - j] = '\0';
                flag2++;
            } else if (flag2 == 1) {
                firatChValue = Data[j];
                flag2++;
            } else if (flag2 == 2) {
                firatChIdentifier = Data[j];
                flag2++;
            }
            j = i + 1;
        }
        i++;
    }
    int LengthData = strlen(value) + strlen(identifier);
    checkLen = atoi(DataLength);

    if (LengthData == checkLen) {
        if (value[0] == firatChValue && identifier[0] == firatChIdentifier) {
            for (i = 0; value[i] != '\0'; i++) {
                D.value[i] = value[i];
            }
            D.value[i] = '\0';
            for (i = 0; identifier[i] != '\0'; i++) {
                D.identifier[i] = identifier[i];
            }
            D.identifier[i] = '\0';
        }
    } else {
        D.value[0] = '0';
        D.value[1] = '\0';
        D.identifier[0] = '0';
        D.identifier[1] = '\0';
    }

    return D;
}


/**
  * @brief  initalize the data id and value array to avoid garbage values
  * and recive the after crc checking and zero usart1HaveNewData
  * to make it ready for new data
  *
  * @param  uartType: The type of UART to receive data from.
  * @retval The received data.
  */
returned_ST_Data ReceiveData() {
    returned_ST_Data Data;
    int i;
    for (i = 0; "null"[i] != '\0'; i++) {
        Data.identifier[i] = "null"[i];
    }
    Data.identifier[i] = '\0';
    for (i = 0; "AAAA"[i] != '\0'; i++) {
        Data.value[i] = "AAAA"[i];
    }
    Data.value[i] = '\0';

    Data = CheckData_received(getBuffer);
    usart1HaveNewData = 0;

    return Data;
}


/**
  * @brief  Prepare data to be sent over UART with the
  * value*?ID*?lenght of data +value *frist char of value*first char of ID* crc .
  * @param  value: The value to be sent.
  * @param  identifier: The identifier to be sent.
  * @retval A pointer to a buffer containing the prepared data.
  */


char* PrepareData(char* identifier, char* value) {
    static char DataSend[100];
    int DataLength = strlen(value) + strlen(identifier);
    char FirstChValue = value[0];
    char FirstChIdentifier = identifier[0];
    sprintf(DataSend, "%s*?%s*?%d*%c*%c*\n", value, identifier, DataLength, FirstChValue, FirstChIdentifier);
   // printf("%s data string \n",DataSend);
    return DataSend;
}

