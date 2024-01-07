/*
 * Modbus.c
 *  Modbus RTU Master and Slave library for STM32 CUBE with FreeRTOS
 *  Created on: May 5, 2020
 *      Author: Alejandro Mera
 *      Adapted from https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "Modbus.h"
#include "timers.h"
#include "semphr.h"

#if ENABLE_TCP == 1
#include "api.h"
#include "ip4_addr.h"
#include "netif.h"
#endif

#ifndef ENABLE_USART_DMA
#define ENABLE_USART_DMA 0
#endif

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

#define lowByte(w) ((w) & 0xff)
#define highByte(w) ((w) >> 8)

modbusHandler_t *mHandlers[MAX_M_HANDLERS];

/// Queue Modbus telegrams for master
const osMessageQueueAttr_t QueueTelegram_attributes = {
	.name = "QueueModbusTelegram"};

const osThreadAttr_t myTaskModbusA_attributes = {
	.name = "TaskModbusSlave",
	.priority = (osPriority_t)osPriorityNormal,
	.stack_size = 128 * 4};

const osThreadAttr_t myTaskModbusA_attributesTCP = {
	.name = "TaskModbusSlave",
	.priority = (osPriority_t)osPriorityNormal,
	.stack_size = 256 * 4};

// Task Modbus Master
// osThreadId_t myTaskModbusAHandle;
const osThreadAttr_t myTaskModbusB_attributes = {
	.name = "TaskModbusMaster",
	.priority = (osPriority_t)osPriorityNormal,
	.stack_size = 128 * 4};

const osThreadAttr_t myTaskModbusB_attributesTCP = {
	.name = "TaskModbusMaster",
	.priority = (osPriority_t)osPriorityNormal,
	.stack_size = 256 * 4};

// Semaphore to access the Modbus Data
const osSemaphoreAttr_t ModBusSphr_attributes = {
	.name = "ModBusSphr"};

uint8_t numberHandlers = 0;

static void sendTxBuffer(modbusHandler_t *modH);
static int16_t getRxBuffer(modbusHandler_t *modH);
static uint8_t validateAnswer(modbusHandler_t *modH);
static void buildException(uint8_t u8exception, modbusHandler_t *modH);
static uint8_t validateRequest(modbusHandler_t *modH);
static uint16_t word(uint8_t H, uint8_t l);
static void get_FC1(modbusHandler_t *modH);
static void get_FC3(modbusHandler_t *modH);
static int8_t process_FC1(modbusHandler_t *modH);
static int8_t process_FC3(modbusHandler_t *modH);
static int8_t process_FC5(modbusHandler_t *modH);
static int8_t process_FC6(modbusHandler_t *modH);
static int8_t process_FC15(modbusHandler_t *modH);
static int8_t process_FC16(modbusHandler_t *modH);
static void vTimerCallbackT35(TimerHandle_t *pxTimer);
static void vTimerCallbackTimeout(TimerHandle_t *pxTimer);
// static int16_t getRxBuffer(modbusHandler_t *modH);
static int8_t SendQuery(modbusHandler_t *modH, modbus_t telegram);

#if ENABLE_TCP == 1

static bool TCPwaitConnData(modbusHandler_t *modH);
static void TCPinitserver(modbusHandler_t *modH);
static mb_errot_t TCPconnectserver(modbusHandler_t *modH, modbus_t *telegram);
static mb_errot_t TCPgetRxBuffer(modbusHandler_t *modH);

#endif

/* Ring Buffer functions */
// This function must be called only after disabling USART RX interrupt or inside of the RX interrupt
void RingAdd(modbusRingBuffer_t *xRingBuffer, uint8_t u8Val)
{
}

// This function must be called only after disabling USART RX interrupt
uint8_t RingGetAllBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer)
{
}

// This function must be called only after disabling USART RX interrupt
uint8_t RingGetNBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer, uint8_t uNumber)
{
}

/* End of Ring Buffer functions */

const unsigned char fctsupported[] =
	{

};

/**
 * @brief
 * Initialization for a Master/Slave.
 * this function will check the configuration parameters
 * of the modbus handler
 *
 * @param modH   modbus handler
 */
void ModbusInit(modbusHandler_t *modH)
{
}

/**
 * @brief
 * Start object.
 *
 * Call this AFTER calling begin() on the serial port, typically within setup().
 *
 * (If you call this function, then you should NOT call any of
 * ModbusRtu's own begin() functions.)
 *
 * @ingroup setup
 */
void ModbusStart(modbusHandler_t *modH)
{
}

void vTimerCallbackTimeout(TimerHandle_t *pxTimer)
{
}

#if ENABLE_TCP == 1

bool TCPwaitConnData(modbusHandler_t *modH)
{
}

void TCPinitserver(modbusHandler_t *modH)
{
}

#endif

void StartTaskModbusSlave(void *argument)
{
}

void ModbusQuery(modbusHandler_t *modH, modbus_t telegram)
{
}

void ModbusQueryInject(modbusHandler_t *modH, modbus_t telegram)
{
}

#if ENABLE_TCP == 1
void ModbusCloseConn(struct netconn *conn)
{
}

#if ENABLE_TCP == 1

static mb_errot_t TCPconnectserver(modbusHandler_t *modH, modbus_t *telegram)
{
}

static mb_errot_t TCPgetRxBuffer(modbusHandler_t *modH)
{
}

#endif

void StartTaskModbusMaster(void *argument)
{
}

/**
 * This method processes functions 1 & 2 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void get_FC1(modbusHandler_t *modH)
{
}

/**
 * This method processes functions 3 & 4 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void get_FC3(modbusHandler_t *modH)
{
}

/**
 * @brief
 * This method validates master incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
uint8_t validateAnswer(modbusHandler_t *modH)
{
	// check message crc vs calculated crc
}

/**
 * @brief
 * This method moves Serial buffer data to the Modbus u8Buffer.
 *
 * @return buffer size if OK, ERR_BUFF_OVERFLOW if u8BufferSize >= MAX_BUFFER
 * @ingroup buffer
 */
int16_t getRxBuffer(modbusHandler_t *modH)
{
}

/**
 * @brief
 * This method validates slave incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup modH Modbus handler
 */
uint8_t validateRequest(modbusHandler_t *modH){exception code thrown

}

/**
 * @brief
 * This method creates a word from 2 bytes
 *
 * @return uint16_t (word)
 * @ingroup H  Most significant byte
 * @ingroup L  Less significant byte
 */
uint16_t word(uint8_t H, uint8_t L)
{
}

/**
 * @brief
 * This method calculates CRC
 *
 * @return uint16_t calculated CRC value for the message
 * @ingroup Buffer
 * @ingroup u8length
 */
uint16_t calcCRC(uint8_t *Buffer, uint8_t u8length)
{
}

/**
 * @brief
 * This method builds an exception message
 *
 * @ingroup u8exception exception number
 * @ingroup modH modbus handler
 */
void buildException(uint8_t u8exception, modbusHandler_t *modH)
{
}

/**
 * @brief
 * This method transmits u8Buffer to Serial line.
 * Only if u8txenpin != 0, there is a flow handling in order to keep
 * the RS485 transceiver in output state as long as the message is being sent.
 * This is done with TC bit.
 * The CRC is appended to the buffer before starting to send it.
 *
 * @return nothing
 * @ingroup modH Modbus handler
 */
static void sendTxBuffer(modbusHandler_t *modH)
{
}

/**
 * @brief
 * This method processes functions 1 & 2
 * This method reads a bit array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
int8_t process_FC1(modbusHandler_t *modH)
{
}

/**
 * @brief
 * This method processes functions 3 & 4
 * This method reads a word array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t process_FC3(modbusHandler_t *modH)
{
}

/**
 * @brief
 * This method processes function 5
 * This method writes a value assigned by the master to a single bit
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
int8_t process_FC5(modbusHandler_t *modH)
{
}

/**
 * @brief
 * This method processes function 6
 * This method writes a value assigned by the master to a single word
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t process_FC6(modbusHandler_t *modH)
{
}

/**
 * @brief
 * This method processes function 15
 * This method writes a bit array assigned by the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
int8_t process_FC15(modbusHandler_t *modH)
{
}

/**
 * @brief
 * This method processes function 16
 * This method writes a word array assigned by the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t process_FC16(modbusHandler_t *modH)
{
}
