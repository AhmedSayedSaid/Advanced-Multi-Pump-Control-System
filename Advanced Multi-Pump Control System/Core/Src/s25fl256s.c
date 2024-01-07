/**
 ******************************************************************************
 * @file    s25fl256s.c
 * @author  MCD Application Team
 * @brief   This file provides the S25FL256S QSPI drivers.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "s25fl256s.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup Components
 * @{
 */

/** @defgroup S25FL256S S25FL256S
 * @{
 */

/** @defgroup S25FL256S_Private_Define S25FL256S Private Define
 * @{
 */
/* To avoid compiling issues for projects using previous version */
#ifndef S25FL256S_DUMMY_CYCLES_READ_DUAL_INOUT
#define S25FL256S_DUMMY_CYCLES_READ_DUAL_INOUT 4U
#endif

#ifndef S25FL256S_DUMMY_CYCLES_READ_QUAD_INOUT
#define S25FL256S_DUMMY_CYCLES_READ_QUAD_INOUT 6U
#endif
/**
 * @}
 */

/** @defgroup S25FL256S_Exported_Functions S25FL256S Exported Functions
 * @{
 */

/**
 * @brief  Get Flash information
 * @param  pInfo pointer to information structure
 * @retval QSPI memory status
 */
int32_t S25FL256S_GetFlashInfo(S25FL256S_Info_t *pInfo)
{
  /* Configure the structure with the memory configuration */
}

/**
 * @brief  This function send a Write Enable and wait it is effective.
 * @param  Ctx Component object pointer
 * @param  Mode Interface mode
 * @retval QSPI memory status
 */
int32_t S25FL256S_WriteEnable(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  This function reset the (WEL) Write Enable Latch bit.
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @retval QSPI memory status
 */
int32_t S25FL256S_WriteDisable(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  Writes an amount of data to the QSPI memory.
 *         SPI/QPI; 1-1-1/1-1-4
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @param  pData Pointer to data to be written
 * @param  WriteAddr Write start address
 * @param  Size Size of data to write. Range 1 ~ 256
 * @retval QSPI memory status
 */
int32_t S25FL256S_PageProgram(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode, uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
}

/**
 * @brief  Polling WIP(Write In Progress) bit become to 0
 * @param  Ctx Component object pointer
 * @param  Mode Interface mode
 * @retval QSPI memory status
 */
int32_t S25FL256S_AutoPollingMemReady(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  This function set the QSPI memory in 4-byte address mode (only old type commands)
 * @param  Ctx Component object pointer
 * @param  Mode Interface mode
 * @retval QSPI memory status
 */
int32_t S25FL256S_Enter4BytesAddressMode(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  This function set the QSPI memory in 4-lines data mode
 * @param  Ctx Component object pointer
 * @param  Mode Interface mode
 * @retval QSPI memory status
 */
int32_t S25FL256S_Enter4LinesDataMode(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  Reads an amount of data from the QSPI memory in STR mode.
 *         SPI/DPI/QPI; 1-1-1/1-1-2/1-2-2/1-1-4/1-4-4
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @param  pData Pointer to data to be read
 * @param  ReadAddr Read start address
 * @param  Size Size of data to read
 * @retval QSPI memory status
 */
int32_t S25FL256S_ReadSTR(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
}

/**
 * @brief  Erases the specified block of the QSPI memory.
 *         S25FL256S support 4K, 64K size block erase commands.
 *         SPI; 1-0-0/1-1-0
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @param  BlockAddress Block address to erase
 * @param  BlockSize 4K or 64K
 * @retval QSPI memory status
 */
int32_t S25FL256S_BlockErase(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode, uint32_t BlockAddress, S25FL256S_Erase_t BlockSize)
{
}

/**
 * @brief  Whole chip erase.
 *         SPI; 1-0-0
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @retval QSPI memory status
 */
int32_t S25FL256S_ChipErase(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  Reads an amount of data from the QSPI memory on STR mode.
 *         SPI/DPI/QPI; 1-1-1/1-1-2/1-2-2/1-1-4/1-4-4
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @retval QSPI memory status
 */
int32_t S25FL256S_EnableMemoryMappedModeSTR(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  Read Flash Status register value
 *         SPI; 1-0-1
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @param  Value to read from status register
 */
int32_t S25FL256S_ReadStatusRegister(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode, uint8_t *Value)
{
}

/**
 * @brief  Read Flash Config register value
 *         SPI; 1-0-1
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @param  Value to read from status register
 */
int32_t S25FL256S_ReadConfigRegister(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode, uint8_t *Value)
{
}

int32_t S25FL256S_ResetMemory(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  Read Flash 3 Byte IDs.
 *         Manufacturer ID, Memory type, Memory density
 *         SPI; 1-0-1
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @param  ID  Flash ID
 * @retval QSPI memory status
 */
int32_t S25FL256S_ReadID(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode, uint8_t *ID)
{
}

/**
 * @brief  Program/Erases suspend. Interruption Program/Erase operations.
 *         After the device has entered Erase-Suspended mode,
 *         system can read any address except the block/sector being Program/Erased.
 *         SPI; 1-0-0
 * @param  Ctx QSPI handle
 * @param  Mode Flash moder
 * @retval QSPI memory status
 */
int32_t S25FL256S_ProgEraseSuspend(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  Program/Erases resume.
 *         SPI; 1-0-0
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @retval QSPI memory status
 */
int32_t S25FL256S_ProgEraseResume(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  Enter deep sleep
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @retval QSPI memory status
 */
int32_t S25FL256S_EnterDeepPowerDown(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode)
{
}

/**
 * @brief  Reads an amount of SFDP data from the QSPI memory.
 *         SFDP : Serial Flash Discoverable Parameter
 * @param  Ctx QSPI handle
 * @param  Mode Flash mode
 * @param  pData Pointer to data to be read
 *         ReadAddr Read start address
 *         Size Size of data to read in Byte
 * @retval QSPI memory status
 */
int32_t S25FL256S_ReadSFDP(QSPI_HandleTypeDef *Ctx, S25FL256S_Interface_t Mode, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
