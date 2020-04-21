/*******************************************************************************
 * @file           : filesystem.c
 * @author         : Andrii Iakovenko
 * @date           : 27 Mar 2020
 * @brief          : This module handles the memory related functions.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "fatfs.h"
#include "flash.h"

#ifdef SDCARD_ENABLED
static FIL fil;
static FILINFO info;
#endif

const char flashFileName[] = FIRMWARE_FILE_NAME;
const char hashFileName[] = FIRMWARE_HASH_FILE_NAME;

uint8_t buffer[FLASH_PAGE_SIZE];
uint32_t bufferLen = 0;

/**
 * @brief   This function erases the memory.
 * @param   address: First address to be erased (the last is the end of the flash).
 * @return  status: Report about the success of the erasing.
 */
flash_status flash_erase(uint32_t sector)
{
  HAL_FLASH_Unlock();

  flash_status status = FLASH_ERROR;
  FLASH_EraseInitTypeDef erase_init;
  uint32_t error = 0u;

  erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase_init.Sector = sector;
  erase_init.Banks = FLASH_BANK_1;
  /* Calculate the number of pages from "address" and the end of flash. */
  erase_init.NbSectors = (FLASH_SECTOR_TOTAL - sector);
  /* Do the actual erasing. */
  if (HAL_OK == HAL_FLASHEx_Erase(&erase_init, &error))
  {
    status = FLASH_OK;
  }

  HAL_FLASH_Lock();

  return status;
}

flash_status flash_erase_size(uint32_t sector, uint32_t size)
{
  HAL_FLASH_Unlock();

  flash_status status = FLASH_ERROR;
  FLASH_EraseInitTypeDef erase_init;
  uint32_t error = 0u;

  erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase_init.Sector = sector;
  erase_init.Banks = FLASH_BANK_1;
  /* Calculate the number of pages from "address" and the end of flash. */
  erase_init.NbSectors = size;
  /* Do the actual erasing. */
  if (HAL_OK == HAL_FLASHEx_Erase(&erase_init, &error))
  {
    status = FLASH_OK;
  }

  HAL_FLASH_Lock();

  return status;
}

/**
 * @brief   This function flashes the memory.
 * @param   address: First address to be written to.
 * @param   *data:   Array of the data that we want to write.
 * @param   *length: Size of the array.
 * @return  status: Report about the success of the writing.
 */
flash_status flash_write(uint32_t address, uint32_t *data, uint32_t length)
{
  flash_status status = FLASH_OK;

  HAL_StatusTypeDef hal_res = HAL_FLASH_Unlock();
  if (hal_res != HAL_OK)
  {
    INFO_MSG("Unlock error: %d", hal_res);
    return FLASH_ERROR;
  }

  /* Loop through the array. */
  for (uint32_t i = 0u; i < length; i++)
  {
    /* If we reached the end of the memory, then report an error and don't do anything else.*/
    if (FLASH_APP_END_ADDRESS <= address)
    {
      status = FLASH_ERROR_SIZE;
      break;
    }
    else
    {
      /* The actual flashing. If there is an error, then report it. */
      if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i]))
      {
        status = FLASH_ERROR_WRITE;
        break;
      }
      /* Read back the content of the memory. If it is wrong, then report an error. */
      if (((data[i])) != (*(volatile uint32_t*) address))
      {
        status = FLASH_ERROR_READBACK;
        break;
      }

      /* Shift the address by a word. */
      address += 4u;
    }
  }

  HAL_FLASH_Lock();

  return status;
}

uint32_t flash_image_size()
{
  return FLASH_END - FLASH_APP_START_ADDRESS;
}

#ifdef SDCARD_ENABLED
uint32_t HashLy(uint32_t hash, const uint8_t *buf, uint32_t size)
{
  for (uint32_t i = 0; i < size; i++)
    hash = (hash * 1664525) + buf[i] + 1013904223;

  return hash;
}

uint32_t calculateFileHash(FIL *pfil)
{
  FRESULT res = f_lseek(pfil, 0);
  if (FR_OK != res)
  {
    //printf("Cannot f_lseek(pfil, 0) for hashing: %d\n", res);
    return 0;
  }
  UINT readed = 0;
  UINT blockSize = 256;
  uint8_t buf[256];
  uint32_t result = 0;

  do
  {
    res = f_read(pfil, buf, blockSize, &readed);
    if (FR_OK != res)
    {
      //printf("Cannot read file to calculate hash: %d\n", res);
      return 0;
    }
    result = HashLy(result, buf, readed);
  } while (readed == blockSize);

  res = f_lseek(pfil, 0);
  if (FR_OK != res)
  {
    //printf("Cannot f_lseek(fil, 0) after hashing: %d\n", res);
    return 0;
  }
  return result;
}

HashCheckResult check_flash_file(uint32_t *hash, uint32_t *trueHash,
    uint32_t *flashFileSize)
{
  // Testing flash file existance
  FRESULT res = f_stat(flashFileName, &info);
  if (res != FR_OK)
    return FLASH_FILE_NOT_EXISTS;

  // Checking file size
  if (info.fsize > flash_image_size())
    return FLASH_FILE_TOO_BIG;

  // Reading file with hash
  *trueHash = 0; // keep 0 if we have no hash value
  res = f_open(&fil, hashFileName, FA_OPEN_EXISTING | FA_READ);
  if (res == FR_OK)
  {
    UINT readed = 0;
    f_read(&fil, (void*) trueHash, sizeof(*trueHash), &readed);
    f_close(&fil);
    if (readed != sizeof(*trueHash))
      *trueHash = 0;
  }

  res = f_open(&fil, flashFileName, FA_OPEN_EXISTING | FA_READ);
  if (res != FR_OK)
    return FLASH_FILE_CANNOT_OPEN;

  *hash = calculateFileHash(&fil);
  f_close(&fil);
  if (*hash == 0)
    return FLASH_FILE_CANNOT_OPEN;

  if (*hash == *trueHash
#ifndef HASH_CHECK_IS_MANDATORY
      || *trueHash == 0
#endif
          )
    return FLASH_FILE_OK;
  else
    return FLASH_FILE_INVALID_HASH;
}

FRESULT readNextPage(uint8_t *target, uint32_t *readed)
{
  uint16_t blocksCount = 16;
  uint16_t fileBlock = FLASH_PAGE_SIZE / blocksCount;
  *readed = 0;
  UINT readedNow = 0;
  FRESULT res = FR_OK;
  for (uint16_t i = 0; i < blocksCount; i++)
  {
    res = f_read(&fil, target, fileBlock, &readedNow);
    target += readedNow;
    *readed += readedNow;
    if (res != FR_OK || readedNow != fileBlock)
      break;
  }
  return res;
}

flash_status flash_file()
{
  FRESULT res = f_open(&fil, flashFileName, FA_OPEN_EXISTING | FA_READ);

  if (res != FR_OK)
    return FLASH_ERROR_FILE;

  INFO_MSG("Erasing MCU sectores...");
  if (FLASH_OK != flash_erase(FLASH_APP_START_SECTOR))
  {
    INFO_MSG("Error during erasing MCU, rebooting system.");
    return FLASH_ERROR_FLASH;
  }

  uint32_t position = FLASH_APP_START_ADDRESS;

  do
  {
    readNextPage(buffer, &bufferLen);
    if (FLASH_OK != flash_write(position, (uint32_t*) &buffer[0], bufferLen))
    {
      INFO_MSG("Cannot write flash page at %lX, rebooting system.", position);
      return FLASH_ERROR_WRITE;
    }
    position += bufferLen;
  } while (bufferLen != 0);

  f_close(&fil);

  return FLASH_OK;
}
#endif
