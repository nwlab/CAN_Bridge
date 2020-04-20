/**
 * @file    flash.h
 * @author  Ferenc Nemeth
 * @date    21 Dec 2018
 * @brief   This module handles the memory related functions.
 *
 *          Copyright (c) 2018 Ferenc Nemeth - https://github.com/ferenc-nemeth
 */

#ifndef FLASH_H_
#define FLASH_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "bootloader-config.h"

typedef enum
{
  FLASH_FILE_OK = 0,
  FLASH_FILE_NOT_EXISTS,
  FLASH_FILE_CANNOT_OPEN,
  FLASH_FILE_INVALID_HASH,
  FLASH_FILE_TOO_BIG
} HashCheckResult;

/* Status report for the functions. */
typedef enum
{
  FLASH_OK = 0,         /**< The action was successful. */
  FLASH_ERROR,          /**< Generic error. */
  FLASH_ERROR_SIZE,     /**< The binary is too big. */
  FLASH_ERROR_WRITE,    /**< Writing failed. */
  FLASH_ERROR_READBACK, /**< Writing was successful, but the content of the memory is wrong. */
  FLASH_ERROR_FILE,
  FLASH_ERROR_FLASH
} flash_status;

flash_status flash_erase(uint32_t address);
flash_status flash_erase_size(uint32_t sector, uint32_t size);
flash_status flash_write(uint32_t address, uint32_t *data, uint32_t length);

#ifdef SDCARD_ENABLED
extern const char flashFileName[];

flash_status flash_file();
uint32_t flash_image_size();
HashCheckResult check_flash_file(uint32_t *hash, uint32_t *trueHash, uint32_t *flashFileSize);
#endif

#endif /* FLASH_H_ */
