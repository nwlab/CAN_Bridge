/*******************************************************************************
  * @file           : filesystem.c
  * @author         : Andrii Iakovenko
  * @date           : 27 Mar 2020
  * @brief          :
  ******************************************************************************/
#include "main.h"
#include "filesystem.h"
#include "fatfs.h"

static FATFS fatfs;

extern SD_HandleTypeDef hsd;

void MX_SDIO_SD_Init(void);

uint8_t init_filesystem()
{
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	FRESULT res = f_mount(&fatfs, "", 1);
	if (res != FR_OK)
	{
		return FILESYSTEM_INIT_FAIL;
	}
	return FILESYSTEM_INIT_OK;
}

void deinit_filesystem()
{
	f_mount(&fatfs, "", 0);
	HAL_SD_DeInit(&hsd);
}
