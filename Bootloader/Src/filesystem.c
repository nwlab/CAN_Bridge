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

SD_HandleTypeDef hsd;

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
void MX_SDIO_SD_Init(void)
{
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
//  hsd.Init.ClockDiv = 3;

  hsd.Init.ClockDiv = SDIO_INIT_CLK_DIV;
}

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
