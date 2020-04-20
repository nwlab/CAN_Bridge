/*******************************************************************************
  * @file           : cardreader.c
  * @author         : Andrii Iakovenko
  * @date           : 27 Mar 2020
  * @brief          :
  ******************************************************************************/
#include "main.h"
#include "bsp_driver_sd.h"
#include "usb_device.h"
#include "usbd_core.h"

extern HAL_SD_CardInfoTypeDef SDCardInfo;
extern SD_HandleTypeDef hsd;
extern USBD_HandleTypeDef hUsbDeviceFS;

void MX_SDIO_SD_Init(void);

void init_cardreader()
{
	MX_SDIO_SD_Init();
	BSP_SD_Init();
	MX_USB_DEVICE_MSC_Init();
#if 0
    HAL_SD_GetCardInfo(&hsd, &SDCardInfo);
    if (SDCardInfo.CardType == CARD_SDHC_SDXC)
    {
      INFO_MSG("CardType    : 0x%08lx, SD High Capacity <32Go, SD Extended Capacity <2To", SDCardInfo.CardType);
    }
    else if (SDCardInfo.CardType == CARD_SDSC)
    {
      INFO_MSG("CardType    : 0x%08lx, SD Standard Capacity <2Go", SDCardInfo.CardType);
    }
    else if (SDCardInfo.CardType == CARD_SECURED)
    {
      INFO_MSG("CardType    : 0x%08lx, CARD_SECURED", SDCardInfo.CardType);
    }
    else
    {
      INFO_MSG("CardType    : 0x%08lx", SDCardInfo.CardType);
    }
    INFO_MSG("CardVersion : 0x%08lx", SDCardInfo.CardVersion);
    INFO_MSG("Class       : 0x%08lx", SDCardInfo.Class);
    INFO_MSG("RelCardAdd  : 0x%08lx", SDCardInfo.RelCardAdd);
    INFO_MSG("BlockNbr    : %ld", SDCardInfo.BlockNbr);
    INFO_MSG("BlockSize   : %ld", SDCardInfo.BlockSize);
    INFO_MSG("LogBlockNbr : %ld", SDCardInfo.LogBlockNbr);
    INFO_MSG("LogBlockSize: %ld", SDCardInfo.LogBlockSize);
#endif
}

void deinit_cardreader()
{
	/* Stop the low level driver  */
	USBD_LL_Stop(&hUsbDeviceFS);

	/* Initialize low level driver */
	USBD_LL_DeInit(&hUsbDeviceFS);
}

