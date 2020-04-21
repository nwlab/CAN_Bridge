/*******************************************************************************
  * @file           : bootloader-config.h
  * @author         : Andrii Iakovenko
  * @date           : 27 Mar 2020
  * @brief          :
  ******************************************************************************/
#ifndef _BOOTLOADER_CONFIG_H_
#define _BOOTLOADER_CONFIG_H_

// Comment out the following to totally disable SDCard
#define SDCARD_ENABLED
// Comment out the following to totally disable XModem protocol
#define XMODEM_ENABLED

#define FIRMWARE_FILE_NAME         "flash.bin"
#define FIRMWARE_HASH_FILE_NAME    "flash.ly"


// Uncomment this if hash check for firmware is mandatory.
// By default if hash file missing firmware considered as correct
//#define HASH_CHECK_IS_MANDATORY

// Comment out the following to totally disable UART output
#define UART_ENABLED
#define UART_BAUDRATE          115200

#define FLASH_PAGE_SIZE        2048                               //2K per page

#define FLASH_START_ADDR       (0x08000000)                        //Origin
#define FLASH_MAX_SIZE         (FLASH_END - FLASH_BASE)            //Max FLASH size - 512 kByte
#define FLASH_END_ADDR         FLASH_END                           //FLASH end address

#define FLASH_CFG_START_ADDRESS ((uint32_t)0x0800C000u)   /*Sector 3*/
#define FLASH_CFG_START_SECTOR  (FLASH_SECTOR_3)
#define FLASH_CFG_SIZE          512 /*0x4000 */
/* Start and end addresses of the user application. */
#define FLASH_APP_START_ADDRESS ((uint32_t)0x08010000u)   /*Sector 4*/
#define FLASH_APP_START_SECTOR  (FLASH_SECTOR_4)
#define FLASH_APP_END_ADDRESS   ((uint32_t)FLASH_END-0x10u) /**< Leave a little extra space at the end. */

#endif /* _BOOTLOADER_CONFIG_H_ */
