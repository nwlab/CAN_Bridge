#ifndef BOOTLOADER_USB_BOOTLOADER_USB_INC_BOOTLOADER_CONFIG_H_
#define BOOTLOADER_USB_BOOTLOADER_USB_INC_BOOTLOADER_CONFIG_H_

#define FIRMWARE_FILE_NAME         "flash.bin"
#define FIRMWARE_HASH_FILE_NAME    "flash.ly"


// Uncomment this if hash check for firmware is mandatory.
// By default if hash file missing firmware considered as correct
//#define HASH_CHECK_IS_MANDATORY


// Comment out the following to totally disable UART output
#define UART_ENABLED
#define UART_BAUDRATE        921600



#define USB_WAITING_PERIOD         1000

//#define SIZE_OF_U32            4                                   //sizeof(u32)
//#define FLASH_PAGE_SIZE         2048                               //2K per page

#define FLASH_START_ADDR       (0x08000000)                        //Origin
#define FLASH_MAX_SIZE         (FLASH_END - FLASH_BASE)            //Max FLASH size - 512 kByte
#define FLASH_END_ADDR         FLASH_END                           //FLASH end address

#define FLASH_BOOT_START_ADDR   (FLASH_START_ADDR)                 //Bootloader start address
#define FLASH_BOOT_SIZE         (0x00010000)                       //64 kByte for bootloader
#define FLASH_USER_START_ADDR   (FLASH_BOOT_START_ADDR + FLASH_BOOT_SIZE)   //User application start address
#define FLASH_USER_SIZE         (0x00032000)                       //200kByte for usir application
#define FLASH_MSD_START_ADDR   (FLASH_USER_START_ADDR + FLASH_USER_SIZE)   //USB MSD stort address
#define FLASH_MSD_SIZE         (0x00032000)                        //200kByte for USB MASS Storage

/* Start and end addresses of the user application. */
#define FLASH_APP_START_ADDRESS ((uint32_t)0x08008000u)   /*Sector 2*/
#define FLASH_APP_START_SECTOR  (FLASH_SECTOR_2)
#define FLASH_APP_END_ADDRESS   ((uint32_t)FLASH_END-0x10u) /**< Leave a little extra space at the end. */

#endif /* BOOTLOADER_USB_BOOTLOADER_USB_INC_BOOTLOADER_CONFIG_H_ */
