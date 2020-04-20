/*******************************************************************************
  * @file           : fatfs.h
  * @author         : Andrii Iakovenko
  * @date           : 27 Mar 2020
  * @brief          : Header for fatfs applications
  ******************************************************************************/
#ifndef __fatfs_H
#define __fatfs_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h" /* defines SD_Driver as external */

extern uint8_t retSD; /* Return value for SD */
extern char SDPath[4]; /* SD logical drive path */
//extern FATFS SDFatFS; /* File system object for SD logical drive */
extern FIL SDFile; /* File object for SD */

void MX_FATFS_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__fatfs_H */
