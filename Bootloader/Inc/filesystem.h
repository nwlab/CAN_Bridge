/*******************************************************************************
  * @file           : filesystem.h
  * @author         : Andrii Iakovenko
  * @date           : 27 Mar 2020
  * @brief          : Header for filesystem.с file.
  ******************************************************************************/
#ifndef _FILESYSTEM_H_
#define _FILESYSTEM_H_

#include <stdint.h>

#define FILESYSTEM_INIT_OK      0
#define FILESYSTEM_INIT_FAIL    1

uint8_t init_filesystem();
void deinit_filesystem();

#endif /* _FILESYSTEM_H_ */
