/*******************************************************************************
  * @file           : filesystem.h
  * @author         : Andrii Iakovenko
  * @date           : 27 Mar 2020
  * @brief          : Header for filesystem.с file.
  ******************************************************************************/
#ifndef _FILESYSTEM_H_
#define _FILESYSTEM_H_

#include <stdint.h>

#include "fatfs.h"

#define FILESYSTEM_INIT_OK      0
#define FILESYSTEM_INIT_FAIL    1

uint8_t init_filesystem();
void deinit_filesystem();

FIL * fopen_( const char * fileName, const char *mode );
int fclose_(FIL   *fo);
size_t fwrite_(const void *data_to_write, size_t size, size_t n, FIL *stream);
size_t fread_(void *ptr, size_t size, size_t n, FIL *stream);

#endif /* _FILESYSTEM_H_ */
