/*
 * logger.c
 *
 *  Created on: May 3, 2020
 *      Author: nightworker
 */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "main.h"
#include "rtc.h"
#include "filesystem.h"
#include "nvs.h"

#define SD_WRITE_BUFFER             (1024*32)//(1024*21)   // 21K
#define SD_WRITE_BUFFER_FLUSH_LIMIT (1024*31)//(1024*20)   // 20K

// buffer for collecting data to write
char sd_buffer[SD_WRITE_BUFFER];
unsigned short int sd_buffer_length = 0;
// buffer for storing ready to write data
char sd_buffer_for_write[SD_WRITE_BUFFER];

unsigned char bReqWrite = 0; // write request, the sd_buffer is being copied to sd_buffer_for_write
unsigned short int sd_buffer_length_for_write = 0;

unsigned char bWriteFault = 0; // in case of overlap or write fault
extern unsigned char bLogging; // if =1 than we logging to SD card

// fill buffer with spaces (before \r\n) to make it 512 byte size
// return 1 if filled and ready to write
int align_buffer()
{
  int i;
  int len;

  if (sd_buffer_length < 2) return 0;
  if (sd_buffer[sd_buffer_length-2] != '\r') return 0;
  if (sd_buffer[sd_buffer_length-1] != '\n') return 0;

  len = BLOCKSIZE - (sd_buffer_length % BLOCKSIZE);
  for (i = 0; i < len; i++)
    sd_buffer[sd_buffer_length + i - 2] = ' ';
  sd_buffer[sd_buffer_length - 2] = ',';
  sd_buffer[sd_buffer_length + len - 2] = '\r';
  sd_buffer[sd_buffer_length + len - 1] = '\n';

  sd_buffer_length += len;

  return 1;
}

// copy input buffer into the buffer for flash writing data
void copy_buffer()
{
  // request write operation
  memcpy(sd_buffer_for_write, sd_buffer, sd_buffer_length);
  sd_buffer_length_for_write = sd_buffer_length;
  sd_buffer_length = 0;
}

void request_write()
{
  if (bReqWrite)
    bWriteFault = 1; // buffer overlapping

  // request write operation
  align_buffer();
  copy_buffer();
  bReqWrite = 1;
}

int iLastWriteSecond = 0;
static struct tm timp;

void fwrite_string(char *pString)
{
  unsigned short int length = strlen(pString);

  // Add string
  memcpy(&sd_buffer[sd_buffer_length], pString, length);
  sd_buffer_length += length;

  // Check flush limit
  if(sd_buffer_length >= SD_WRITE_BUFFER_FLUSH_LIMIT)
  {
    request_write();
  }
}

// file writing
FATFS SDC_FS;
FIL *file;
FRESULT fres;

int i;
int iSecond;
#define STRLINE_LENGTH 1024
char sLine[STRLINE_LENGTH];
uint32_t stLastWriting;
unsigned char bIncludeTimestamp = 1;
int iFilterMask = 0;
int iFilterValue = 0;
unsigned char bLogStdMsgs = 1;
unsigned char bLogExtMsgs = 1;
char sTmp[128];

void read_config_file()
{
  uint16_t size = 0;

  if (nvs_get("log_ext", (uint8_t*)&bLogExtMsgs, &size, sizeof(bLogExtMsgs)) == KEY_NOT_FOUND ||
      nvs_get("log_std", (uint8_t*)&bLogStdMsgs, &size, sizeof(bLogStdMsgs)) == KEY_NOT_FOUND ||
      nvs_get("log_time", (uint8_t*)&bIncludeTimestamp, &size, sizeof(bIncludeTimestamp)) == KEY_NOT_FOUND ||
      nvs_get("log_idfv", (uint8_t*)&iFilterValue, &size, sizeof(iFilterValue)) == KEY_NOT_FOUND ||
      nvs_get("log_idfm", (uint8_t*)&iFilterMask, &size, sizeof(iFilterMask)) == KEY_NOT_FOUND)
  {
    DEBUG_MSG("No LOG parameters in NVS, Set to default");
    iFilterMask = 0;
    iFilterValue = 0;
    bIncludeTimestamp = 1;
    bLogStdMsgs = 1;
    bLogExtMsgs = 1;
    nvs_put("log_ext", (uint8_t*)&bLogExtMsgs, sizeof(bLogExtMsgs), sizeof(bLogExtMsgs));
    nvs_put("log_std", (uint8_t*)&bLogStdMsgs, sizeof(bLogStdMsgs), sizeof(bLogStdMsgs));
    nvs_put("log_time", (uint8_t*)&bIncludeTimestamp, sizeof(bIncludeTimestamp), sizeof(bIncludeTimestamp));
    nvs_put("log_idfv", (uint8_t*)&iFilterValue, sizeof(iFilterValue), sizeof(iFilterValue));
    nvs_put("log_idfm", (uint8_t*)&iFilterMask, sizeof(iFilterMask), sizeof(iFilterMask));
    if (nvs_commit() != NVS_OK)
    {
      DEBUG_MSG("Flash commit failed");
    }
  }
}

void start_log()
{
  // open file and write the begining of the load
  RTC_GetTime(&hrtc, &timp);
  sprintf(sLine, "%04d-%02d-%02dT%02d-%02d-%02dZ.csv", timp.tm_year + 1900, timp.tm_mon, timp.tm_mday, timp.tm_hour, timp.tm_min, timp.tm_sec); // making new file

  DEBUG_MSG("Open file %s", sLine);

  file = fopen_(sLine, "a");
  if (!file)
  {
    DEBUG_MSG("Error open file %s", sLine);
    return;
  }
  if (bIncludeTimestamp)
    strcpy(sLine, "Timestamp,ID,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7\r\n");
  else
    strcpy(sLine, "ID,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7\r\n");
  fwrite_string(sLine);
  align_buffer();
  fwrite_(sd_buffer, 1, sd_buffer_length, file);
  f_sync(file);

  // reset buffer counters
  sd_buffer_length_for_write = 0;
  sd_buffer_length = 0;

  bWriteFault = 0;

  stLastWriting = HAL_GetTick(); // record time when we did write

  bLogging = 1;
}

void write_log()
{
  if (bReqWrite && file)
  {
    LedB(1);
    DEBUG_MSG("Write log to file %s", sLine);
    if (fwrite_(sd_buffer_for_write, 1, sd_buffer_length_for_write, file) != sd_buffer_length_for_write)
      bWriteFault = 2;
    if (f_sync(file) != FR_OK)
      bWriteFault = 2;
    bReqWrite = 0;

    if (bWriteFault)
    {
      DEBUG_MSG("Write fault : %d", bWriteFault);
    }
    stLastWriting = HAL_GetTick(); // record time when we did write

    LedB(0);
  }
}

void ProcessLogging(CAN_RxHeaderTypeDef *pTxMsg, uint8_t aData[])
{
  if (bLogging)
  {
    // checking message acceptance
    if (pTxMsg->IDE)
    {
      // message with extended ID received

      // are we accepting extended ID?
      if (!bLogExtMsgs)
        return;

      // then check filter conditions
      if ((pTxMsg->ExtId & iFilterMask) != (iFilterValue & iFilterMask))
        return;
    }
    else
    {
      // message with standard ID received

      // are we accepting standard ID?
      if (!bLogStdMsgs)
        return;

      // then check filter conditions
      if ((pTxMsg->StdId & iFilterMask) != (iFilterValue & iFilterMask))
        return;
    }

    // write down data
    if (bIncludeTimestamp)
      sprintf(sTmp, "%d,%X", (int)HAL_GetTick(), (int)pTxMsg->ExtId);
    else
      sprintf(sTmp, "%X", (int)pTxMsg->ExtId);

    for (i = 0; i < pTxMsg->DLC; i++)
    {
      sprintf(sTmp + strlen(sTmp), ",%02X", aData[i]);
    }

    strcat(sTmp, "\r\n");
    fwrite_string(sTmp);
  }
}
