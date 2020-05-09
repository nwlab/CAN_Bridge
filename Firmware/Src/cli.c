/*
 * cli.c
 *
 *  Created on: Apr 30, 2020
 *      Author: nightworker
 */

/*
 * List of commands
 * info
 * time [HH:MM:SS]
 * date [DD-MM-YY]
 * log [enable|disable]
 * can filter <id> <filterid> <filtermask>
 * can replace <id> <replaceid>
 * can replace clear
 *
 */
#include <stdio.h>

#include "libcli.h"
#include "usbd_cdc_if.h"
#include "rx_queue.h"
#include "nvs.h"
#include "logger.h"
#include "rtc.h"
#include "can.h"

struct cli_def *cli;
static uint8_t value = 0;
static uint16_t size = 0;
extern unsigned char bLogging; // if =1 than we logging to SD card

#define CLI_CAN_HEADER "<IDMask> <IDFilter> <NewIDMask> <NewIDValue> <DataMaskHigh> <DataMaskLow> <DataFilterHigh> <DataFilterLow> <NewDataMaskHigh> <NewDataMaskLow> <NewDataValueHigh> <NewDataValueLow>"
#define CLI_CAN_FORMAT "%5d 0x%4X 0x%6X 0x%7X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X"

void CAN_Loop();

int cmd_info(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_boot(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_can(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_can_filter(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_can_replace(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_log(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_log_enable(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_log_disable(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_date(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_time(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_commit(struct cli_def *cli, const char *command, char *argv[], int argc);

ssize_t stdin_cli_read(struct cli_def *cli, void *buf, size_t count)
{
  *(uint8_t*) buf = rx_queue_get();
  return count;
}

ssize_t stdout_cli_write(struct cli_def *cli, const void *buf, size_t count)
{
  CDC_Transmit_FS((void*) buf, count);
  return count;
}

int regular_callback(struct cli_def *cli)
{
  CAN_Loop();
  return 0;
}

void CLI_Init()
{
  struct cli_command *cc;
  cli = cli_init();
  cli_regular(cli, regular_callback);
  cli_read_callback(cli, stdin_cli_read);
  cli_write_callback(cli, stdout_cli_write);

  cli_register_command(cli, NULL, "info", cmd_info, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, "Device information");
  cli_register_command(cli, NULL, "boot", cmd_boot, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, "Boot loader status");
#ifdef HAL_RTC_MODULE_ENABLED
  cli_register_command(cli, NULL, "date", cmd_date, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, "Get/set date");
  cli_register_command(cli, NULL, "time", cmd_time, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, "Get/set time");
#endif

  cc = cli_register_command(cli, NULL, "can", cmd_can, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, "CAN commands");
  cli_register_command(cli, cc, "filter", cmd_can_filter, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
  cli_register_command(cli, cc, "replace", cmd_can_replace, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);

  /* Check file system is mounted */
  if (gFSInitialized)
  {
    cc = cli_register_command(cli, NULL, "log", cmd_log, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, "Configure logging");
    cli_register_command(cli, cc, "enable", cmd_log_enable, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
    cli_register_command(cli, cc, "disable", cmd_log_disable, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
  }

  cli_register_command(cli, NULL, "commit", cmd_commit, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
}

void CLI_Loop()
{
  DEBUG_MSG("Start CLI loop");
  cli_loop(cli, 0);
  DEBUG_MSG("Exited from CLI loop");
}

int cmd_info(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  cli_print(cli, "%s, Version : %s",  APP_NAME, APP_VERSION );
  nvs_get("boot", &value, &size, 1);
  cli_print(cli, "boot start : %s", value == LOADER_MODE_APP ? "application" : "bootloader");

  return CLI_OK;
}

int cmd_boot(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  if (argc > 0)
  {
    if (strcmp(argv[0], "?") == 0)
    {
      cli_print(cli, "Specify boot type (xmodem | flash)");
      return CLI_OK;
    }
//    sscanf(argv[0], "%u", &dev);
  }
  nvs_get("boot", &value, &size, 1);
  cli_print(cli, "boot start : %s", value == LOADER_MODE_APP ? "application" : "bootloader");
  return CLI_OK;
}

int cmd_can(struct cli_def *cli, const char *command, char *argv[], int argc)
{

  if (argc > 0)
  {
    if (strcmp(argv[0], "?") == 0)
    {
      cli_print(cli, "[clear|filter|replace]");
      return CLI_OK;
    }
    if (strcmp(argv[0], "clear") == 0)
    {
      cli_print(cli, "Clear statistics");
      iCAN1_Received = 0;
      iCAN2_Received = 0;
      iCAN1_Transmited = 0;
      iCAN2_Transmited = 0;
      return CLI_OK;
    }
  }
  cli_print(cli, "CAN1 Baudrate : %d", (int) (1000 * 3.0 / (float) (iCAN1_Prescaler) + 0.5));
  cli_print(cli, "CAN1 FilterId : 0x%X,  FilterMask : 0x%X", (int)iCAN1_FilterId, (int)iCAN1_FilterMaskId);
  cli_print(cli, "CAN2 Baudrate : %d", (int) (1000 * 3.0 / (float) (iCAN2_Prescaler) + 0.5));
  cli_print(cli, "CAN2 FilterId : 0x%X,  FilterMask : 0x%X", (int)iCAN2_FilterId, (int)iCAN2_FilterMaskId);
  cli_print(cli, "Replace count : %d", iReplace_Count);
  if (iReplace_Count)
    cli_print(cli, CLI_CAN_HEADER);
  for (int i=0; i<iReplace_Count; i++)
  {
    cli_print(cli, CLI_CAN_FORMAT, i,
                  iReplace[i].IDMask,
                  iReplace[i].IDFilter,
                  iReplace[i].NewIDMask,
                  iReplace[i].NewIDValue,
                  iReplace[i].DataMaskHigh,
                  iReplace[i].DataMaskLow,
                  iReplace[i].DataFilterHigh,
                  iReplace[i].DataFilterLow,
                  iReplace[i].NewDataMaskHigh,
                  iReplace[i].NewDataMaskLow,
                  iReplace[i].NewDataValueHigh,
                  iReplace[i].NewDataValueLow);
  }
  cli_print(cli, "CAN%d Received   : %d", 1, (int)iCAN1_Received);
  cli_print(cli, "CAN%d Transmited : %d", 1, (int)iCAN1_Transmited);
  cli_print(cli, "CAN%d Received   : %d", 2, (int)iCAN2_Received);
  cli_print(cli, "CAN%d Transmited : %d", 2, (int)iCAN2_Transmited);
  return CLI_OK;
}

/* can filter <id> <filterid> <filtermask> */
int cmd_can_filter(struct cli_def *cli, const char *command, char *argv[], int argc)
{
    int dev = 0;
    if (strcmp(argv[0], "?") == 0)
    {
      cli_print(cli, "[1|2] <filterid> <filtermask>");
      return CLI_OK;
    }
    if (argc > 0)
      sscanf(argv[0], "%u", &dev);
    if (argc > 1)
    {
      int fid = 0;
      int fmask = 0;
      sscanf(argv[1], "%x", &fid);
      sscanf(argv[2], "%x", &fmask);
      if (dev == 1)
      {
        iCAN1_FilterId = fid;
        iCAN1_FilterMaskId = fmask;
        nvs_put("c1_fid", (uint8_t*) &iCAN1_FilterId, sizeof(iCAN1_FilterId), sizeof(iCAN1_FilterId));
        nvs_put("c1_fmid", (uint8_t*) &iCAN1_FilterMaskId, sizeof(iCAN1_FilterMaskId), sizeof(iCAN1_FilterMaskId));
      }
      if (dev == 2)
      {
        iCAN2_FilterId = fid;
        iCAN2_FilterMaskId = fmask;
        nvs_put("c2_fid", (uint8_t*) &iCAN2_FilterId, sizeof(iCAN2_FilterId), sizeof(iCAN2_FilterId));
        nvs_put("c2_fmid", (uint8_t*) &iCAN2_FilterMaskId, sizeof(iCAN2_FilterMaskId), sizeof(iCAN2_FilterMaskId));
      }
    }
    else
    {
      if (dev == 1 || dev == 0)
      {
        cli_print(cli, "CAN1 FilterId : 0x%X,  FilterMask : 0x%X", (int)iCAN1_FilterId, (int)iCAN1_FilterMaskId);
      }
      if (dev == 2 || dev == 0)
      {
        cli_print(cli, "CAN2 FilterId : 0x%X,  FilterMask : 0x%X", (int)iCAN2_FilterId, (int)iCAN2_FilterMaskId);
      }
    }
  return CLI_OK;
}

/* can replace [clear] <idmask> <idfilter> <newidmask> <newidvalue> <datamaskhigh> <datamasklow>*/
int cmd_can_replace(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  if (argc > 0)
  {
    if (strcmp(argv[0], "?") == 0)
    {
      cli_print(cli, CLI_CAN_HEADER);
      return CLI_OK;
    }
    if (strcmp(argv[0], "clear") == 0)
    {
      iReplace_Count = 0;
      cli_print(cli, "Ok");
      return CLI_OK;
    }
    sscanf(argv[1], "%x", &iReplace[iReplace_Count].IDMask);
    sscanf(argv[2], "%x", &iReplace[iReplace_Count].IDFilter);
    sscanf(argv[3], "%x", &iReplace[iReplace_Count].NewIDMask);
    sscanf(argv[4], "%x", &iReplace[iReplace_Count].NewIDValue);
    sscanf(argv[5], "%x", &iReplace[iReplace_Count].DataMaskHigh);
    sscanf(argv[6], "%x", &iReplace[iReplace_Count].DataMaskLow);
    sscanf(argv[7], "%x", &iReplace[iReplace_Count].DataFilterHigh);
    sscanf(argv[8], "%x", &iReplace[iReplace_Count].DataFilterLow);
    sscanf(argv[9], "%x", &iReplace[iReplace_Count].NewDataMaskHigh);
    sscanf(argv[10], "%x", &iReplace[iReplace_Count].NewDataMaskLow);
    sscanf(argv[11], "%x", &iReplace[iReplace_Count].NewDataValueHigh);
    sscanf(argv[12], "%x", &iReplace[iReplace_Count].NewDataValueLow);
    iReplace_Count++;
  }
  else
  {
    cli_print(cli, "Index IDMask IDFilter NewIDMask NewIDValue DataMaskHigh DataMaskLow DataFilterHigh DataFilterLow NewDataMaskHigh NewDataMaskLow NewDataValueHigh NewDataValueLow");
    for (int i=0; i<iReplace_Count; i++)
    {
      cli_print(cli, CLI_CAN_FORMAT, i,
                    iReplace[i].IDMask,
                    iReplace[i].IDFilter,
                    iReplace[i].NewIDMask,
                    iReplace[i].NewIDValue,
                    iReplace[i].DataMaskHigh,
                    iReplace[i].DataMaskLow,
                    iReplace[i].DataFilterHigh,
                    iReplace[i].DataFilterLow,
                    iReplace[i].NewDataMaskHigh,
                    iReplace[i].NewDataMaskLow,
                    iReplace[i].NewDataValueHigh,
                    iReplace[i].NewDataValueLow);
    }
  }
  return CLI_OK;
}
/*
 * log - show info
 * log enable
 * log disable
 *
 * */
int cmd_log(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  cli_print(cli, "Logger %s", bLogging?"started":"stopped");
  return CLI_OK;
}

int cmd_log_enable(struct cli_def *cli, const char *command, char *argv[], int argc)
{
    if (bLogging)
    {
      cli_print(cli, "Logger already started");
      return CLI_OK;
    }

    bLogging = 1;

    start_log();

    return CLI_OK;
}

int cmd_log_disable(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  if (!bLogging)
  {
    cli_print(cli, "Logger already stopped");
    return CLI_OK;
  }

  bLogging = 0;

  // we are in logging state -- should write the rest of log
  request_write();

  return CLI_OK;
}

int cmd_date(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  RTC_DateTypeDef sDate;
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

  if (argc > 0)
  {
    int year, month, day;
    if (3 != sscanf(argv[0], "%d-%d-%d", &day, &month, &year))
    {
      cli_print(cli, "Error date format");
    }
    else
    {
      HAL_StatusTypeDef res;

      sDate.Year = year - 2000;
      sDate.Month = month;
      sDate.Date = day;

      res = HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
      if (res != HAL_OK)
      {
        cli_print(cli, "HAL_RTC_SetDate failed: %d", res);
      }
    }
  }
  else
  {
    cli_print(cli, "%.2d-%.2d-%.2d", sDate.Month, sDate.Date, 2000 + sDate.Year);
  }
  return CLI_OK;
}

int cmd_time(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  if (argc > 0)
  {
    int hour, min, sec;
    if (3 != sscanf(argv[0], "%d:%d:%d", &hour, &min, &sec))
    {
      cli_print(cli, "Error time format");
    }
    else
    {
      RTC_TimeTypeDef sTime;
      HAL_StatusTypeDef res;

      HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);

      sTime.Hours = hour;
      sTime.Minutes = min;
      sTime.Seconds = sec;
      sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
      sTime.StoreOperation = RTC_STOREOPERATION_SET;

      res = HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BIN);
      if (res != HAL_OK)
      {
        cli_print(cli, "HAL_RTC_SetTime failed: %d", res);
      }
    }
  }
  else
  {
    // Show time
    RTC_TimeTypeDef sTime;
    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BCD);
    cli_print(cli, "%.2d:%.2d:%.2d", ((sTime.Hours & 0x0F) + ((sTime.Hours & 0xF0) >> 4) * 10),
        ((sTime.Minutes & 0x0F) + ((sTime.Minutes & 0xF0) >> 4) * 10),
        ((sTime.Seconds & 0x0F) + ((sTime.Seconds & 0xF0) >> 4) * 10));
  }
  return CLI_OK;
}

int cmd_commit(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  CAN_Write_Param();

  if (nvs_commit() != NVS_OK)
  {
    cli_print(cli, "Flash commit failed");
  }
  else
  {
    cli_print(cli, "Ok");
  }
  return CLI_OK;
}
