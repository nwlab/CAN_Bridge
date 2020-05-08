/*
 * cli.c
 *
 *  Created on: Apr 30, 2020
 *      Author: nightworker
 */
#include <stdio.h>

#include "libcli.h"
#include "usbd_cdc_if.h"
#include "rx_queue.h"
#include "nvs.h"
#include "logger.h"
#include "rtc.h"

struct cli_def *cli;

void CAN_Loop();

int cmd_info(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_boot(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_can(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_log(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_no_log(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_date(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_time(struct cli_def *cli, const char *command, char *argv[], int argc);

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
  struct cli_command *c_no;
  cli = cli_init();
//	 cli_set_context(cli, (void*)&param);
//	 cli_set_banner(cli, banner);
  cli_telnet_protocol(cli, 1);
  cli_regular(cli, regular_callback);
//	 cli_regular_interval(cli, param->regular_interval); // Defaults to 1 second
  cli_read_callback(cli, stdin_cli_read);
  cli_write_callback(cli, stdout_cli_write);

  c_no = cli_register_command(cli, NULL, "no", NULL, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
  cli_register_command(cli, NULL, "info", cmd_info, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
  cli_register_command(cli, NULL, "boot", cmd_boot, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
  cli_register_command(cli, NULL, "date", cmd_date, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
  cli_register_command(cli, NULL, "time", cmd_time, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);

  cli_register_command(cli, NULL, "can", cmd_can, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
  cli_register_command(cli, NULL, "log", cmd_log, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
  cli_register_command(cli, c_no, "log", cmd_no_log, PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
}

void CLI_Loop()
{
  DEBUG_MSG("Start CLI loop");
  cli_loop(cli, 0);
  DEBUG_MSG("Exited from CLI loop");
}

int cmd_info(struct cli_def *cli, const char *command, char *argv[], int argc)
{

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
  uint8_t run_mode = 0;
  uint16_t size = 0;
  nvs_get("boot", &run_mode, &size, 1);
  cli_print(cli, "boot start : %s", run_mode == LOADER_MODE_APP ? "application" : "bootloader");
  return CLI_OK;
}

int cmd_can(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  int iCAN1_Prescaler;
  int iCAN2_Prescaler;
  int iReplace_Count;
  uint16_t size = 0;
  if (argc > 0)
  {

  }
  else
  {
    nvs_get("c1_pre", (uint8_t*) &iCAN1_Prescaler, &size, sizeof(iCAN1_Prescaler));
    nvs_get("c2_pre", (uint8_t*) &iCAN2_Prescaler, &size, sizeof(iCAN2_Prescaler));
    nvs_get("repl_cnt", (uint8_t*) &iReplace_Count, &size, sizeof(iReplace_Count));

    cli_print(cli, "CAN1 Baudrate : %d", (int) (1000 * 3.0 / (float) (iCAN1_Prescaler) + 0.5));
    cli_print(cli, "CAN2 Baudrate : %d", (int) (1000 * 3.0 / (float) (iCAN2_Prescaler) + 0.5));
    cli_print(cli, "Replace count : %d", iReplace_Count);
  }

  return CLI_OK;
}

extern unsigned char bLogging; // if =1 than we logging to SD card

int cmd_log(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  if (argc > 0)
  {
    if (strcmp(argv[0], "?") == 0)
    {
      cli_print(cli, "Specify parameter name");
      return CLI_OK;
    }
  }
  else
  {
    if (bLogging)
    {
      cli_print(cli, "Logger already started");
      return CLI_OK;
    }

    bLogging = 1;

    start_log();

  }
  return CLI_OK;
}

int cmd_no_log(struct cli_def *cli, const char *command, char *argv[], int argc)
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
  if (argc > 0)
  {
    int year, month, day;
    if (3 != sscanf(argv[0], "%d-%d-%d", &day, &month, &year))
    {
      cli_print(cli, "Error date format");
    }
    else
    {
      RTC_DateTypeDef sDate;
      HAL_StatusTypeDef res;

      memset(&sDate, 0, sizeof(sDate));

      sDate.Year = year;
      sDate.Month = month;
      sDate.Date = day;

      res = HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
      if (res != HAL_OK)
      {
        cli_print(cli, "HAL_RTC_SetDate failed: %d", res);
      }
    }
  }
  else
  {
    // Show date
    RTC_DateTypeDef sDate;
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
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

      memset(&sTime, 0, sizeof(sTime));

      sTime.Hours = hour;
      sTime.Minutes = min;
      sTime.Seconds = sec;

      res = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
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
