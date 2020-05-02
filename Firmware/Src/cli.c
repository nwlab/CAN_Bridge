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

struct cli_def *cli;

void CAN_Loop();

int cmd_info(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_boot(struct cli_def *cli, const char *command, char *argv[], int argc);
int cmd_can(struct cli_def *cli, const char *command, char *argv[], int argc);

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
  struct cli_command *c_show;
  struct cli_command *c_set;
  cli = cli_init();
//	 cli_set_context(cli, (void*)&param);
//	 cli_set_banner(cli, banner);
  cli_telnet_protocol(cli, 1);
  cli_regular(cli, regular_callback);
//	 cli_regular_interval(cli, param->regular_interval); // Defaults to 1 second
  cli_read_callback(cli, stdin_cli_read);
  cli_write_callback(cli, stdout_cli_write);

  cli_register_command(cli, NULL, "info", cmd_info, PRIVILEGE_UNPRIVILEGED,
      MODE_EXEC, NULL);
  c_show = cli_register_command(cli, NULL, "show", cmd_boot,
      PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
  cli_register_command(cli, c_show, "boot", cmd_boot, PRIVILEGE_UNPRIVILEGED,
      MODE_EXEC, NULL);
  cli_register_command(cli, c_show, "can", cmd_can, PRIVILEGE_UNPRIVILEGED,
      MODE_EXEC, NULL);
  c_set = cli_register_command(cli, NULL, "set", cmd_boot,
      PRIVILEGE_UNPRIVILEGED, MODE_EXEC, NULL);
  cli_register_command(cli, c_set, "can", cmd_can, PRIVILEGE_UNPRIVILEGED,
      MODE_EXEC, NULL);

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
  if (nvs_get("boot", &run_mode, &size, 1) == KEY_NOT_FOUND)
  {
    cli_print(cli, "boot flag not found");
  }
  else
  {
    cli_print(cli, "boot start : %s",
        run_mode == LOADER_MODE_APP ? "application" : "bootloader");
  }
  return CLI_OK;
}

int cmd_can(struct cli_def *cli, const char *command, char *argv[], int argc)
{
  int iCAN1_Prescaler;
  int iCAN2_Prescaler;
  uint16_t size = 0;
  nvs_get("c1_pre", (uint8_t*) &iCAN1_Prescaler, &size,
      sizeof(iCAN1_Prescaler));
  nvs_get("c2_pre", (uint8_t*) &iCAN2_Prescaler, &size,
      sizeof(iCAN2_Prescaler));

  cli_print(cli, "CAN1 Baudrate : %d",
      (int) (1000 * 3.0 / (float) (iCAN1_Prescaler) + 0.5));
  cli_print(cli, "CAN2 Baudrate : %d",
      (int) (1000 * 3.0 / (float) (iCAN2_Prescaler) + 0.5));

  return CLI_OK;
}
