/*******************************************************************************
  * @file           : boot.c
  * @author         : Andrii Iakovenko
  * @date           : 27 Mar 2020
  * @brief          : Reboot and jump app functions implementation
  ******************************************************************************/
#include <stdio.h>
#include "main.h"
#include "boot.h"
#include "gpio.h"
#include "bootloader-config.h"

#ifdef UART_ENABLED
extern UART_HandleTypeDef huart3;
#endif
extern SD_HandleTypeDef hsd;
/* Function pointer for jumping to user application. */
typedef void (*fnc_ptr)(void);

/**
 * @brief   Actually jumps to the user application.
 * @param   appJumpAddress
 * @return  void
 */
__attribute__ ((noreturn))
void jump_to_app(uint32_t appJumpAddress)
{
  /* Function pointer to the address of the user application. */
  fnc_ptr func_app;
  func_app = (fnc_ptr)(*(volatile uint32_t*) (FLASH_APP_START_ADDRESS+4u));

  /* Deinit all modules*/
  HAL_RCC_DeInit();
  GPIO_DeInit();
  HAL_DeInit();
  /* Change the main stack pointer. */
  __set_MSP(*(volatile uint32_t*)FLASH_APP_START_ADDRESS);
  func_app();
  for (;;);
}

void reboot()
{
  INFO_MSG("Rebooting...");
  HAL_SD_DeInit(&hsd);
#ifdef UART_ENABLED
  HAL_UART_DeInit(&huart3);
#endif
	NVIC_SystemReset();
}
