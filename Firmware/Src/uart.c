/*
 * uart.c
 *
 *  Created on: May 1, 2020
 *      Author: nightworker
 */
#include "main.h"

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Timeout for HAL. */
#define UART_TIMEOUT ((uint16_t)1000u)

UART_HandleTypeDef huart3;

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  uint8_t data = (uint8_t) ch;

  /* Make available the UART module. */
  if (HAL_UART_STATE_TIMEOUT == HAL_UART_GetState(&huart3))
  {
    HAL_UART_Abort(&huart3);
  }

  HAL_UART_Transmit(&huart3, &data, 1u, UART_TIMEOUT);

  return ch;
}

int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
}

void infinite_message(char* msg)
{
  uint16_t length = 0u;

  /* Calculate the length. */
  while ('\0' != msg[length])
  {
    length++;
  }

  for (;;)
  {
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, length, UART_TIMEOUT);
    HAL_Delay(10000);
    LedB_Toggle();
  }
}
