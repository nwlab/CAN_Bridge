/**
 * @file    uart.c
 * @author  Ferenc Nemeth
 * @date    21 Dec 2018
 * @brief   This module is a layer between the HAL UART functions and my Xmodem protocol.
 *
 *          Copyright (c) 2018 Ferenc Nemeth - https://github.com/ferenc-nemeth
 */

#include "uart.h"

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */

  uart_transmit_ch((uint8_t)ch);

  return ch;
}

int _write(int file,char *ptr, int len)
{
 int DataIdx;
 for(DataIdx= 0; DataIdx< len; DataIdx++)
 {
 __io_putchar(*ptr++);
 }
 return len;
}

/**
 * @brief   Receives data from UART.
 * @param   *data: Array to save the received data.
 * @param   length:  Size of the data.
 * @return  status: Report about the success of the receiving.
 */
uart_status uart_receive(uint8_t *data, uint16_t length)
{
  uart_status status = UART_ERROR;

  if (HAL_OK == HAL_UART_Receive(&huart3, data, length, UART_TIMEOUT))
  {
    status = UART_OK;
  }

  return status;
}

/**
 * @brief   Transmits a string to UART.
 * @param   *data: Array of the data.
 * @return  status: Report about the success of the transmission.
 */
uart_status uart_transmit_str(uint8_t *data)
{
  uart_status status = UART_ERROR;
  uint16_t length = 0u;

  /* Calculate the length. */
  while ('\0' != data[length])
  {
    length++;
  }

  if (HAL_OK == HAL_UART_Transmit(&huart3, data, length, UART_TIMEOUT))
  {
    status = UART_OK;
  }

  return status;
}

/**
 * @brief   Transmits a single char to UART.
 * @param   *data: The char.
 * @return  status: Report about the success of the transmission.
 */
uart_status uart_transmit_ch(uint8_t data)
{
  uart_status status = UART_ERROR;

  /* Make available the UART module. */
  if (HAL_UART_STATE_TIMEOUT == HAL_UART_GetState(&huart3))
  {
    HAL_UART_Abort(&huart3);
  }

  if (HAL_OK == HAL_UART_Transmit(&huart3, &data, 1u, UART_TIMEOUT))
  {
    status = UART_OK;
  }
  return status;
}
