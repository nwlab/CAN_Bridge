/*******************************************************************************
  * @file           : gpio.h
  * @author         : Andrii Iakovenko
  * @date           : 27 Mar 2020
  * @brief          : This module handles the gpio related functions.
  ******************************************************************************/
#ifndef _GPIO_H_
#define _GPIO_H_

#include "main.h"

#define USB_RESET_PIN      GPIO_PIN_10
#define USB_RESET_GPIO     GPIOA

#define USB_CONNECTED_PIN  GPIO_PIN_9
#define USB_CONNECTED_GPIO GPIOA

#define LEDG_PIN GPIO_PIN_7
#define LEDG_GPIO GPIOA

#define LEDB_PIN GPIO_PIN_6
#define LEDB_GPIO GPIOA

#define LEDR_PIN GPIO_PIN_5
#define LEDR_GPIO GPIOA


void User_GPIO_Init(void);
void User_GPIO_DeInit();
uint8_t GPIO_usb_is_connected();

#endif // _GPIO_H_
