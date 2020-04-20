/*
 * ring_buffer.c
 *
 *  Created on: Apr 19, 2020
 *      Author: nightworker
 */

#include "main.h"

#define RX_QUEUE_LENGTH 1024
static uint8_t bRxQueue[RX_QUEUE_LENGTH];
static int iRxQueueCount = 0;
static int iRxQueueIn = 0;
static int iRxQueueOut = 0;

// Simple Queue implementation for serial port



void rx_queue_put(uint8_t data)
{
  if (iRxQueueCount < RX_QUEUE_LENGTH)
  {
    bRxQueue[iRxQueueIn] = data;
    iRxQueueIn = iRxQueueIn + 1;
    if (iRxQueueIn >= RX_QUEUE_LENGTH) iRxQueueIn = 0;
    iRxQueueCount++;
  }
}

uint8_t rx_queue_get()
{
  uint8_t data = 0;

  if (iRxQueueCount > 0)
  {
    data = bRxQueue[iRxQueueOut];
    iRxQueueOut = iRxQueueOut + 1;
    if (iRxQueueOut >= RX_QUEUE_LENGTH) iRxQueueOut = 0;
    iRxQueueCount--;
  }
  return data;
}

uint8_t rx_queue_not_empty()
{
  return iRxQueueCount > 0;
}

