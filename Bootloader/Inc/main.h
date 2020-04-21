/*******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************/
#ifndef _MAIN_H_
#define _MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
#include "bootloader-config.h"
#include "uart.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#if ( DEBUG == DEBUG_ENABLE )
    /* printf definetion */
    #define DEBUG_MSG( fmt, ...) \
    do \
    { \
      printf( "[%08ld]%s:%d:%s(): " fmt "\r\n", HAL_GetTick(), __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    }while( 0 );
#else
    #define DEBUG_MSG( fmt, ... )
#endif

#if ( DEBUG == DEBUG_ENABLE )
    /* printf definetion */
    #define INFO_MSG( fmt, ...) \
    do \
    { \
      printf( fmt "\r\n", ##__VA_ARGS__); \
    }while( 0 );
#else
    #define INFO_MSG( fmt, ... )
#endif

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void main_tick_1ms();
void main_tick_5ms();
void main_tick_10ms();
void main_tick_50ms();
void main_tick_100ms();
void main_tick_500ms();
void main_tick_1s();

/* Private defines -----------------------------------------------------------*/

#define LOADER_MODE_TEST         0x00
#define LOADER_MODE_APP          0x12
#define LOADER_MODE_FLASH        0x43

#ifdef __cplusplus
}
#endif

#endif /* _MAIN_H_ */

