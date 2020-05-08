/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define FLASH_CFG_START_ADDRESS ((uint32_t)0x0800C000u)   /*Sector 3*/
#define FLASH_CFG_START_SECTOR  (FLASH_SECTOR_3)
#define FLASH_CFG_SIZE          2048 /*0x4000 */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
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

#define LOADER_MODE_TEST         0x00
#define LOADER_MODE_APP          0x12
#define LOADER_MODE_FLASH        0x43
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void LedG(uint8_t On);
void LedG_Toggle(void);
void LedB(uint8_t On);
void LedB_Toggle(void);
void LedR(uint8_t On);
void LedR_Toggle(void);

extern uint32_t toggle_time_g;
extern uint32_t toggle_time_b;

#define Error_Handler() __Error_Handler(__FUNCTION__, __LINE__)
void __Error_Handler(const char *func, int line);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
