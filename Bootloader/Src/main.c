/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bootloader-config.h"
#include "stm32f4xx_hal.h"
#include "xmodem.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART3_UART_Init(void);
void RunTests(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void GoToUserApp(void)
{
	uint32_t appJumpAddress;
	void (*GoToApp)(void);
	appJumpAddress = *((volatile uint32_t*)(FLASH_APP_START_ADDRESS + 4));
	GoToApp = (void (*)(void))appJumpAddress;
	SCB->VTOR = FLASH_APP_START_ADDRESS;
	__set_MSP(*((volatile uint32_t*) FLASH_APP_START_ADDRESS)); //stack pointer (to RAM) for USER app in this address
	GoToApp();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  MX_SDIO_SD_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

#if defined(UART_ENABLED)
  printf("\n\r================================\n\r");
  printf("USB/Xmodem Bootloader\n\r");
  printf("https://github.com/nwlab/CAN_Bridge\n\r");
  printf("Based on https://github.com/ferenc-nemeth/stm32-bootloader\n\r");
  printf("================================\n\r");
  printf("Compiled : " __DATE__ ", " __TIME__ "\n\r");
  printf("Flash base address : 0x%08lX\n\r", FLASH_BASE);
  printf("Flash size :         0x%08lX\n\r", (FLASH_END - FLASH_BASE));
  printf("User application start address : 0x%08lX\n\r", FLASH_APP_START_ADDRESS);
  printf("================================\n\r\n\r");
#endif

  RunTests(); // this is function to run transmission tests

  /* If the button is pressed, then jump to the user application,
   * otherwise stay in the bootloader. */
//  if(!HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin))
//  {
//    printf("Jumping to user application...\n\r");
//    flash_jump_to_app();
//  }
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* Turn on the green LED to indicate, that we are in bootloader mode.*/
    HAL_GPIO_WritePin(LEDR_GPIO, LEDR_PIN, GPIO_PIN_SET);
    /* Ask for new data and start the Xmodem protocol. */
    printf("Please send a new binary file with Xmodem protocol to update the firmware.\n\r");
    xmodem_process();
    /* We only exit the xmodem protocol, if there are any errors.
     * In that case, notify the user and start over. */
    printf("\n\rFailed... Please try again.\n\r");
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//#define TEST_USB_TRANSMIT
// if defined leds blink
//#define TEST_BLINKY

void RunTests()
{
  #ifdef TEST_BLINKY

    while(1)
    {
       // Toggle all LED
      LedG_Toggle();
      LedR_Toggle();
      LedB_Toggle();
       // Delay for 1sec
      HAL_Delay(1000);
    }
  #endif

  #ifdef TEST_USB_TRANSMIT
    while(1)
    {
      uint8_t data = 'A';
       // Toggle all LED
      HAL_GPIO_TogglePin(LEDG_GPIO, LEDG_PIN);

      if (CDC_Transmit_FS(&data, 1) == USBD_OK)
      {
        printf("Send OK\n\r");
      }
      else
      {
        printf("Send ERROR\n\r");
      }
       // Delay for 1sec
      HAL_Delay(1000);
    }
  #endif

  #ifdef TEST_LISTEN_ONLY
    while (1) if (RxQueueNotEmpty()) UART_ProcessData(RxQueueGet());
  #endif
}
//----------------------------------------------------------------------------------------
// Simple Queue implementation for serial port

#define RX_QUEUE_LENGTH 64
static uint8_t bRxQueue[RX_QUEUE_LENGTH];
static int iRxQueueCount = 0;
static int iRxQueueIn = 0;
static int iRxQueueOut = 0;

void RxQueuePut(uint8_t data)
{
  if (iRxQueueCount < RX_QUEUE_LENGTH)
  {
    bRxQueue[iRxQueueIn] = data;
    iRxQueueIn = iRxQueueIn + 1;
    if (iRxQueueIn >= RX_QUEUE_LENGTH) iRxQueueIn = 0;
    iRxQueueCount++;
  }
}

uint8_t RxQueueGet()
{
  uint8_t data = 0;

  if (iRxQueueCount > 0)
  {
    data = bRxQueue[iRxQueueOut];
    iRxQueueOut = iRxQueueOut + 1;
    if (iRxQueueOut >= RX_QUEUE_LENGTH) iRxQueueOut = 0;
    iRxQueueCount--;
  }

  HAL_GPIO_WritePin(LEDG_GPIO, LEDG_PIN, GPIO_PIN_RESET);
  return data;
}

uint8_t RxQueueNotEmpty() { return iRxQueueCount > 0; }

xmodem_status xmodem_receive(uint8_t *data, uint16_t length)
{
  uint16_t i = 0;
  while (i<length)
  {
    if (RxQueueNotEmpty())
    {
      data[i] = RxQueueGet();
      i++;
    }
  }
  return X_OK;
}

xmodem_status xmodem_transmit_ch(uint8_t data)
{
  HAL_GPIO_WritePin(LEDB_GPIO, LEDB_PIN, GPIO_PIN_SET);
  if (CDC_Transmit_FS(&data, 1) == USBD_OK)
  {
    HAL_GPIO_WritePin(LEDB_GPIO, LEDB_PIN, GPIO_PIN_RESET);
    return X_OK;
  }
  return X_ERROR_UART;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
