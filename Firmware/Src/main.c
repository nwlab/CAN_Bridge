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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nvs.h"
#include "rx_queue.h"
#include "cli.h"
#include "uart.h"
#include "gpio.h"
#include "filesystem.h"
#include "logger.h"
#include "can.h"
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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t gSystemInitialized = 0;
volatile uint8_t gFSInitialized = 0;

unsigned char bLogging = 0; // if =1 than we logging to SD card

// if defined CAN1 acting as loopback
// #define CAN1_LOOPBACK

// if defined CAN2 acting as loopback
// #define CAN2_LOOPBACK

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void User_GPIO_Init(void);
void LedG(uint8_t On);
void LedG_Toggle(void);
void LedB(uint8_t On);
void LedB_Toggle(void);
void LedR(uint8_t On);
void LedR_Toggle(void);
void ProcessModification(CAN_TxHeaderTypeDef* pTxMsg);
void RunTests(void);

void UART_ProcessData(uint8_t rx);

void Read_Param();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  nvs_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  User_GPIO_Init();
  CAN_Init();
  LedG(1);

  CLI_Init();

  gSystemInitialized = 1;

 /* Output a message on Hyperterminal using printf function */
  printf("\n\r" APP_NAME "\n\r");
  printf("Version : " APP_VERSION "\n\r");
  printf("Compiled : " __DATE__ ", " __TIME__ "\n\r");
  INFO_MSG("CPU speed  : %ld MHz", SystemCoreClock / 1000000);

  if (init_filesystem() == FILESYSTEM_INIT_OK)
  {
    printf("FS mount successfuly\n\r");
    gFSInitialized = 1;
  }
  else
  {
    printf("Error mount FS\n\r");
  }

  RunTests(); // this is function to run transmission tests 
 
  // this is loopback cycle
  if (GPIO_usb_is_connected())
  {
    CLI_Loop();
  }
  /* USER CODE END 2 */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  DEBUG_MSG("Original infinite loop");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CAN_Loop();
    if (rx_queue_not_empty()) UART_ProcessData(rx_queue_get());
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /* Read parameters from nvram */
  CAN_Read_Param();

#if 0
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
#endif
  /* Configure the CAN peripheral */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = iCAN1_Prescaler; //3 -- 1Msps , 6 -- 500Kbps
  #ifdef CAN1_LOOPBACK
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  #else
    hcan1.Init.Mode = CAN_MODE_NORMAL;
  #endif
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Configure the CAN Filter */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
  CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */
#if 0
  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
#endif
  /* Configure the CAN peripheral */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = iCAN2_Prescaler; //3 -- 1Msps, 6 --500kBps
  #ifdef CAN2_LOOPBACK
    hcan2.Init.Mode = CAN_MODE_LOOPBACK;
  #else
    hcan2.Init.Mode = CAN_MODE_NORMAL;
  #endif
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 15;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
  /* USER CODE END CAN2_Init 2 */
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  static uint8_t usart3_init = 0;
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  if (++usart3_init != 1)
    return;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */


uint32_t toggle_time_r = 0;
uint32_t toggle_time_g = 0;
uint32_t toggle_time_b = 0;

void main_tick_1ms()
{
}
void main_tick_5ms()
{
}
void main_tick_10ms()
{
}
void main_tick_50ms()
{
  if (toggle_time_r > 0)
  {
    HAL_GPIO_TogglePin(LEDR_GPIO, LEDR_PIN);
    toggle_time_r--;
    if (toggle_time_r == 0)
      HAL_GPIO_WritePin(LEDR_GPIO, LEDR_PIN, GPIO_PIN_RESET);
  }
  if (toggle_time_g > 0)
  {
    HAL_GPIO_TogglePin(LEDG_GPIO, LEDG_PIN);
    toggle_time_g--;
    if (toggle_time_g == 0)
      HAL_GPIO_WritePin(LEDG_GPIO, LEDG_PIN, GPIO_PIN_RESET);
  }
  if (toggle_time_b > 0)
  {
    HAL_GPIO_TogglePin(LEDB_GPIO, LEDB_PIN);
    toggle_time_b--;
    if (toggle_time_b == 0)
      HAL_GPIO_WritePin(LEDB_GPIO, LEDB_PIN, GPIO_PIN_RESET);
  }
}
void main_tick_100ms()
{
}
void main_tick_500ms()
{
}
void main_tick_1s()
{
}

void LedG(uint8_t On)
{
  HAL_GPIO_WritePin(LEDG_GPIO, LEDG_PIN, On);
}

void LedG_Toggle()
{
  HAL_GPIO_TogglePin(LEDG_GPIO, LEDG_PIN);
}

void LedB(uint8_t On)
{
  HAL_GPIO_WritePin(LEDB_GPIO, LEDB_PIN, On);
}

void LedB_Toggle()
{
  HAL_GPIO_TogglePin(LEDB_GPIO, LEDB_PIN);
}

void LedR(uint8_t On)
{
  HAL_GPIO_WritePin(LEDR_GPIO, LEDR_PIN, On);
}

void LedR_Toggle()
{
  HAL_GPIO_TogglePin(LEDR_GPIO, LEDR_PIN);
}

//----------------------------------------------------------------------------------------
// Serial UART over USB configuration code section (communication protocol FSM)
//----------------------------------------------------------------------------------------
typedef enum
{
  UART_STATE_IDLE, // waiting for command
  UART_STATE_SYNC_1, // receiving sync byte 1
  UART_STATE_SYNC_2, // receiving sync byte 2
  UART_STATE_ADDR_0, // receiving addr byte 0
  UART_STATE_ADDR_1, // receiving addr byte 1
  UART_STATE_ADDR_2, // receiving addr byte 1
  UART_STATE_ADDR_3, // receiving addr byte 1
  UART_STATE_LEN_0, // receiving length byte 2
  UART_STATE_LEN_1, // receiving sync byte 2
  UART_STATE_CMD, // command byte
  UART_STATE_WRITE_0, // receive write byte 0
  UART_STATE_WRITE_1, // receive write byte 0
  UART_STATE_WRITE_2, // receive write byte 0
  UART_STATE_WRITE_3, // receive write byte 0
} UART_RX_CMD_States;

UART_RX_CMD_States bUartRxState = UART_STATE_IDLE;
uint32_t iMemRWAddr = 0;
uint16_t iMemRWByteCount = 0;
uint8_t * bUARTMemTxBuff; // this is actually memory buffer for
uint8_t bUARTMemTxResponse[3] = {'1', '2', 'Y'};



// Variable used for Erase procedure
static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t PAGEError = 0;
uint32_t iFlashData = 0;

#include "usbd_cdc_if.h"

void UART_ProcessData(uint8_t rx)
{
  if (bUartRxState == UART_STATE_IDLE)
  {
    if (rx == '1')
      bUartRxState = UART_STATE_SYNC_1;
  }
  else
  if (bUartRxState == UART_STATE_SYNC_1)
  {
    if (rx == '2')
      bUartRxState = UART_STATE_ADDR_0;
    else
      bUartRxState = UART_STATE_IDLE;
  }
  else
  if (bUartRxState == UART_STATE_ADDR_0)
  {
    iMemRWAddr = rx << 24;
    bUartRxState = UART_STATE_ADDR_1;
  }
  else
  if (bUartRxState == UART_STATE_ADDR_1)
  {
    iMemRWAddr |= rx << 16;
    bUartRxState = UART_STATE_ADDR_2;
  }
  else
  if (bUartRxState == UART_STATE_ADDR_2)
  {
    iMemRWAddr |= rx << 8;
    bUartRxState = UART_STATE_ADDR_3;
  }
  else
  if (bUartRxState == UART_STATE_ADDR_3)
  {
    iMemRWAddr |= rx << 0;
    bUartRxState = UART_STATE_LEN_0;
  }
  else
  if (bUartRxState == UART_STATE_LEN_0)
  {
    iMemRWByteCount = rx << 16;
    bUartRxState = UART_STATE_LEN_1;
  }
  else
  if (bUartRxState == UART_STATE_LEN_1)
  {
    iMemRWByteCount |= rx << 0;
    bUartRxState = UART_STATE_CMD;
  }
  else
  if (bUartRxState == UART_STATE_CMD)
  {
    if (rx == 'R')
    {
      bUartRxState = UART_STATE_IDLE;
      bUARTMemTxBuff = ((uint8_t*)(iMemRWAddr)); // construct pointer to memory from integer address
      CDC_Transmit_FS(bUARTMemTxBuff, iMemRWByteCount);
    }
    else
    if (rx == 'E')
    {
      bUartRxState = UART_STATE_IDLE;
      HAL_NVIC_SystemReset();
    }
    else
    if (rx == 'F')
    {
      // Unlock the Flash to enable the flash control register access 
      HAL_FLASH_Unlock();

      // Fill EraseInit structure
#ifdef STM32F405xx
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.Sector      = iMemRWAddr;
		EraseInitStruct.NbSectors   = 1;
#else			
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = iMemRWAddr;
		EraseInitStruct.NbPages     = 1; // we arasing only one page (teh last one)
#endif
      
      // Erase the user Flash area
      if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
      {
        LedR(1);
        bUARTMemTxResponse[2] = 'N';
      }
      else
        bUARTMemTxResponse[2] = 'Y';
      
      // transmit back response
      CDC_Transmit_FS(bUARTMemTxResponse, 3);
  
      bUartRxState = UART_STATE_IDLE;
    }
    else
    if (rx == 'W')
    {
      bUartRxState = UART_STATE_WRITE_0;
      
      // Unlock the Flash to enable the flash control register access 
      HAL_FLASH_Unlock();
    }
  }
  else
  if (bUartRxState == UART_STATE_WRITE_0)
  {
    iFlashData = (rx)&0xFF;
    bUartRxState = UART_STATE_WRITE_1;
  }
  else
  if (bUartRxState == UART_STATE_WRITE_1)
  {
    iFlashData |= ((rx)&0xFF) << 8;
    bUartRxState = UART_STATE_WRITE_2;
  }
  else
  if (bUartRxState == UART_STATE_WRITE_2)
  {
    iFlashData |= ((rx)&0xFF) << 16;
    bUartRxState = UART_STATE_WRITE_3;
  }
  else
  if (bUartRxState == UART_STATE_WRITE_3)
  {
    iFlashData |= ((rx)&0xFF) << 24;
    bUartRxState = UART_STATE_WRITE_3;
    // writing data
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, iMemRWAddr, iFlashData) != HAL_OK)
    {
      // here is error situation
      LedR(1);
      bUARTMemTxResponse[2] = 'N';
      
      // lock flash control
      HAL_FLASH_Lock();

      // transmit back response
      CDC_Transmit_FS(bUARTMemTxResponse, 3);
      
      bUartRxState = UART_STATE_IDLE;
    }
    
    // incrementing address
    iMemRWByteCount = iMemRWByteCount - 4;
    iMemRWAddr = iMemRWAddr + 4;
    
    if (iMemRWByteCount <= 0)
    {
      // programming finished
      bUARTMemTxResponse[2] = 'Y';
      
      // lock flash control
      HAL_FLASH_Lock();

      // transmit back response 
      CDC_Transmit_FS(bUARTMemTxResponse, 3);
      
      bUartRxState = UART_STATE_IDLE;
    }
    else
      bUartRxState = UART_STATE_WRITE_0;
  }
}

//----------------------------------------------------------------------------------------
// Functional tests for transmission
//----------------------------------------------------------------------------------------
// type of CAN ID for tests
// #define TEST_STD // if not defined, then Extended ID used

// if defined CAN1 constantly transmits the data
// #define TEST_CAN1_TRANSMIT

// if defined CAN2 constantly transmits the data
//#define TEST_CAN2_TRANSMIT

// if defined then we waiting for received data and then checks the contents (used in TEST_CANX_TRANSMIT)
//#define TEST_LOOPBACK_CHECK

// incremented data and ID then masked to constrain the range
//#define TEST_TRANSMIT_MASK 0xF
#define TEST_TRANSMIT_MASK 0xFFFFFFFF

// if defined CAN1 transmits one packet every 1s
//#define TEST_CAN1_INJECTION

// if defined CAN2 transmits one packet every 1s
//#define TEST_CAN2_INJECTION

// if defined then no transmission
//#define TEST_LISTEN_ONLY

// if defined leds blink
//#define TEST_BLINKY

void RunTests()
{
#if defined(TEST_CAN1_INJECTION) || defined(TEST_CAN2_INJECTION) || defined(TEST_CAN1_TRANSMIT) || defined(TEST_CAN2_TRANSMIT)
  uint32_t tickstart = 0;
  CanTxMsgTypeDef *pTxMsg;
  CanRxMsgTypeDef *pRxMsg;
  CAN_HandleTypeDef *hcan;
#endif
  
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
  
  #ifdef TEST_LISTEN_ONLY
    while (1) if (RxQueueNotEmpty()) UART_ProcessData(RxQueueGet());
  #endif
  

  #if defined(TEST_CAN1_INJECTION) || defined(TEST_CAN2_INJECTION)
    // selecting buffer and CAN instance 
    #if defined(TEST_CAN1_INJECTION)
      pTxMsg = hcan1.pTxMsg;
      hcan = &hcan1;
    #else        
      pTxMsg = hcan2.pTxMsg;
      hcan = &hcan2;
    #endif
  
    pTxMsg->StdId = 0x7EA;
    pTxMsg->ExtId = 0x7EA;
    pTxMsg->IDE = CAN_ID_STD;
    pTxMsg->DLC = 8;
    pTxMsg->Data[7] = 0x44;
    pTxMsg->Data[6] = 0x54;
    pTxMsg->Data[5] = 0x4A;
    pTxMsg->Data[4] = 0x01;
    pTxMsg->Data[3] = 0x02;
    pTxMsg->Data[2] = 0x49;
    pTxMsg->Data[1] = 0x14;
    pTxMsg->Data[0] = 0x10;
  
    #ifdef TEST_STD
      pTxMsg->IDE = CAN_ID_STD;
    #else
      pTxMsg->IDE = CAN_ID_EXT;
    #endif
  
    while (1)
    {
      if (HAL_CAN_Transmit_IT(hcan) != HAL_OK)
      {
        LedR_Toggle();
      }
      
      // waiting one second
      tickstart = HAL_GetTick();
      while((HAL_GetTick() - tickstart) < 1000)
      {
        if (RxQueueNotEmpty()) UART_ProcessData(RxQueueGet());
      }
    }
  #endif

  #if defined(TEST_CAN1_TRANSMIT) || defined(TEST_CAN2_TRANSMIT)
    // selecting buffer and CAN instance 
    #if defined(TEST_CAN1_TRANSMIT)
      pTxMsg = hcan1.pTxMsg;
      pRxMsg = hcan1.pRxMsg;
      hcan = &hcan1;
    #else        
      pTxMsg = hcan2.pTxMsg;
      pRxMsg = hcan2.pRxMsg;
      hcan = &hcan2;
    #endif
    
    pTxMsg->StdId = 0;
    pTxMsg->ExtId = 1;
    pTxMsg->IDE = CAN_ID_STD;
    pTxMsg->DLC = 8;
    pTxMsg->Data[0] = 0;
    pTxMsg->Data[1] = 1;
    pTxMsg->Data[2] = 2;
    pTxMsg->Data[3] = 3;
    pTxMsg->Data[4] = 4;
    pTxMsg->Data[5] = 5;
    pTxMsg->Data[6] = 6;
    pTxMsg->Data[7] = 7;
    
    #ifdef TEST_STD
      pTxMsg->IDE = CAN_ID_STD;
    #else
      pTxMsg->IDE = CAN_ID_EXT;
    #endif
    
    while (1)
    {
      #if defined(TEST_CAN1_TRANSMIT)
        bCAN2_TxReq = 0; 
      #else
        bCAN1_TxReq = 0; 
      #endif
      
        DEBUG_MSG("ErrorCode: %x", (int)hcan->ErrorCode);
        DEBUG_MSG("State: %x", (int)hcan->State);
      // sending
      //if (HAL_CAN_Transmit_IT(hcan) != HAL_OK)
      if (HAL_CAN_Transmit(hcan, 10) != HAL_OK)
      {
        LedR_Toggle();
      }
      
      #ifdef TEST_LOOPBACK_CHECK
        // waiting for reception
        #if defined(TEST_CAN1_TRANSMIT)
          while (bCAN2_TxReq == 0) 
        #else
          while (bCAN1_TxReq == 0) 
        #endif
        { 
          if (RxQueueNotEmpty()) UART_ProcessData(RxQueueGet()); 
        };
        
        #ifdef TEST_STD
          if (pTxMsg->StdId != pRxMsg->StdId) LedR(1);
        #else
          if (pTxMsg->ExtId != pRxMsg->ExtId) LedR(1);
        #endif
        
        if (pTxMsg->Data[0] != pRxMsg->Data[0]) LedR(1);
        if (pTxMsg->Data[1] != pRxMsg->Data[1]) LedR(1);
        if (pTxMsg->Data[2] != pRxMsg->Data[2]) LedR(1);
        if (pTxMsg->Data[3] != pRxMsg->Data[3]) LedR(1);
        if (pTxMsg->Data[4] != pRxMsg->Data[4]) LedR(1);
        if (pTxMsg->Data[5] != pRxMsg->Data[5]) LedR(1);
        if (pTxMsg->Data[6] != pRxMsg->Data[6]) LedR(1);
        if (pTxMsg->Data[7] != pRxMsg->Data[7]) LedR(1);
      #endif
        
      pTxMsg->StdId++;  pTxMsg->StdId &= ((1 << 11)-1);
      pTxMsg->ExtId++;  pTxMsg->ExtId &= ((1 << 24)-1);
      pTxMsg->Data[0]++;
      pTxMsg->Data[1]++;
      pTxMsg->Data[2]++;
      pTxMsg->Data[3]++;
      pTxMsg->Data[4]++;
      pTxMsg->Data[5]++;
      pTxMsg->Data[6]++;
      pTxMsg->Data[7]++;

      pTxMsg->StdId &= TEST_TRANSMIT_MASK;
      pTxMsg->ExtId &= TEST_TRANSMIT_MASK;
//if (pTxMsg->StdId & 0x8) pTxMsg->StdId |= 0x100; if (pTxMsg->ExtId & 0x8) pTxMsg->ExtId |= 0x100;
      pTxMsg->Data[0] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[1] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[2] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[3] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[4] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[5] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[6] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[7] &= TEST_TRANSMIT_MASK;
      
      // waiting one second
      tickstart = HAL_GetTick();
      while((HAL_GetTick() - tickstart) < 1000)
      {
        if (rx_queue_not_empty()) UART_ProcessData(rx_queue_get());
      }
    }
  #endif
}

#if 0
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
#else
void __Error_Handler(const char *func, int line)
{
#endif
  /* User can add his own implementation to report the HAL error return state */
  if (gSystemInitialized == 1)
  {
    char msg[256];
    snprintf(msg, sizeof(msg), "\n\rError handle at : %s:%d\n\r", func, line);
	  LedR(1);
	  infinite_message(msg);
  }
  else
  {
    printf("\n\rError handle at : %s:%d\n\r", func, line);
  }
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
  /* User can add his own implementation to report the file name and line number */
    printf("Wrong parameters value: file %s on line %d\r\n", file, (int)line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
