/******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "bootloader-config.h"
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_storage_if.h"
#include "xmodem.h"
#include "flash.h"
#include "boot.h"
#include "filesystem.h"
#include "cardreader.h"
#include "uart.h"
#include "rx_queue.h"
#include "nvs.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypeDef SDCardInfo;
UART_HandleTypeDef huart3;
extern USBD_HandleTypeDef hUsbDeviceFS;

volatile uint8_t gSystemInitialized = 0;

uint8_t USB_rx_buffer[0x200];
volatile uint32_t USB_rx_buffer_lead_ptr = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void RunTests(void);
void usb_rx_process(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  uint8_t run_mode = LOADER_MODE_APP;
  uint8_t flash_proto = LOADER_PROTO_XMODEM;
  uint32_t bootaddr = FLASH_APP_START_ADDRESS;
  uint16_t size = 0;

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
#if defined(UART_ENABLED)
  MX_USART3_UART_Init();
#endif
  nvs_init();
  gSystemInitialized = 1;

  // Reading state from MCU flash's NVR page
  if (nvs_get("boot", &run_mode, &size, 1) == KEY_NOT_FOUND
      || nvs_get("proto", &run_mode, &size, 1) == KEY_NOT_FOUND)
  {
    INFO_MSG("Set default configuration.");
    run_mode = LOADER_MODE_FLASH;
    flash_proto = LOADER_PROTO_XMODEM;
    nvs_clear();
    nvs_put("boot", &run_mode, 1, 1);
    nvs_put("proto", &flash_proto, 1, 1);
    if (nvs_commit() != NVS_OK)
    {
      INFO_MSG("flash commit failed");
    }
  }

#if defined(UART_ENABLED)
  printf("================================\n\r");
  printf("USB/Xmodem/MassStorage Bootloader\n\r");
  printf("https://github.com/nwlab/CAN_Bridge\n\r");
  printf("Based on https://github.com/ferenc-nemeth/stm32-bootloader\n\r");
  printf("================================\n\r");
  printf("Compiled : " __DATE__ ", " __TIME__ "\n\r");
  INFO_MSG("CPU speed  : %ld MHz", SystemCoreClock / 1000000);
  INFO_MSG("Flash size : %d kB", *(uint16_t* )(FLASHSIZE_BASE));
  INFO_MSG("UID [HEX]  : %04lx %04lx %04lx", *(uint32_t* )(UID_BASE),
      *(uint32_t* )(UID_BASE + 0x04), *(uint32_t* )(UID_BASE + 0x08));
  INFO_MSG("Flash base address : 0x%08lX", FLASH_BASE);
  INFO_MSG("Flash size :         0x%08lX", (FLASH_END - FLASH_BASE));
  INFO_MSG("User application start address : 0x%08lX", FLASH_APP_START_ADDRESS);
  INFO_MSG("User application start sector  : %d", FLASH_APP_START_SECTOR);
  INFO_MSG("Configuration address          : 0x%08lX", FLASH_CFG_START_ADDRESS);
  INFO_MSG("Run mode    : %s",
      run_mode==LOADER_MODE_APP?"App":(run_mode==LOADER_MODE_FLASH?"Flasher":"Unknown"));
  INFO_MSG("Flash proto : %s",
      flash_proto==LOADER_PROTO_XMODEM?"xmodem":(flash_proto==LOADER_PROTO_STORAGE?"storage":"Unknown"));
  printf("================================\n\r");
#endif

  if (run_mode == LOADER_MODE_APP)
  {
    if (nvs_get("bootaddr", (uint8_t*) &bootaddr, &size, 4) == KEY_NOT_FOUND)
    {
      bootaddr = FLASH_APP_START_ADDRESS;
      nvs_put("bootaddr", (uint8_t*) &bootaddr, 4, 4);
    }
    INFO_MSG("Jumping to user application by addr : 0x%08lX...", bootaddr);
    jump_to_app(bootaddr);
  }

  if (flash_proto == LOADER_PROTO_XMODEM)
  {
    INFO_MSG("Starting CDC USB.");
    MX_USB_DEVICE_Init();
    /* Infinite loop */
    while (1)
    {
      /* Turn on the green LED to indicate, that we are in bootloader mode.*/
      HAL_GPIO_WritePin(LEDR_GPIO, LEDR_PIN, GPIO_PIN_SET);
      /* Ask for new data and start the Xmodem protocol. */
      printf(
          "Please send a new binary file with Xmodem protocol to update the firmware.\n\r");
      xmodem_process();
      /* We only exit the xmodem protocol, if there are any errors.
       * In that case, notify the user and start over. */
      printf("\n\rFailed... Please try again.\n\r");
    }
  }

  if (flash_proto == LOADER_PROTO_STORAGE)
  {
#ifdef SDCARD_ENABLED
    // Ok, something was programmed
    // Enabling FAT FS
    if (init_filesystem() != FILESYSTEM_INIT_OK)
    {
      // Cannot mount SD-card and pick FAT fs for some reason
      // If nothing was programmed yet
      INFO_MSG("Cannot mount file system or find file with flash");
    }
    else
    {
      // We have properly mounted FAT fs and now we should check hash sum of flash.bin
      uint32_t storedHash, fileHash, trueHash, sizeHash;
      HashCheckResult hcr = check_flash_file(&fileHash, &trueHash, &sizeHash);
      switch (hcr)
      {
        case FLASH_FILE_OK:
          // Check if no upgrade was provided
          if (nvs_get("hash", (uint8_t*) &storedHash, &size, 4) == NVS_OK)
          {
            if (storedHash == fileHash)
            {
              INFO_MSG("Firmware is up-to-date, nothing to update");
              break;
            }
          }
          // Here we should flash our MCU
          flash_status fr = flash_file();
          switch (fr)
          {
            case FLASH_OK:
            {
              INFO_MSG("MCU successfuly programmed");
              FRESULT res = f_unlink(flashFileName);
              if (res != FR_OK)
              {
                INFO_MSG("Cannot delete firmware file");
              }

              run_mode = LOADER_MODE_APP;
              nvs_put("hash", (uint8_t*) &fileHash, 4, 4);
              nvs_put("boot", &run_mode, 1, 1);
              if (nvs_commit() != NVS_OK)
              {
                INFO_MSG("flash commit failed");
              }
              break;
            }
            case FLASH_ERROR_FILE:
              infinite_message("Flash error: cannot read file\n\r");
              break;
            case FLASH_ERROR_FLASH:
              infinite_message(
                  "Flash error: cannot write or erase flash memory. Maybe you MCU is totally broken\n\r");
              break;
            default:
              break;
          }
          break;
        case FLASH_FILE_NOT_EXISTS:
        {
          INFO_MSG("Flash file not exists on sd-card");
          break;
        }
        case FLASH_FILE_CANNOT_OPEN:
        {
          INFO_MSG("Cannot read from flash file");
          break;
        }
        case FLASH_FILE_INVALID_HASH:
        {
          INFO_MSG(
              "Flash file hash is invalid: %lu against %lu specified in .ly file",
              fileHash, trueHash);
          break;
        }
        case FLASH_FILE_TOO_BIG:
        {
          INFO_MSG(
              "Flash file is too big: %lu against %lu available in MCUDEBUG_MSG",
              sizeHash, flash_image_size());
          break;
        }
      }
      deinit_filesystem();
    }

    INFO_MSG("Starting MassStorage USB.");
    init_cardreader();
    infinite_message("System in USB device mode\n\r");
#else
    infinite_message("MassStorage not supported.\n\r");
#endif
  }

  RunTests(); // this is function to run transmission tests

}

uint32_t toggle_time_r = 0;
uint32_t toggle_time_g = 0;
uint32_t toggle_time_b = 0;

void main_tick_1ms()
{
}
void main_tick_5ms()
{
  usb_rx_process();
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
  }
  if (toggle_time_g > 0)
  {
    HAL_GPIO_TogglePin(LEDG_GPIO, LEDG_PIN);
    toggle_time_g--;
  }
  if (toggle_time_b > 0)
  {
    HAL_GPIO_TogglePin(LEDB_GPIO, LEDB_PIN);
    toggle_time_b--;
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

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct =
  { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct =
  { 0 };

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
void MX_SDIO_SD_Init(void)
{
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
//  hsd.Init.ClockDiv = 3;

  hsd.Init.ClockDiv = SDIO_INIT_CLK_DIV;
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = UART_BAUDRATE;
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
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct =
  { 0 };

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
      GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

//#define TEST_USB_TRANSMIT
// if defined leds blink
//#define TEST_BLINKY

/*
 *
 */
void RunTests()
{
#ifdef TEST_BLINKY

    while(1)
    {
       // Toggle all LED
      HAL_GPIO_TogglePin(LEDR_GPIO, LEDR_PIN);
      HAL_GPIO_TogglePin(LEDG_GPIO, LEDG_PIN);
      HAL_GPIO_TogglePin(LEDB_GPIO, LEDB_PIN);
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
    while (1) if (rx_queue_not_empty()) UART_ProcessData(rx_queue_get());
  #endif
}

/*
 *
 */
void usb_rx_process(void)
{
  if (USB_rx_buffer_lead_ptr > 0)
  {
    for (uint32_t i = 0; i < USB_rx_buffer_lead_ptr; i++)
    {
      rx_queue_put(USB_rx_buffer[i]);
    }
    USB_rx_buffer_lead_ptr = 0;
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }
}

/*
 *
 */
xmodem_status xmodem_receive(uint8_t *data, uint16_t length, uint32_t timeout)
{
  // loop to read bytes
  for (uint32_t i = 0; i < length; i++)
  {
    // Wait until we have at least 1 byte to read
    uint32_t start = HAL_GetTick();
    while (!rx_queue_not_empty())
    {
      // Wraparound of tick is taken care of by 2's complement arithmetic.
      if (HAL_GetTick() - start >= timeout)
      {
        // timeout
        DEBUG_MSG("UART receive timeout, length:%d, received:%ld", length, i);
        return X_ERROR_UART;
      }
    }

    // Copy byte from device to user buffer
    data[i] = rx_queue_get();
    toggle_time_g = 2;
  }
  return X_OK;
}

/*
 *
 */
xmodem_status xmodem_transmit_ch(uint8_t data)
{
  if (CDC_Transmit_FS(&data, 1) == USBD_OK)
  {
    toggle_time_b = 2;
    return X_OK;
  }
  DEBUG_MSG("UART transmit error.");
  return X_ERROR_UART;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  INFO_MSG("HAL ERROR");
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
  INFO_MSG("Wrong parameters value: file %s on line %d", file, line);
}
#endif /* USE_FULL_ASSERT */

/****END OF FILE****/
