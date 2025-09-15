/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <sys/types.h>

#include "global.h"
#include "map.h"
#include "usbd_customhid.h"
#include "utils.h"
#include "at24cxx/at24cxx.h"
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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId HidCommandTaskHandle;
uint32_t HidCommandBuffer[ 128 ];
osStaticThreadDef_t HidCommandControlBlock;
osThreadId HidReportTaskHandle;
uint32_t HidReportTaskBuffer[ 128 ];
osStaticThreadDef_t HidReportTaskControlBlock;
osThreadId MCP3208ReaderHandle;
uint32_t ReadFromMCP3208Buffer[ 64 ];
osStaticThreadDef_t ReadFromMCP3208ControlBlock;
osThreadId ReadClibTaskHandle;
uint32_t ReadClibTaskBuffer[ 64 ];
osStaticThreadDef_t ReadClibTaskControlBlock;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void const * argument);
void HidCommand(void const * argument);
void HidReport(void const * argument);
void ReadFromMCP3208(void const * argument);
void ReadClib(void const * argument);

/* USER CODE BEGIN PFP */

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
  slipAvgFilter_Ch = malloc(sizeof(SlipAvgFilter_t) * 8);
  for (int i = 0; i < 8; i++)
  {
    slipAvgFilter_Ch[i] = SlipAvgFilterCreate(8);
  }
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  hid_rx_queue = xQueueCreate(2, sizeof(HidOutEvent_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of HidCommandTask */
  osThreadStaticDef(HidCommandTask, HidCommand, osPriorityBelowNormal, 0, 128, HidCommandBuffer, &HidCommandControlBlock);
  HidCommandTaskHandle = osThreadCreate(osThread(HidCommandTask), NULL);

  /* definition and creation of HidReportTask */
  osThreadStaticDef(HidReportTask, HidReport, osPriorityHigh, 0, 128, HidReportTaskBuffer, &HidReportTaskControlBlock);
  HidReportTaskHandle = osThreadCreate(osThread(HidReportTask), NULL);

  /* definition and creation of MCP3208Reader */
  osThreadStaticDef(MCP3208Reader, ReadFromMCP3208, osPriorityNormal, 0, 64, ReadFromMCP3208Buffer, &ReadFromMCP3208ControlBlock);
  MCP3208ReaderHandle = osThreadCreate(osThread(MCP3208Reader), NULL);

  /* definition and creation of ReadClibTask */
  osThreadStaticDef(ReadClibTask, ReadClib, osPriorityBelowNormal, 0, 64, ReadClibTaskBuffer, &ReadClibTaskControlBlock);
  ReadClibTaskHandle = osThreadCreate(osThread(ReadClibTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  MCP3208_Init(&mcp3208, &hspi2, MCP3208_CS_GPIO_Port, MCP3208_CS_Pin);
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AT24_WP_GPIO_Port, AT24_WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCP3208_CS_GPIO_Port, MCP3208_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AT24_WP_Pin */
  GPIO_InitStruct.Pin = AT24_WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AT24_WP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCP3208_CS_Pin */
  GPIO_InitStruct.Pin = MCP3208_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MCP3208_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
      HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
      osDelay(pdMS_TO_TICKS(500));
    }else
    {
      HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
      osDelay(pdMS_TO_TICKS(50));
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_HidCommand */
/**
* @brief Function implementing the HidCommandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HidCommand */
void HidCommand(void const * argument)
{
  /* USER CODE BEGIN HidCommand */
  HidOutEvent_t out_event;
  uint8_t buf[2];
  /* Infinite loop */
  for(;;)
  {
     if (xQueueReceive(hid_rx_queue, &out_event, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            switch (out_event.event_idx)
            {
            case 0x21: // 校准命令
                {
                    switch (out_event.state)
                    {
                    case 0x03:
                        Uint16ToUint8Array(slipAvgFilter_Ch[0].avg, buf, 1);
                        at24_write(RUDDER_INPUT_MIN_OFFSET_ADDR, buf, 2, 100);
                        break;
                    case 0x04:
                        Uint16ToUint8Array(slipAvgFilter_Ch[0].avg, buf, 1);
                        at24_write(RUDDER_INPUT_MAX_OFFSET_ADDR, buf, 2, 100);
                        break;
                    case 0x05:
                        Uint16ToUint8Array(slipAvgFilter_Ch[4].avg, buf, 1);
                        at24_write(PIC_LEFT_BREAK_INPUT_MIN_OFFSET_ADDR, buf, 2, 100);
                        break;
                    case 0x06:
                        Uint16ToUint8Array(slipAvgFilter_Ch[4].avg, buf, 1);
                        at24_write(PIC_LEFT_BREAK_INPUT_MAX_OFFSET_ADDR, buf, 2, 100);
                        break;
                    case 0x07:
                        Uint16ToUint8Array(slipAvgFilter_Ch[5].avg, buf, 1);
                        at24_write(PIC_RIGHT_BREAK_INPUT_MIN_OFFSET_ADDR, buf, 2, 100);
                        break;
                    case 0x08:
                        Uint16ToUint8Array(slipAvgFilter_Ch[5].avg, buf, 1);
                        at24_write(PIC_RIGHT_BREAK_INPUT_MAX_OFFSET_ADDR, buf, 2, 100);
                        break;
                    case 0x09:
                        Uint16ToUint8Array(slipAvgFilter_Ch[6].avg, buf, 1);
                        at24_write(PF_LEFT_BREAK_INPUT_MIN_OFFSET_ADDR, buf, 2, 100);
                        break;
                    case 0x10:
                        Uint16ToUint8Array(slipAvgFilter_Ch[6].avg, buf, 1);
                        at24_write(PF_LEFT_BREAK_INPUT_MAX_OFFSET_ADDR, buf, 2, 100);
                        break;
                    case 0x11:
                      Uint16ToUint8Array(slipAvgFilter_Ch[7].avg, buf, 1);
                      at24_write(PF_RIGHT_BREAK_INPUT_MIN_OFFSET_ADDR, buf, 2, 100);
                      break;
                    case 0x12:
                      Uint16ToUint8Array(slipAvgFilter_Ch[7].avg, buf, 1);
                      at24_write(PF_RIGHT_BREAK_INPUT_MAX_OFFSET_ADDR, buf, 2, 100);
                      break;
                    default:
                        break;
                    }
                }
            default:
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
  }
  /* USER CODE END HidCommand */
}

/* USER CODE BEGIN Header_HidReport */
/**
* @brief Function implementing the HidReportTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HidReport */
void HidReport(void const * argument)
{
  /* USER CODE BEGIN HidReport */
  report14.report_id = 0x14;
  /* Infinite loop */
  for(;;)
  {
    const uint16_t rz = map(
            slipAvgFilter_Ch[0].avg,
            rudder_input_min_clib, rudder_input_max_clib,
            0, (1 << 10) - 1);
    report14.rz = rz;

    const uint16_t pic_left = map(
            slipAvgFilter_Ch[4].avg,
            pic_left_break_input_min_clib, pic_left_break_input_max_clib,
            0, (1 << 10) - 1);
    const uint16_t pf_left = map(
            slipAvgFilter_Ch[6].avg,
            pf_left_break_input_min_clib, pf_left_break_input_max_clib,
            0, (1 << 10) - 1);

    report14.dial = pic_left >= pf_left ? pic_left : pf_left;

    const uint16_t pic_right = map(
            slipAvgFilter_Ch[5].avg,
            pic_right_break_input_min_clib, pic_right_break_input_max_clib,
            0, (1 << 10) - 1);
    const uint16_t pf_right = map(
            slipAvgFilter_Ch[7].avg,
            pf_right_break_input_min_clib, pf_right_break_input_max_clib,
            0, (1 << 10) - 1);

    report14.slider = pic_right >= pf_right ? pic_right : pf_right;

    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report14, sizeof(report14));

    vTaskDelay(pdMS_TO_TICKS(20));
  }
  /* USER CODE END HidReport */
}

/* USER CODE BEGIN Header_ReadFromMCP3208 */
/**
* @brief Function implementing the MCP3208Reader thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadFromMCP3208 */
void ReadFromMCP3208(void const * argument)
{
  /* USER CODE BEGIN ReadFromMCP3208 */
  /* Infinite loop */
  uint16_t s, fs;
  for(;;)
  {
      s = MCP3208_ReadChannel(&mcp3208, 0);
      fs = SlipAvgFilterGetAvgWithInput(&slipAvgFilter_Ch[0], s);
      UNUSED(fs);
      vTaskDelay(pdMS_TO_TICKS(4));
      s = MCP3208_ReadChannel(&mcp3208, 4);
      fs = SlipAvgFilterGetAvgWithInput(&slipAvgFilter_Ch[4], s);
      UNUSED(fs);
      vTaskDelay(pdMS_TO_TICKS(4));
      s = MCP3208_ReadChannel(&mcp3208, 5);
      fs = SlipAvgFilterGetAvgWithInput(&slipAvgFilter_Ch[5], s);
      UNUSED(fs);
      vTaskDelay(pdMS_TO_TICKS(4));
      s = MCP3208_ReadChannel(&mcp3208, 6);
      fs = SlipAvgFilterGetAvgWithInput(&slipAvgFilter_Ch[6], s);
      UNUSED(fs);
      vTaskDelay(pdMS_TO_TICKS(4));
      s = MCP3208_ReadChannel(&mcp3208, 7);
      fs = SlipAvgFilterGetAvgWithInput(&slipAvgFilter_Ch[7], s);
      UNUSED(fs);
      vTaskDelay(pdMS_TO_TICKS(4));
  }
  /* USER CODE END ReadFromMCP3208 */
}

/* USER CODE BEGIN Header_ReadClib */
/**
* @brief Function implementing the ReadClibTask thread.
* @param argument: Not used
* @retval None
*/
typedef struct
{
  uint16_t at24_address;
  uint16_t* addr;
} read_clib_job_type;
/* USER CODE END Header_ReadClib */
void ReadClib(void const * argument)
{
  /* USER CODE BEGIN ReadClib */
  /* Infinite loop */
  int at24_read_timeout = 64;
  uint8_t buf[2];
  while (1)
  {
    at24_read(RUDDER_INPUT_MIN_OFFSET_ADDR, buf, 2, at24_read_timeout);
    if (buf[0] != 0xFF || buf[1] != 0xFF) rudder_input_min_clib = Uint8ArrayToUint16(buf, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    at24_read(RUDDER_INPUT_MAX_OFFSET_ADDR, buf, 2, at24_read_timeout);
    if (buf[0] != 0xFF || buf[1] != 0xFF) rudder_input_max_clib = Uint8ArrayToUint16(buf, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    at24_read(PIC_LEFT_BREAK_INPUT_MIN_OFFSET_ADDR, buf, 2, at24_read_timeout);
    if (buf[0] != 0xFF || buf[1] != 0xFF) pic_left_break_input_min_clib = Uint8ArrayToUint16(buf, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    at24_read(PIC_LEFT_BREAK_INPUT_MAX_OFFSET_ADDR, buf, 2, at24_read_timeout);
    if (buf[0] != 0xFF || buf[1] != 0xFF) pic_left_break_input_max_clib = Uint8ArrayToUint16(buf, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    at24_read(PIC_RIGHT_BREAK_INPUT_MIN_OFFSET_ADDR, buf, 2, at24_read_timeout);
    if (buf[0] != 0xFF || buf[1] != 0xFF) pic_right_break_input_min_clib = Uint8ArrayToUint16(buf, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    at24_read(PIC_RIGHT_BREAK_INPUT_MAX_OFFSET_ADDR, buf, 2, at24_read_timeout);
    if (buf[0] != 0xFF || buf[1] != 0xFF) pic_right_break_input_max_clib = Uint8ArrayToUint16(buf, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    at24_read(PF_LEFT_BREAK_INPUT_MIN_OFFSET_ADDR, buf, 2, at24_read_timeout);
    if (buf[0] != 0xFF || buf[1] != 0xFF) pf_left_break_input_min_clib = Uint8ArrayToUint16(buf, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    at24_read(PF_LEFT_BREAK_INPUT_MAX_OFFSET_ADDR, buf, 2, at24_read_timeout);
    if (buf[0] != 0xFF || buf[1] != 0xFF) pf_left_break_input_max_clib = Uint8ArrayToUint16(buf, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    at24_read(PF_RIGHT_BREAK_INPUT_MIN_OFFSET_ADDR, buf, 2, at24_read_timeout);
    if (buf[0] != 0xFF || buf[1] != 0xFF) pf_right_break_input_min_clib = Uint8ArrayToUint16(buf, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    at24_read(PF_RIGHT_BREAK_INPUT_MAX_OFFSET_ADDR, buf, 2, at24_read_timeout);
    if (buf[0] != 0xFF || buf[1] != 0xFF) pf_right_break_input_max_clib = Uint8ArrayToUint16(buf, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

  }
  /* USER CODE END ReadClib */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
