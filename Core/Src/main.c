/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Bluepill, send strings via uart in DMA, several DMA start
  ******************************************************************************
  *
  * Connect:
  *  - PA9 to FTDI chip RX pin in order to receive transmitted strings
  * During debug session:
  *  - create a console with the FTDI serial port at 115200 baud
  *
  * @attention
  *
  * Copyright (c) 2022 SZŐKE, Sándor (alexware)
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * It is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// CUSTOM CODE FOLLOWS
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// CUSTOM CODE FOLLOWS
#define TX_BUFFER_LENGTH  1024
#define DMA_TX_LENGTH     128
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
// CUSTOM CODE FOLLOWS
volatile uint8_t uart_tx_buffer[TX_BUFFER_LENGTH];
volatile uint8_t uart_tx_dma_buffer[DMA_TX_LENGTH];
volatile size_t uart_data_start;        // start offset of valid data in buffer
volatile size_t uart_data_len;          // length of valid data in buffer
volatile size_t uart_current_dma_len;   // current transfer during dma operation
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// CUSTOM CODE FOLLOWS
size_t send_debug_string(char * message);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// CUSTOM CODE FOLLOWS
  uart_data_start = 0;
  uart_data_len = 0;
  uart_current_dma_len = 0;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // CUSTOM CODE FOLLOWS
//send_debug_string("012345678901234567890123456789");  // DMA buffer play
  send_debug_string("This is the  1st line.\r\n");
  send_debug_string("This is the  2nd line..\r\n");
  send_debug_string("This is the  3rd line...\r\n");
  send_debug_string("This is the  4th line....\r\n");
  send_debug_string("This is the  5th line.....\r\n");
  send_debug_string("This is the  6th line......\r\n");
  send_debug_string("This is the  7th line.......\r\n");
  send_debug_string("This is the  8th line........\r\n");
  send_debug_string("This is the  9th line.........\r\n");
  send_debug_string("This is the 10th line..........\r\n");
  /* USER CODE END 2 */

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
  huart1.Init.Mode = UART_MODE_TX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUILTIN_LED_Pin */
  GPIO_InitStruct.Pin = BUILTIN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUILTIN_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// CUSTOM CODE FOLLOWS
// ---------------------------------------------------------------------------
/**
  * @brief Schedule next DMA session, can be called from IRQ
  * @param None
  * @retval None
  */
void DMA_send_next_buffer() // can be called from IRQ
{
  static size_t length;
  static size_t new_start;
  static size_t new_len;

  if(uart_current_dma_len == 0 && uart_data_len == 0)  // zero length transfer not fired
    return;

  if (uart_current_dma_len) //transfer in progress finished, buffer properties adjust needed
  {
    new_start = ((uart_data_start + uart_current_dma_len) >= TX_BUFFER_LENGTH)? 0 : (uart_data_start + uart_current_dma_len);
    new_len = uart_data_len -  uart_current_dma_len;
    uart_data_start = new_start;
    uart_data_len = new_len;
    uart_current_dma_len = 0;
  }
  // start a new transfer
  if (uart_data_start + uart_data_len > TX_BUFFER_LENGTH)   // check for wrap around...
    // only until the end of the puffer
    length = ((uart_data_start + uart_data_len - TX_BUFFER_LENGTH) > DMA_TX_LENGTH)? DMA_TX_LENGTH:(uart_data_start + uart_data_len - TX_BUFFER_LENGTH);
  else
    // length limited only by DMA length
    length = (uart_data_len > DMA_TX_LENGTH)? DMA_TX_LENGTH : uart_data_len;

  uart_current_dma_len = length;
  HAL_UART_Transmit_DMA(&huart1, ( uint8_t *) uart_tx_buffer + uart_data_start, uart_current_dma_len);
}
// ---------------------------------------------------------------------------
/**
  * @brief put message into puffer, wait if not enough free space (6ms)
  * @param message: string to be transmitted via uart
  * @retval size_t: number of characters inserted, must match length of message
  */
size_t send_debug_string(char * message)  //
{
  static size_t free_space;   // available free space in buffer
  static size_t message_len;  // length of current message
  static size_t copied;       // the already copied number of characters
  static size_t to_copy_start;// where to copy first character
  static size_t to_copy_len;  // how much character to copy

  message_len = strlen(message);
  copied = 0;
  while(copied < message_len)
  {
    __disable_irq();
    free_space = TX_BUFFER_LENGTH - uart_data_len;
    if (free_space > 0)
    {
      to_copy_start = ((uart_data_start + uart_data_len) >= TX_BUFFER_LENGTH)? ((uart_data_start + uart_data_len) - TX_BUFFER_LENGTH):(uart_data_start + uart_data_len);
      to_copy_len = MIN(((to_copy_start + message_len - copied) > TX_BUFFER_LENGTH) ? (TX_BUFFER_LENGTH - to_copy_start) : (message_len - copied), free_space) ;
      memcpy(uart_tx_buffer + to_copy_start, message + copied, to_copy_len);
      uart_data_len += to_copy_len;
      copied += to_copy_len;
      free_space -= to_copy_len;
    }
    if (!uart_current_dma_len)  // no transfer is take place start a new one
      DMA_send_next_buffer();

    __enable_irq();
    if ((copied < message_len) && (free_space == 0))
      HAL_Delay(6);   // if freeRTOS is used than different function must be used
  }
  return copied;
}
// ---------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
 DMA_send_next_buffer();
}
// ---------------------------------------------------------------------------
/* USER CODE END 4 */

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
