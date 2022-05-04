/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//*******************************[Aliases}**************************************
#define ADC_RESOLUTION 4096 // adc resolution 12 bits
#define BUF_SIZE 256 // Size of usart transmit buffer
#define NUM_ADC_CHANNELS 8 // Number of adc channels used for battery voltage measurment [0 to 6 = 7 Total] + total battery voltage 
#define NUM_AVG 10 // Number of samples to average per adc channel
#define TOLAL_BAT_VOL_SCALE_FAC 11.357 // The scale factor calculated by 28.2v / 2.483 (At the time of measuring)
/* 
72Mhz / 36000 [PRESCALER] = 2Khz timer clock
1 / 2000 = 0.0005s aka 500us per ARR count
20000 * 0.0005s = 10s, so 20000 ARR counts = 10s delay, whats why INTERVAL is 20k
*/
#define INTERVAL 20000 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//*******************************[Global Flags]*********************************
uint8_t txempty;
uint8_t rxfull;
uint8_t startTransmit = 0;

//*******************************[Global Buffers]*******************************
uint8_t rxbuff[8];

//******************************[Global Variables]******************************
uint16_t adcChannelArray[NUM_ADC_CHANNELS]; // Used by dma to store each channel's val in a seperate element 
uint16_t batVoltage[NUM_ADC_CHANNELS][NUM_AVG]; // 2d array for bat voltage per channel of X samples 
uint16_t avgBatVoltage[NUM_ADC_CHANNELS]; // Final averaged bat voltage and initized with offset for each channel 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  char buffer[BUF_SIZE + 1]; // Buffer size = 8bits * buf_size
  memset(buffer, 0 , BUF_SIZE); // Init buffer
  
  const char format[] = "[ADC1] = %dmv, [ADC2] = %dmv, [ADC3] = %dmv, [ADC4] = %dmv, [ADC5] = %dmv, [ADC6] = %dmv, [ADC7] = %dmv, [BATVOL] = %dmv\r\n";
  
  HAL_StatusTypeDef ret; // Return error

  ret = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // Calibrate ADC on each Startup
  if(ret != HAL_OK) Error_Handler();

  ret = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcChannelArray, NUM_ADC_CHANNELS); // Attach adc to dma 
  if(ret != HAL_OK) Error_Handler();

  ret = HAL_TIM_Base_Start(&htim16); // Start timer 16
  if(ret != HAL_OK) Error_Handler();
  
  txempty = 1; // Init tx flag
  uint16_t timerVal = __HAL_TIM_GET_COUNTER(&htim16); // Init timerVal

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(__HAL_TIM_GET_COUNTER(&htim16) - timerVal >= INTERVAL && startTransmit == 1)
    {      
      sprintf(buffer, format, avgBatVoltage[0], avgBatVoltage[1], avgBatVoltage[2], avgBatVoltage[3], avgBatVoltage[4], avgBatVoltage[5], avgBatVoltage[6], avgBatVoltage[7]);
      
      startTransmit = 0; // Reset flag
      while(txempty == 0);
      ret = HAL_UART_Transmit_DMA(&huart4, (uint8_t *)buffer, BUF_SIZE);
      if(ret != HAL_OK) Error_Handler();
      txempty = 0;

      ret = HAL_ADC_Start(&hadc1);
      if(ret != HAL_OK) Error_Handler();
      
      timerVal = __HAL_TIM_GET_COUNTER(&htim16); // Update with current timer count
    }

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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//***********************[ISRs}**********************************
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc == &hadc1)
  {
    uint32_t sum = 0;

    // Building a 7x10 matrix 
    for(int j = 0; j < NUM_AVG; ++j)
    {
      for(int i = 0; i < NUM_ADC_CHANNELS; ++i)
        {
          if(i == 7)
            batVoltage[i][j] = TOLAL_BAT_VOL_SCALE_FAC * (2965UL * adcChannelArray[i] / ADC_RESOLUTION); // Conversion of Total battery voltage
          else
            batVoltage[i][j] = 2 * (3000UL * adcChannelArray[i] / ADC_RESOLUTION); // Conversion of battery bank 1-7 and scaling it back after dividing it in hardware
        }
    }

    // Taking average of each colume 
    for(int x = 0; x < NUM_ADC_CHANNELS; ++x)
    {
      for(int y = 0; y < NUM_AVG; ++y)
        sum += batVoltage[x][y];
      
      // Adding offsets to each battery banks
      switch (x)
      {
      case 0:
        avgBatVoltage[x] = 110 + (sum / NUM_AVG);
        break;
      
      case 1:
        avgBatVoltage[x] = 150 + (sum / NUM_AVG);
        break;

      case 2:
        avgBatVoltage[x] = 130 + (sum / NUM_AVG);
        break;
      
      case 3:
        avgBatVoltage[x] = 70 + (sum / NUM_AVG);
        break;

      case 4:
        avgBatVoltage[x] = (sum / NUM_AVG) - 40;
        break;
      
      case 5:
        avgBatVoltage[x] = (sum / NUM_AVG) - 115;
        break;
      
      case 6:
        avgBatVoltage[x] = 340 + (sum / NUM_AVG);
        break;

      default:
        avgBatVoltage[x] = sum / NUM_AVG; // This one is for the total battery voltage reading
      }
      sum = 0; // Reset sum back to zero
    }

    startTransmit = 1; // Setting the transmit ready flag
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart4)
	{
		rxfull = 1;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart4)
	{
		txempty = 1;
	}
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
