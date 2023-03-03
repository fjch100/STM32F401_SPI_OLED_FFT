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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"usbd_cdc_if.h"
#include "fft_stm32.h"
#include <string.h>
#include <stdio.h>
#include "adcFC.h"
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
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
float result;
char buffer[80];
volatile char snd_pin_activated;
uint32_t ticks, actualTick, previousTick;
void func_test_fft(void);
void clearBuffer(char *bu);
void creaData(void);
/*
These values can be changed in order to evaluate the functions
*/
#define NUMSAMPLES 64
uint16_t mysamples = NUMSAMPLES; //This value MUST ALWAYS be a power of 2
double mysignalFrequency = 1000;
double sampfrequency = 5000;
uint8_t myamplitude = 100;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double varReal1[NUMSAMPLES];
double varImag1[NUMSAMPLES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
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
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  previousTick = HAL_GetTick();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 // creaData();
	 // func_test_fft();
	 // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  if (snd_pin_activated){
		 snd_pin_activated=0;
		 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		 clearBuffer(buffer);
//		 sprintf(buffer,"%s", "Computed magnitudes: \r\n");
//		 CDC_Transmit_FS(buffer, strlen(buffer));
	  }
	  HAL_Delay(200);
	  /*
	  result = AdcRead(0, 100)* FMULT;
	  sprintf(buffer,"Temp: %.2f Cent \r\n", result);
	  CDC_Transmit_FS(buffer, strlen(buffer));


	  HAL_Delay(400);
	  actualTick = HAL_GetTick();
	  ticks = actualTick - previousTick;
	  previousTick = HAL_GetTick();
	  sprintf(buffer,"Ticks: %ld \r\n", ticks);
	  CDC_Transmit_FS(buffer, strlen(buffer));
	   */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SND_Pin */
  GPIO_InitStruct.Pin = SND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SND_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	snd_pin_activated=1;
}


void func_test_fft(){
	previousTick = HAL_GetTick();

/******* Sent to USB Input Real Data *******************
	clearBuffer(buffer);
	sprintf(buffer,"%s", "Data: \r\n");
	CDC_Transmit_FS(buffer, strlen(buffer));
	PrintVector(varReal1, mysamples, SCL_TIME, mysamples, sampfrequency);
*/

	Windowing(varReal1, mysamples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);

/********* Sent to USB Weighted (after windowing) Input Data
//	clearBuffer(buffer);
//	sprintf(buffer,"%s", "Weighed data: \r\n");
//	CDC_Transmit_FS(buffer, strlen(buffer));
//	PrintVector(varReal1, mysamples, SCL_TIME, mysamples, sampfrequency);
*/

	ComputeFFT(varReal1, varImag1, mysamples, FFT_FORWARD);

/******** Sent to USB computed FFT Real & Imaginary Data
	clearBuffer(buffer);
	sprintf(buffer,"%s", "Computed Real values: \r\n");
	CDC_Transmit_FS(buffer, strlen(buffer));
	PrintVector(varReal1, mysamples, SCL_INDEX, mysamples, sampfrequency);

	clearBuffer(buffer);
	sprintf(buffer,"%s", "Computed Imaginary values: \r\n");
	CDC_Transmit_FS(buffer, strlen(buffer));
	PrintVector(varImag1, mysamples, SCL_INDEX, mysamples, sampfrequency);
*/

	ComplexToMagnitude(varReal1, varImag1, mysamples);

	actualTick =  HAL_GetTick();
	ticks = actualTick - previousTick;

/******* Sent to USB Computed magnitudes **********/
	clearBuffer(buffer);
	sprintf(buffer,"%s", "Computed magnitudes: \r\n");
	CDC_Transmit_FS(buffer, strlen(buffer));
	PrintVector(varReal1, (mysamples >> 1), SCL_FREQUENCY, mysamples, sampfrequency);

	double x = MajorPeak(varReal1, mysamples, sampfrequency);

/******* Sent to USB Major Peak detected ********/
	clearBuffer(buffer);
	sprintf(buffer,"Major Peak:  %0.2f \r\n Ticks = %ld \r\n -----------------------\r\n", x, ticks);
	CDC_Transmit_FS(buffer, strlen(buffer));
}


void clearBuffer(char *bu){
	memset(bu, 0, 50);
}


void creaData(void){
 /* Build raw data */
	double cycles = (((mysamples-1) * mysignalFrequency) / sampfrequency); //Number of signal cycles that the sampling will read
	for (uint16_t i = 0; i < mysamples; i++)
	{
	  varReal1[i] = (myamplitude * (sin((i * (twoPi * cycles)) / mysamples))) / 2.0;/* Build data with positive and negative values*/
	  //vReal[i] = uint8_t((amplitude * (sin((i * (twoPi * cycles)) / samples) + 1.0)) / 2.0);/* Build data displaced on the Y axis to include only positive values*/
	  varImag1[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
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
