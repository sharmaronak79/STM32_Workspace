/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_host.h"
#include<string.h>
#include<stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);

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

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USB_HOST_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();

  uint8_t R44 = 0x2C;
  uint8_t R44_data[2]={0x00,0x00};

  uint8_t R45 = 0x2D;
  uint8_t R45_data[2]={0x00,0x00};

  uint8_t R36 = 0x24;
  uint8_t R36_data[2]={0x00,0x1E};

  uint8_t R43 = 0x2B;
  uint8_t R43_data[2]={0x00,0x00};

  uint8_t R0 = 0x00;
  uint8_t R0_data[2]={0x64,0x70};

  uint8_t Rx_Data[2]={0x00,0x00};

  uint8_t Buf[]="Application is Running\n";
  HAL_UART_Transmit(&huart3, Buf, strlen((char *)Buf), 1000);


  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
  while (1)
  {
	  while(! HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
	  HAL_Delay(200);


	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &R44, 1, 1000);
	  HAL_SPI_Transmit(&hspi1, R44_data, 2, 1000);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &R45, 1, 1000);
	  HAL_SPI_Transmit(&hspi1, R45_data, 2, 1000);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &R36, 1, 1000);
	  HAL_SPI_Transmit(&hspi1, R36_data, 2, 1000);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &R43, 1, 1000);
	  HAL_SPI_Transmit(&hspi1, R43_data, 2, 1000);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &R0, 1, 1000);
	  HAL_SPI_Transmit(&hspi1, R0_data, 2, 1000);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

	  strcpy((char *)Buf,"Programming is Done\n");
	  HAL_UART_Transmit(&huart3, Buf, strlen((char *)Buf), 1000);

	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_SPI_Receive(&hspi1, &R0, 1, 1000);
	  HAL_SPI_Receive(&hspi1,Rx_Data, 2, 1000);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

	  sprintf((char *)Buf,"Data is : 0x%x 0x%x",Rx_Data[0],Rx_Data[1]);
	  HAL_UART_Transmit(&huart3, Buf, strlen((char *)Buf), 1000);

  }

}

/*
 *
 *Register values for Reset GUI
  uint8_t R44 = 0x2C;
  uint8_t R44_data[2]={0x00,0x00};

  uint8_t R45 = 0x2D;
  uint8_t R45_data[2]={0x00,0x00};

  uint8_t R36 = 0x24;
  uint8_t R36_data[2]={0x00,0x1E};

  uint8_t R43 = 0x2B;
  uint8_t R43_data[2]={0x00,0x00};

  uint8_t R0 = 0x00;
  uint8_t R0_data[2]={0x64,0x70};

  //Register value for 100 MHz
   *
  uint8_t R44 = 0x2C;
  uint8_t R44_data[2]={0x00,0x00};

  uint8_t R45 = 0x2D;
  uint8_t R45_data[2]={0x00,0x00};

  uint8_t R36 = 0x24;
  uint8_t R36_data[2]={0x00,0x20};

  uint8_t R43 = 0x2B;
  uint8_t R43_data[2]={0x00,0x00};

  uint8_t R0 = 0x00;
  uint8_t R0_data[2]={0x64,0x70};


  //Register value for 90 MHz
   *
  uint8_t R44 = 0x2C;
  uint8_t R44_data[2]={0xCC,0xCC};

  uint8_t R45 = 0x2D;
  uint8_t R45_data[2]={0xCC,0xCD};

  uint8_t R36 = 0x24;
  uint8_t R36_data[2]={0x00,0x1C};

  uint8_t R43 = 0x2B;
  uint8_t R43_data[2]={0x01,0x40};

  uint8_t R0 = 0x00;
  uint8_t R0_data[2]={0x64,0x70};

 */





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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 71;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin|SMPS_V1_Pin|SMPS_EN_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OverCurrent_Pin SMPS_PG_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin|SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_PowerSwitchOn_Pin SMPS_V1_Pin SMPS_EN_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin|SMPS_V1_Pin|SMPS_EN_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
