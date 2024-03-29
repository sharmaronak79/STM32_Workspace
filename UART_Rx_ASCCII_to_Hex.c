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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
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
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void LMX_default_RegConfig_Value_7point5_GHz(void);
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
	uint8_t Buf[100]="Application is Running";

	uint8_t Rx_Adr;
	uint8_t Rx_data[2];



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
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf),100);

  LMX_default_RegConfig_Value_7point5_GHz();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint8_t uart_rcv_data[8]={0};
	  uint8_t uart_rcv_data_adr[3]={0};
	  uint8_t Tx_reg[3]={0};
	  strcpy((char*)Buf,"\r\nEnter register number and data in 0xaadddd form : 0x");
	  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf), 1000);

	  HAL_UART_Receive(&huart1, uart_rcv_data, 6, HAL_MAX_DELAY);

	  for(int i =0;i<=6;i++){
		  if(uart_rcv_data[i]>=65 && uart_rcv_data[i]<=70){
			  uart_rcv_data[i]=uart_rcv_data[i]-55;
		  }else if(uart_rcv_data[i]>=97 && uart_rcv_data[i]<=102){
			  uart_rcv_data[i]=uart_rcv_data[i]-87;
		  }else if(uart_rcv_data[i]>=48 && uart_rcv_data[i]<=56){
			  uart_rcv_data[i]=uart_rcv_data[i]-48;
		  }else{
			  strcpy((char*)Buf,"\r\n something wrong in the Entry ");
			  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf), 1000);
		  }
	  }
	  Tx_reg[0]= (((uart_rcv_data[0]<<4) & 0xF0) | ((uart_rcv_data[1]) & 0x0F));
	  Tx_reg[1]= (((uart_rcv_data[2]<<4) & 0xF0) | ((uart_rcv_data[3]) & 0x0F));
	  Tx_reg[2]= (((uart_rcv_data[4]<<4) & 0xF0) | ((uart_rcv_data[5]) & 0x0F));

	  sprintf((char*)Buf,"\r\n Tx_reg[] = 0x%x %x %x",Tx_reg[0],Tx_reg[1],Tx_reg[2]);
	  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf), 1000);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); //CS pin low
	  HAL_SPI_Transmit(&hspi3, Tx_reg, 3, 100); // SPI Transimt
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); //CS high


//////////////////////////////Read the Address of the Register that you want to read from//////////////////////////


	  strcpy((char*)Buf,"\r\nEnter Read register Address :0x ");
	  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf), 1000);

	  HAL_UART_Receive(&huart1, uart_rcv_data_adr, 2, HAL_MAX_DELAY);
	  //Rx_Adr=(atoi((char*)uart_rcv_data_adr));
	  for(int i =0;i<=1;i++){
	 		  if(uart_rcv_data_adr[i]>=65 && uart_rcv_data_adr[i]<=70){
	 			  uart_rcv_data_adr[i]=uart_rcv_data_adr[i]-55;
	 		  }else if(uart_rcv_data_adr[i]>=97 && uart_rcv_data_adr[i]<=102){
	 			  uart_rcv_data_adr[i]=uart_rcv_data_adr[i]-87;
	 		  }else if(uart_rcv_data_adr[i]>=48 && uart_rcv_data_adr[i]<=56){
	 			  uart_rcv_data_adr[i]=uart_rcv_data_adr[i]-48;
	 		  }else{
	 			  strcpy((char*)Buf,"\r\n something wrong in the Entry ");
	 			  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf), 1000);
	 		  }
	 	  }
	  Rx_Adr= (((uart_rcv_data_adr[0]<<4) & 0xF0) | ((uart_rcv_data_adr[1]) & 0x0F));
	  Rx_Adr=Rx_Adr | 0x80;
	  sprintf((char*)Buf,"\r\n Rx_Adr = 0x%x",Rx_Adr);
	  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf), 1000);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); //CS pin low
	  HAL_SPI_Transmit(&hspi3, &Rx_Adr, 1, 100); // SPI Transimt
	  HAL_SPI_Receive(&hspi3, Rx_data, 2, 100);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); //CS high
	  sprintf((char*)Buf,"\n\rReceive data is : 0x%X 0x%X",Rx_data[0],Rx_data[1]);
	  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf), 100);



  }
  /* USER CODE END 3 */
}


void LMX_default_RegConfig_Value_7point5_GHz(void){

	int R[123]={0x7A0000 ,0x790000 ,0x780000 ,0x770009 ,0x760000 ,0x750000 ,0x740EEE ,0x73DF8F ,0x72C00B ,0x718221 ,0x70FFFF ,0x6F0000 ,0x6E001F ,0x6D0000 ,0x6C0000 ,0x6B0000 ,
			0x6A0000 ,0x69000A ,0x680014 ,0x670014 ,0x660028 ,0x6503E8 ,0x640533 ,0x6319B9 ,0x621C80 ,0x610000 ,0x6017F8 ,0x5F0000 ,0x5E0000 ,0x5D1000 ,0x5C0000 ,0x5B0000 ,
			0x5A0000 ,0x590000 ,0x5803FF ,0x57FF00 ,0x560040 ,0x550000 ,0x540040 ,0x530F00 ,0x520000 ,0x510000 ,0x5001C0 ,0x4F0112 ,0x4E0001 ,0x4D06CC ,0x4C0243 ,0x4B1350 ,
			0x4A97E5 ,0x499D7D ,0x480008 ,0x478801 ,0x46001E ,0x450011 ,0x440000 ,0x430000 ,0x42003F ,0x410001 ,0x404080 ,0x3FC350 ,0x3E0000 ,0x3D03E8 ,0x3C01F4 ,0x3B1388 ,
			0x3A0000 ,0x390001 ,0x380001 ,0x370002 ,0x360000 ,0x350000 ,0x340000 ,0x33203F ,0x320080 ,0x310000 ,0x304180 ,0x2F0300 ,0x2E0300 ,0x2D0000 ,0x2C0000 ,0x2B0000 ,
			0x2A0000 ,0x290000 ,0x280000 ,0x270168 ,0x260000 ,0x250500 ,0x240039 ,0x233140 ,0x220010 ,0x210000 ,0x201141 ,0x1F0401 ,0x1EB18C ,0x1D318C ,0x1C0639 ,0x1B8001 ,
			0x1A0DB0 ,0x190624 ,0x180E34 ,0x171103 ,0x162281 ,0x151C64 ,0x142750 ,0x132138 ,0x1203E8 ,0x111440 ,0x10271C ,0x0F2001 ,0x0E3001 ,0x0D0038 ,0x0C0408 ,0x0B0603 ,
			0x0A0880 ,0x090005 ,0x08C802 ,0x0700C8 ,0x060A43 ,0x053832 ,0x044204 ,0x030041 ,0x0281F4 ,0x01D7A0 ,0x004070 };

	uint8_t S_Buf[50];
	uint8_t Adr;
	uint8_t data[2];


	Adr=0x00;
	data[0]=0x00;
	data[1]=0x00;



	for(int8_t i=0;i<=15;i++)
		{
			//Bit wise operations to get the address and data to be sent
			Adr = (R[i] & 0xFF0000)>>16;
			data[0]=(R[i] & 0xFF00)>>8;
			data[1]=(R[i] & 0xFF);

			sprintf((char *)S_Buf,"Address and Data : 0x%X 0x%X 0x%X \n",Adr,data[0],data[1]);
			HAL_UART_Transmit(&huart1, S_Buf, strlen((char*)S_Buf),100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//CS Pin low
			HAL_SPI_Transmit(&hspi3, &Adr, 1, 100); //Transmit address
			HAL_SPI_Transmit(&hspi3, &data[0], 2, 100); //Transmit data
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS pin High
		}
	HAL_Delay(10);

	for(int8_t i=16;i<=31;i++)
		{
			//Bit wise operations to get the address and data to be sent
			Adr = (R[i] & 0xFF0000)>>16;
			data[0]=(R[i] & 0xFF00)>>8;
			data[1]=(R[i] & 0xFF);

			sprintf((char *)S_Buf,"Address and Data : 0x%X 0x%X 0x%X \n",Adr,data[0],data[1]);
			HAL_UART_Transmit(&huart1, S_Buf, strlen((char*)S_Buf),100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//CS Pin low
			HAL_SPI_Transmit(&hspi3, &Adr, 1, 100); //Transmit address
			HAL_SPI_Transmit(&hspi3, &data[0], 2, 100); //Transmit data
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS pin High
		}
	HAL_Delay(10);

	for(int8_t i=32;i<=47;i++)
		{
			//Bit wise operations to get the address and data to be sent
			Adr = (R[i] & 0xFF0000)>>16;
			data[0]=(R[i] & 0xFF00)>>8;
			data[1]=(R[i] & 0xFF);

			sprintf((char *)S_Buf,"Address and Data : 0x%X 0x%X 0x%X \n",Adr,data[0],data[1]);
			HAL_UART_Transmit(&huart1, S_Buf, strlen((char*)S_Buf),100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//CS Pin low
			HAL_SPI_Transmit(&hspi3, &Adr, 1, 100); //Transmit address
			HAL_SPI_Transmit(&hspi3, &data[0], 2, 100); //Transmit data
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS pin High
		}
	HAL_Delay(10);

	for(int8_t i=48;i<=63;i++)
		{
			//Bit wise operations to get the address and data to be sent
			Adr = (R[i] & 0xFF0000)>>16;
			data[0]=(R[i] & 0xFF00)>>8;
			data[1]=(R[i] & 0xFF);

			sprintf((char *)S_Buf,"Address and Data : 0x%X 0x%X 0x%X \n",Adr,data[0],data[1]);
			HAL_UART_Transmit(&huart1, S_Buf, strlen((char*)S_Buf),100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//CS Pin low
			HAL_SPI_Transmit(&hspi3, &Adr, 1, 100); //Transmit address
			HAL_SPI_Transmit(&hspi3, &data[0], 2, 100); //Transmit data
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS pin High
		}
	HAL_Delay(10);

	for(int8_t i=64;i<=79;i++)
		{
			//Bit wise operations to get the address and data to be sent
			Adr = (R[i] & 0xFF0000)>>16;
			data[0]=(R[i] & 0xFF00)>>8;
			data[1]=(R[i] & 0xFF);

			sprintf((char *)S_Buf,"Address and Data : 0x%X 0x%X 0x%X \n",Adr,data[0],data[1]);
			HAL_UART_Transmit(&huart1, S_Buf, strlen((char*)S_Buf),100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//CS Pin low
			HAL_SPI_Transmit(&hspi3, &Adr, 1, 100); //Transmit address
			HAL_SPI_Transmit(&hspi3, &data[0], 2, 100); //Transmit data
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS pin High
		}
	HAL_Delay(10);

	for(int8_t i=80;i<=95;i++)
		{
			//Bit wise operations to get the address and data to be sent
			Adr = (R[i] & 0xFF0000)>>16;
			data[0]=(R[i] & 0xFF00)>>8;
			data[1]=(R[i] & 0xFF);

			sprintf((char *)S_Buf,"Address and Data : 0x%X 0x%X 0x%X \n",Adr,data[0],data[1]);
			HAL_UART_Transmit(&huart1, S_Buf, strlen((char*)S_Buf),100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//CS Pin low
			HAL_SPI_Transmit(&hspi3, &Adr, 1, 100); //Transmit address
			HAL_SPI_Transmit(&hspi3, &data[0], 2, 100); //Transmit data
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS pin High
		}
	HAL_Delay(10);

	for(int8_t i=96;i<=111;i++)
		{
			//Bit wise operations to get the address and data to be sent
			Adr = (R[i] & 0xFF0000)>>16;
			data[0]=(R[i] & 0xFF00)>>8;
			data[1]=(R[i] & 0xFF);

			sprintf((char *)S_Buf,"Address and Data : 0x%X 0x%X 0x%X \n",Adr,data[0],data[1]);
			HAL_UART_Transmit(&huart1, S_Buf, strlen((char*)S_Buf),100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//CS Pin low
			HAL_SPI_Transmit(&hspi3, &Adr, 1, 100); //Transmit address
			HAL_SPI_Transmit(&hspi3, &data[0], 2, 100); //Transmit data
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS pin High
		}
	HAL_Delay(10);

	for(int8_t i=112;i<=122;i++)
		{
			//Bit wise operations to get the address and data to be sent
			Adr = (R[i] & 0xFF0000)>>16;
			data[0]=(R[i] & 0xFF00)>>8;
			data[1]=(R[i] & 0xFF);

			sprintf((char *)S_Buf,"Address and Data : 0x%X 0x%X 0x%X \n",Adr,data[0],data[1]);
			HAL_UART_Transmit(&huart1, S_Buf, strlen((char*)S_Buf),100);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//CS Pin low
			HAL_SPI_Transmit(&hspi3, &Adr, 1, 100); //Transmit address
			HAL_SPI_Transmit(&hspi3, &data[0], 2, 100); //Transmit data
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS pin High
		}
	HAL_Delay(10);


   strcpy((char *)S_Buf,"All registers are set for Default values \n");
   HAL_UART_Transmit(&huart1, S_Buf, strlen((char*)S_Buf),1000);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin|SMPS_V1_Pin|SMPS_EN_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
