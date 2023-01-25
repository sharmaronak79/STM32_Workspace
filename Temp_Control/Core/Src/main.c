/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
   @Author	    : Ronak Sharma
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
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

HAL_StatusTypeDef ret; // to store return value of communication function
uint8_t buf[50];
float last_Module_Temp=0;
float last_Room_Temp = 0;

//Variable for Temperature sensor ADT1407
static const uint8_t TMP1407_Module_ADDR = 0x48<<1;
static const uint8_t TMP1407_Room_ADDR = 0x49<<1;
static const uint8_t Temp_REG = 0x00;
float Module_Temp;
float Room_Temp;
float Set_Temp=40.000;
uint8_t Update_Rate = 30;

//Variable for Fan Controller
static const uint8_t FAN_ADDR = 0x2F<<1;
uint8_t read_TACH_High_Reg = 0x3E;
uint8_t read_TACH_Low_Reg = 0x3F;

float RPM;
float Target_RPM = 4200;
float last_RPM = 0;

uint8_t i=0;
float Sec0_Temp;
float Sec30_Temp;
float diff = 0.000;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static void MX_I2C2_Init(void);

static void MX_USART3_UART_Init(void);


/* USER CODE BEGIN PFP */
void Temp_Sensor_Init(void);
void Read_Temp(void);
void Fan_Control_Init(void);
void Read_RPM(void);
void Set_RPM(float Target_RPM);
void Set_Temp_RPM(float Set_Temperature);
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
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  Fan_Control_Init();
  Temp_Sensor_Init();
  /* USER CODE BEGIN 2 */


  strcpy((char*)buf, "Application is Running\r\n");
  HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */

   // while(! HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);


    Read_Temp();
    if((i==0) | (i==1)){
    	Sec0_Temp=Module_Temp;
    }else if(i % Update_Rate == 0){
    	Sec30_Temp = Module_Temp;
    	diff = Sec30_Temp - Sec0_Temp;
    }


    if((i==0)| (i % Update_Rate == 0) ){
        Set_Temp_RPM(Set_Temp);
      }

    if(Module_Temp <= Set_Temp+0.05 && Module_Temp >= Set_Temp-0.05){
    	Update_Rate = 10;
    }else{
    	Update_Rate = 10;
    }

    sprintf((char*)buf,"Target RPM : %0.1f ,",Target_RPM);
    HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), 100);
    Read_RPM();
    //Set_RPM(Target_RPM);

    i++;
    HAL_Delay(1000);

  }
  /* USER CODE END 3 */
}




/*
 * @brief Fan Initialization Function
 * @Setting, internal Step size and all, Fan Spin and all
 * @retval None
 */
void Fan_Control_Init(){
	/*
	 *  				Reg_Addr	Value
	 * Fan Config 1		0x32   		0x8C
	 * Fan Cnfig 2		0x33   		0x68
	 * Max Step			0x37   		0x05
	 * Fan Min Drive	0x38  		0x12
	 */
	//uint8_t data_buf[4]={0x8C,0x68,0x05,0x12};// data to write the Register of Fan Controller
	uint8_t config1[2]={0x32,0x8C};
	uint8_t config2[2]={0x33,0x68};
	uint8_t Max_step[2]={0x37,0x05};
	uint8_t Min_drive[2]={0x38,0x12};

	HAL_I2C_Master_Transmit(&hi2c2, FAN_ADDR, config1, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c2, FAN_ADDR, config2, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c2, FAN_ADDR, Max_step, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c2, FAN_ADDR, Min_drive, 2, 100);

	//HAL_I2C_Mem_Write(&hi2c2, FAN_ADDR, 0x32, 8,&data_buf[0], 1, 100);
	//HAL_I2C_Mem_Write(&hi2c2, FAN_ADDR, 0x33, 8,&data_buf[1], 1, 100);
	//HAL_I2C_Mem_Write(&hi2c2, FAN_ADDR, 0x37, 8,&data_buf[2], 1, 100);
	//HAL_I2C_Mem_Write(&hi2c2, FAN_ADDR, 0x38, 8,&data_buf[3], 1, 100);
}

/*
 * @brief Temperature Sensor Configuration for 16bit resolution
 * @ Param 3 : Configuration Register Address
 * @Param  5 : Pointer to data buffer here is the value for 16 bit resolution Ref.ADT7420 Data Sheet
 * @retval None
 */
void Temp_Sensor_Init(){
	uint8_t data_buf[2]={0x03,0x81}; // value for 16 bit resolution

	//HAL_I2C_Mem_Write(&hi2c2, TMP1407_Module_ADDR, 0x03, 8,&data_buf[0], 1, 100);
	//HAL_I2C_Mem_Write(&hi2c2, TMP1407_Room_ADDR, 0x03, 8,&data_buf[0], 1, 100);
	HAL_I2C_Master_Transmit(&hi2c2, TMP1407_Module_ADDR, data_buf, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c2, TMP1407_Room_ADDR, data_buf, 2, 100);

}


/*
 * @brief Read Temperature
 * retval None
 * */
void Read_Temp(void){

	buf[0]=Temp_REG;


	// Read Temperature from Module Assembly
	ret = HAL_I2C_Master_Transmit(&hi2c2,TMP1407_Module_ADDR, buf, 1, 100);

	if(ret != HAL_OK){
		strcpy((char*)buf,"Tx Error for Module Assembly \r\n");
	}else{
		// Read two bytes from the temperature register or Slave register
		ret = HAL_I2C_Master_Receive(&hi2c2, TMP1407_Module_ADDR, buf, 2, 100);

		if(ret != HAL_OK){
			strcpy((char*)buf," Rx Error for Module Assembly \r\n");
		}else{

			//Combine the bytes
			uint16_t reading=0;
			reading = buf[0]<<8;
			reading = reading | buf[1];

			//Convert to Temperature
			//Module_Temp = reading * 0.0078;
			Module_Temp = (reading * 0.0078) - last_Module_Temp;
			Module_Temp = Module_Temp * 0.1 + last_Module_Temp;

			last_Module_Temp = Module_Temp;
			sprintf((char*)buf,"Module Temp = %0.3f ,",Module_Temp);

		}
	}

	HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), 100);


	//Read Temperature from Room Temperature Module
		ret = HAL_I2C_Master_Transmit(&hi2c2,TMP1407_Room_ADDR, buf, 1, 100);

		if(ret != HAL_OK){
			strcpy((char*)buf,"Tx Error for Room Temperature Module \r\n");
		}else{
			// Read two bytes from the temperature register or Slave register
			ret = HAL_I2C_Master_Receive(&hi2c2, TMP1407_Room_ADDR, buf, 2, 100);

			if(ret != HAL_OK){
				strcpy((char*)buf," Rx Error for Room Temperature Module \r\n");
			}else{

				//Combine the bytes
			    uint16_t reading = 0;
				reading = buf[0]<<8;
				reading = reading | buf[1];

				//Convert to Temperature
				Room_Temp = reading * 0.0078;
				//Room_Temp = (reading * 0.0078) - last_Room_Temp;
				//Room_Temp = Room_Temp * 0.1 + last_Room_Temp;

				//last_Room_Temp= Room_Temp;

				sprintf((char*)buf,"Room Temp = %0.3f ,",Room_Temp);

			}
		}

		HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), 100);

}

/*
 * @brief Read RPM from Fan controller EMC2301
 * retval None
 */
void Read_RPM(void){
	static const uint8_t FAN_ADDR = 0x2F<<1;
	uint8_t read_TACH_High_Reg = 0x3E;
	uint8_t read_TACH_Low_Reg = 0x3F;

	uint8_t val_3E=0;
	uint8_t val_3F=0;
	uint16_t val;

	HAL_I2C_Master_Transmit(&hi2c2, FAN_ADDR, &read_TACH_High_Reg, 1, 100);
	if(ret != HAL_OK){
		strcpy((char*)buf,"Error in Tx for 3E Reg\n");
	}else{
		HAL_I2C_Master_Receive(&hi2c2, FAN_ADDR, &val_3E, 1, 100);
		val = val_3E;
		val= val<<8;
	}

	HAL_I2C_Master_Transmit(&hi2c2, FAN_ADDR, &read_TACH_Low_Reg, 1, 100);
	if(ret != HAL_OK){
		strcpy((char*)buf,"Error in Tx for 3F Reg\n");
	}else{
		HAL_I2C_Master_Receive(&hi2c2, FAN_ADDR, &val_3F, 1, 100);
		val= val | val_3F;
		val = val>>3;
	}

	//RPM = 3932160 / val;
	RPM = (3932160 / val) - last_RPM;
	RPM = RPM * 0.1 + last_RPM;
	last_RPM = RPM;

	sprintf((char*)buf,"Actual RPM : %0.1f \n",RPM);

	HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), 100);

}


/*
 * @brief Set Target RPM
 */
void Set_RPM(float Target_RPM){
	char TTH_Reg;
	char TTL_Reg;
	uint16_t TACH;
	TACH = 3932160/Target_RPM;
	TTL_Reg = (TACH<<3) & 0xF8;
	TTH_Reg = (TACH>>5) & 0xFF;

	uint8_t TTL_buf[2];
	TTL_buf[0]=0x3C;
	TTL_buf[1]=TTL_Reg;
	uint8_t TTH_buf[2];
	TTH_buf[0]=0x3D;
	TTH_buf[1]=TTH_Reg;

	HAL_I2C_Master_Transmit(&hi2c2, FAN_ADDR, TTL_buf, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c2, FAN_ADDR, TTH_buf, 2, 100);

	//HAL_I2C_Mem_Write(&hi2c2, FAN_ADDR, 0x3C, 8,(uint8_t*)&TTL_Reg, 1, 100);
	//HAL_I2C_Mem_Write(&hi2c2, FAN_ADDR, 0x3D, 8,(uint8_t*)&TTH_Reg, 1, 100);

}


void Set_Temp_RPM(float Set_Temperature){


  if(Module_Temp <= Set_Temperature+0.04 && Module_Temp >= Set_Temperature-0.04){

      //Target_RPM = Target_RPM;
      if(diff >= 0.010){
        Target_RPM = Target_RPM + 40;
      }else if(diff <= -0.010){
        Target_RPM = Target_RPM - 40;
      }else if(diff >= 0.002){
        Target_RPM = Target_RPM + 20;
      }else if(diff <= -0.002){
        Target_RPM = Target_RPM - 20;
      }else{
        Target_RPM = Target_RPM;
      }
  }else if(Module_Temp > Set_Temperature + 0.04){
      //Target_RPM = Target_RPM + 30;
      if(diff >= 0.01){
        Target_RPM = Target_RPM + 50;
      }else if(diff <= -0.01){
        Target_RPM = Target_RPM - 50;
      }else{
        Target_RPM = Target_RPM + 30;
      }
  }else if(Module_Temp < Set_Temperature - 0.04){
      //Target_RPM = Target_RPM - 30;
      if(diff >= 0.01){
        Target_RPM = Target_RPM + 50;
      }else if(diff <= -0.01){
        Target_RPM = Target_RPM - 50;
      }else{
        Target_RPM = Target_RPM - 30;
      }
  }else{
      Target_RPM = Target_RPM;

  }


  Set_RPM(Target_RPM);



   i=0;


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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c2.Init.Timing = 0x00101A26;
  hi2c2.Init.OwnAddress1 = 160;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin|SMPS_V1_Pin|SMPS_EN_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
