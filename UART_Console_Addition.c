 int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t Buf[100]="Application is Running";
	uint8_t ch[3];



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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  while(! HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
	  HAL_Delay(200);
	  strcpy((char*)Buf,"\n\rIn the Loop");
	  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf),100);
	  */
	  strcpy((char*)Buf,"\n\rEnter number A: ");
	  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf),100);
	  HAL_UART_Receive(&huart1, ch, 2, HAL_MAX_DELAY);
	  int A=atoi((char*)ch);

	  strcpy((char*)Buf,"\n\rEnter number B: ");
	  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf),100);
	  HAL_UART_Receive(&huart1, ch, 2,  HAL_MAX_DELAY);
	  int B=atoi((char*)ch);


	  int sum=A+B;
	  sprintf((char*)Buf,"\n\r Sum : %d",sum);
	  HAL_UART_Transmit(&huart1, Buf, strlen((char*)Buf),100);



  }
  /* USER CODE END 3 */
}
