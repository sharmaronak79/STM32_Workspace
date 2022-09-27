/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Sep 20, 2022
 *      Author: ronakkumar_sharma
 */


#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScalar[8] = {2,4,8,16,32,64,128,256};
uint8_t APB1_PreScalar[4] = {2,4,8,16};


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= (1 << I2C_CR1_START);

}
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){

	uint32_t dummy_read;
	//check for the device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){

		//device is in master mode
		 if(pI2CHandle-> TxRxState == I2C_BUSY_IN_RX){
			 if(pI2CHandle->RxSize == 1){
				 //first disable the clock
				 I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				 //clear the ADDR flag  ( read SR1, read SR2 )
				 dummy_read = pI2CHandle->pI2Cx->SR1;
				 dummy_read = pI2CHandle->pI2Cx->SR2;
				 (void )dummy_read;
			 }
		 }else{
			 //clear the ADDR flag  ( read SR1, read SR2 )
				 dummy_read = pI2CHandle->pI2Cx->SR1;
				 dummy_read = pI2CHandle->pI2Cx->SR2;
				 (void )dummy_read;

		      }


	}else{

		//device is in slave mode
		//clear the ADDR flag  ( read SR1, read SR2 )
		 dummy_read = pI2CHandle->pI2Cx->SR1;
		 dummy_read = pI2CHandle->pI2Cx->SR2;
		 (void )dummy_read;

	}

}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - Enable the clock for particular I2C peripheral
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}

	}else{
		if(pI2Cx == I2C1){
				I2C1_PCLK_DI();
			}else if(pI2Cx == I2C2){
				I2C2_PCLK_DI();
			}else if(pI2Cx == I2C3){
				I2C3_PCLK_DI();
			}
	}

}


uint8_t RCC_GetPLLOutputClock(){

	return 0;
}


uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1 , SystemClk;
	uint8_t clksrc , temp, ahbp , apb1;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0){

		SystemClk = 16000000;
	}else if(clksrc == 1){
		SystemClk = 8000000;
	}else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	//For AHB1 prescalar
	temp = ((RCC -> CFGR >> 4) & 0xF);

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_PreScalar[temp - 8];
	}


	//For APB1 prescalar
	temp = ((RCC -> CFGR >> 10) & 0x7);

	if(temp < 4){
		apb1 = 1;
	}else{
		ahbp = APB1_PreScalar[temp - 4];
	}


	pclk1 = (SystemClk/ahbp) / apb1;

	return pclk1;

}

/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - Initialize the I2C peripheral
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

   //program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations, Clock Control Register
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}


/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - Reset the peripheral I2C, by reset the register value in
 * 						APB1RSTR , here we have defined the macro for the same
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){

	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}



/*********************************************************************
 * @fn      		  - I2C_Peripheral Control
 *
 * @brief             - Enable that particular peripheral , like I2C 1/2/3
 * 						Like here the PE- Peripheral Enable Bit will be 0 or 1 for Disable and Enable
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){

		pI2Cx ->CR1 |= (1<<0);
	}else{
		pI2Cx ->CR1 &= ~(1<<0);
	}
}



/*********************************************************************
 * @fn      		  - I2C_Master SendData
 *
 * @brief             -
 *
 * @param[in]         -here we will need I2C Handle, one buffer where we will keep data to be sent,
 * 					   length of the data and the address of the slave where we want to send the data
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR){

	//1. Generate the start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx); // this is just one helper function

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));


	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )

		I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1

		while( ! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));


	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)

		I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until len becomes 0, before sending any data we have to check the DataRegister that is it Empty or not?
	    while(Len > 0){
			 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // wait till TXE is set
			 pI2CHandle->pI2Cx->DR = *pTxbuffer;
			 *pTxbuffer++;
			 Len--;

		 }


	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF

	    if(SR == I2C_DISABLE_SR){
	    I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
	    }

}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t SR)
{

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1

	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);


		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

		//generate STOP condition
		if(SR == I2C_DISABLE_SR){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{

			//clear the ADDR flag
			I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{


			//wait until RXNE becomes 1
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				if(SR == I2C_DISABLE_SR){
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}

	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}

}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}



uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName)
		{
			return FLAG_SET;
		}
		return FLAG_RESET;
}



uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
	{

		uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX ;
			pI2CHandle->DevAddr = SlaveAddr ;
			pI2CHandle->Sr = Sr ;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;

	}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t SR){

			uint8_t busystate = pI2CHandle->TxRxState;

			if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
			{
				pI2CHandle->pRxBuffer = pRxBuffer;
				pI2CHandle->RxLen = Len;
				pI2CHandle->TxRxState = I2C_BUSY_IN_RX ;
				pI2CHandle->DevAddr = SlaveAddr ;
				pI2CHandle->Sr = SR ;

				//Implement code to Generate START Condition
				I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

				//Implement the code to enable ITBUFEN Control Bit
				pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

				//Implement the code to enable ITEVFEN Control Bit
				pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

				//Implement the code to enable ITERREN Control Bit
				pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

			}

			return busystate;
}



void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 6 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
			}
		}
}


void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section  = IRQNumber %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

		*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );


}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){

		//Interrupt handling for master and slave m ode of a device
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	//1. Handle for interrupt generated by SB event
	// NOte: SB flag is not only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	if(temp1 && temp3){
		//This interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block ;ets execute the address phase

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){

			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx , pI2CHandle->DevAddr);

		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx , pI2CHandle->DevAddr);

		}


	}

	//2.Handle for interrupt generated by ADDR event
	//Note : when master mode: Address is sent
	//       when slave mode: Address match with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	if(temp1 && temp3){
		//interrupt is generated because of ADDR event
		// ADDR flag is set, now when ADDR flag is set, Clock will be stretched and Master or Slave will go in busy state
		I2C_ClearADDRFlag(pI2CHandle);

	}

	//3.Handle for interrupt generated by BTF(Byte transfer finished ) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

	if(temp1 && temp3){
		// BTF flag is set

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){

				//make sure that TXE is also set
				if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){

					//BTF and TXE = 1, so we will close the transmission

					if(pI2CHandle->TxLen == 0){
							//1. generate the STOP condition, we can do that only when repeated start is disable
							if(pI2CHandle->Sr == I2C_DISABLE_SR)
							{
								I2C_GenerateStopCondition(pI2CHandle-> pI2Cx);
							}

							//2. reset all the member elements of the handle structure

								I2C_CloseSendData(pI2CHandle);

							//3. notify the application transmission is complete
							I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
						   }
					} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
									;
							  }

					}
	}





	//4. Handle for interrupt generated by STOPF event
	//Note : STOP detection flag is applicable only slave mode . For master  this flag will never be set
	//the below code block will not be executed by the master since STOPf will not set in master mode

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3){
			// STOPF flag is set
			// clear the STOPF ,i.e  First, read SR1 then write to CR1

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}





	//5. Handle for Interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3){

		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			// when TXE flag is set , that means Data Register is empty, and we have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);

			}
		}
	}




	//6. Handle for Interrupt generated by RXNE event
		temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);

		if(temp1 && temp2 && temp3){

			//Check for the device mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
					//the device is Master
					// RXNE flag is set
				if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

					I2C_MasterHandleRXNEInterrupt(pI2CHandle);
				 }
			}

		}
}



static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){

	if(pI2CHandle->TxLen > 0){

						//1. load the data in to DR
						pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

						//2. decrement the TxLen
						pI2CHandle-> TxLen--;


						//3. Increment the buffer address
						pI2CHandle->pTxBuffer++;

					}

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	// we have to do the data reception
			if (pI2CHandle->RxSize == 1){

				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

				pI2CHandle->RxLen--;

			}

			if(pI2CHandle->RxSize > 1){

				if(pI2CHandle->RxLen == 2){
					//clear the ACK bit
					I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				}

				//read DR
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
				pI2CHandle->pRxBuffer++;
				pI2CHandle->RxLen--;


			}

			if(pI2CHandle->RxLen == 0){

				//close the I2C data reception and notify the aplication


				//1. generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR){
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
				//2. CLose the I2C Rx
					I2C_CloseReceiveData(pI2CHandle);

				//3. Notify the Application
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

			}
}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){

	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(  1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){

	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(  1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}




void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag

		//Implement the code to notify the application about the error

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag

		//Implement the code to notify the application about the error
	}

}


